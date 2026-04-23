from flask import Flask, jsonify, request, send_from_directory
import subprocess
import os
import sys
import signal
import atexit
import struct
import time
import json
import math
from datetime import datetime

from hardware_interface_3_28 import get_sensor_data, get_shm_age_ms

app = Flask(__name__)

PROGRAMS = {
    "sensors":    "sensors_3_28.py",
    "ultrasonic": "ultrasonic_bg.py",
    "pid":        "deployPID_3_28.py",
    "motorwasd":  "motor_wasd.py",
}

PROGRAM_PROCS = {name: None for name in PROGRAMS}


def _stop_all_on_exit():
    """Kill all managed subprocesses when the webapp exits (Ctrl+C, sys.exit, crash).
    Without this, os.setsid() subprocesses survive the Flask process and run as
    orphans — causing duplicate SHM writers, db lock conflicts, and calibration failures
    on the next webapp start."""
    for name in list(PROGRAMS.keys()):
        proc = PROGRAM_PROCS.get(name)
        if proc is None or proc.poll() is not None:
            continue
        try:
            if os.name != "nt":
                os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
            else:
                proc.terminate()
            proc.wait(timeout=3.0)
        except Exception:
            try:
                proc.kill()
            except Exception:
                pass
        finally:
            PROGRAM_PROCS[name] = None


atexit.register(_stop_all_on_exit)


# Set when start_program fails (missing script, Popen error, or child exited immediately)
LAST_PROGRAM_START_ERROR = None

# Cache file paths (same directory as this script)
_DIR = os.path.dirname(os.path.abspath(__file__))
OBS_CACHE_FILE        = os.path.join(_DIR, "obs_cache.bin")
ULTRASONIC_CACHE_FILE = os.path.join(_DIR, "ultrasonic_cache.bin")
MOTOR_CMD_FILE        = os.path.join(_DIR, "motor_command.json")
MOTOR_STATE_FILE      = os.path.join(_DIR, "motor_state.bin")
SENSOR_STATS_FILE     = os.path.join(_DIR, "sensor_stats.bin")
US_STATS_FILE         = os.path.join(_DIR, "us_stats.bin")
PID_GAINS_FILE        = os.path.join(_DIR, "pid_gains.json")

_pid_start_time = None


def _read_pid_timing_stats():
    """Parse last 100 data rows from the most recent PID log file for dt_ms stats."""
    try:
        import glob as _glob
        logs = sorted(_glob.glob(os.path.join(_DIR, "robot_pid_new_log_*.txt")))
        if not logs:
            return None
        with open(logs[-1], "r") as f:
            lines = f.readlines()
        dt_vals = []
        for line in reversed(lines):
            line = line.strip()
            if not line or line.startswith("timestamp"):
                continue
            parts = line.split(",")
            if len(parts) >= 10:
                try:
                    dt_vals.append(float(parts[9]))
                except ValueError:
                    pass
            if len(dt_vals) >= 100:
                break
        if not dt_vals:
            return None
        avg = sum(dt_vals) / len(dt_vals)
        return {
            "hz":     round(1000.0 / avg, 1) if avg > 0 else None,
            "avg_ms": round(avg, 2),
            "min_ms": round(min(dt_vals), 2),
            "max_ms": round(max(dt_vals), 2),
        }
    except Exception:
        return None


# Joystick throttling
last_joystick_send = 0
joystick_throttle_interval = 0.25  # seconds


def log_to_file(tag, message):
    """Log events to numbers.txt with timestamp"""
    try:
        with open(os.path.join(_DIR, "numbers.txt"), "a") as f:
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
            f.write(f"{timestamp} | {tag} | {message}\n")
    except Exception as e:
        print(f"Error writing to numbers.txt: {e}")


# ============================
# GENERIC PROGRAM MANAGEMENT
# ============================
def is_program_running(name):
    proc = PROGRAM_PROCS.get(name)
    return proc is not None and proc.poll() is None


def _clear_dead_program_slot(name):
    """Drop finished Popen handles so a new start is not blocked."""
    proc = PROGRAM_PROCS.get(name)
    if proc is None:
        return
    try:
        if proc.poll() is not None:
            PROGRAM_PROCS[name] = None
    except Exception:
        PROGRAM_PROCS[name] = None


def _tail_file_utf8(path, max_bytes=3000):
    try:
        with open(path, "rb") as f:
            f.seek(0, os.SEEK_END)
            sz = f.tell()
            f.seek(max(0, sz - max_bytes))
            return f.read().decode("utf-8", errors="replace").strip()
    except Exception:
        return ""


def _kill_orphans(script_name):
    """Kill any orphaned instances of a script left over from a previous webapp
    session.  Without this, a new sensor/PID process would collide with the
    orphan on I2C / SHM and fail to calibrate or control."""
    if os.name == "nt":
        return
    try:
        subprocess.run(
            ["pkill", "-f", script_name],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            timeout=3,
        )
        time.sleep(0.3)
    except Exception:
        pass


def start_program(name):
    global LAST_PROGRAM_START_ERROR
    LAST_PROGRAM_START_ERROR = None

    if name not in PROGRAMS:
        LAST_PROGRAM_START_ERROR = f"Unknown program {name!r}"
        log_to_file("ERROR", LAST_PROGRAM_START_ERROR)
        return False

    _clear_dead_program_slot(name)

    if is_program_running(name):
        return True

    # Kill any orphaned process from a previous webapp session before launching.
    script = PROGRAMS[name]
    _kill_orphans(script)

    # If starting sensors, flush the cached SHM reader so it reconnects to the
    # new SHM that sensors_3_28.py will create (the old one is unlinked on startup).
    if name == "sensors":
        try:
            from hardware_interface_3_28 import _close_shm_reader
            _close_shm_reader()
        except Exception:
            pass

    script_path = os.path.join(_DIR, script)
    if not os.path.isfile(script_path):
        LAST_PROGRAM_START_ERROR = f"Script not found: {script_path}"
        log_to_file("ERROR", LAST_PROGRAM_START_ERROR)
        return False

    stderr_log = os.path.join(_DIR, f"{name}_stderr.log")
    cmd = [sys.executable, "-u", script_path]

    popen_kw = dict(
        cwd=_DIR,
        stdout=subprocess.DEVNULL,
    )
    if os.name != "nt":
        popen_kw["preexec_fn"] = os.setsid
    else:
        popen_kw["creationflags"] = subprocess.CREATE_NEW_PROCESS_GROUP

    def _launch(err_fp):
        return subprocess.Popen(cmd, stderr=err_fp, **popen_kw)

    try:
        with open(stderr_log, "ab", buffering=0) as err_f:
            try:
                PROGRAM_PROCS[name] = _launch(err_f)
            except OSError:
                if os.name == "nt" and popen_kw.pop("creationflags", None) is not None:
                    PROGRAM_PROCS[name] = _launch(err_f)
                else:
                    raise
    except Exception as e:
        LAST_PROGRAM_START_ERROR = f"Popen failed: {e}"
        log_to_file("ERROR", f"Failed to start program '{name}': {e}")
        PROGRAM_PROCS[name] = None
        return False

    time.sleep(0.2)
    proc = PROGRAM_PROCS.get(name)
    if proc is None:
        LAST_PROGRAM_START_ERROR = "Internal error: process slot empty after launch"
        return False

    exit_code = proc.poll()
    if exit_code is not None:
        PROGRAM_PROCS[name] = None
        tail = _tail_file_utf8(stderr_log, 4000)
        if tail:
            LAST_PROGRAM_START_ERROR = (
                f"Exited immediately with code {exit_code}. Stderr (see {name}_stderr.log):\n"
                f"{tail[-1800:]}"
            )
        else:
            LAST_PROGRAM_START_ERROR = (
                f"Exited immediately with code {exit_code}. Check {name}_stderr.log in {_DIR}"
            )
        log_to_file("ERROR", f"Program '{name}' {LAST_PROGRAM_START_ERROR[:400]}")
        return False

    log_to_file(
        "SYSTEM",
        f"Program '{name}' started: {sys.executable} {script_path} cwd={_DIR}",
    )
    return True


def _start_failure_reason():
    return LAST_PROGRAM_START_ERROR


# 8-byte double timestamp + 9 float32 — same as hardware_interface_3_28 / sensors_3_28
_OBS_CACHE_MIN_BYTES = 8 + 9 * 4


def _read_obs_cache_relaxed(target_velocity=0.0, target_rotation_rate=0.0, max_age_s=5.0):
    """
    Read obs_cache.bin without the 50ms PID freshness gate.
    Used for /sensor_data when get_sensor_data() is None (scheduler jitter, web poll rate, etc.).
    """
    try:
        if not os.path.isfile(OBS_CACHE_FILE):
            return None
        with open(OBS_CACHE_FILE, "rb") as f:
            raw = f.read()
        if len(raw) < _OBS_CACHE_MIN_BYTES:
            return None
        ts, = struct.unpack_from("<d", raw, 0)
        obs = list(struct.unpack_from("<9f", raw, 8))
        age_s = time.monotonic() - ts
        if max_age_s is not None and age_s > float(max_age_s):
            return None
        if len(obs) >= 8:
            obs[6] = float(target_velocity) - float(obs[0])
            obs[7] = float(target_rotation_rate) - float(obs[3])
        return obs
    except Exception:
        return None


def stop_program(name):
    if not is_program_running(name):
        return
    try:
        proc = PROGRAM_PROCS[name]
        if os.name != "nt":
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        else:
            proc.terminate()
        # Wait up to 5 s for the process to fully exit before returning.
        # This prevents GPIO/SHM conflicts if the program is restarted immediately.
        try:
            proc.wait(timeout=5.0)
        except Exception:
            try:
                proc.kill()
            except Exception:
                pass
        log_to_file("SYSTEM", f"Program '{name}' stopped")
    except Exception as e:
        log_to_file("ERROR", f"Failed to stop program '{name}': {e}")
    finally:
        PROGRAM_PROCS[name] = None


# Legacy helpers
def is_imu_running():
    return is_program_running("imu")


def start_imu():
    return start_program("imu")


def stop_imu():
    stop_program("imu")


def is_motor_test_running():
    return is_program_running("motortest")


def start_motor_test():
    return start_program("motortest")


def stop_motor_test():
    stop_program("motortest")


# =======================
# PARSE SENSOR LINE
# =======================
def parse_sensor_line(line):
    result = {
        "IMU1": {
            "Time": "",
            "Forward/backwards Tilt": "",
            "Side-to-Side Tilt": "",
            "Yaw": "",
            "Pitch Rate": "",
            "Roll Rate": "",
            "Rotational Velocity": "",
        },
        "IMU1Linear": {
            "Linear Velocity": "",
            "X velocity": "",
            "Y velocity": "",
        },
        "EncoderL": {
            "Speed": "",
            "Direction": "",
        },
        "EncoderR": {
            "Speed": "",
            "Direction": "",
        },
        "Robot": {
            "Yaw Rate": "",
        },
        "Pendulum": {
            "Angular Velocity": "",
            "Angle": "",
            "AngleDeg": "",
        },
        "Ultrasonic": {
            "Right": "",
            "Left": "",
        },
    }

    def num2sig(x):
        try:
            v = float(x)
        except Exception:
            return x
        return f"{v:.2g}"

    try:
        tokens = [tok.strip() for tok in line.split(",")]
        idx = 0

        if idx < len(tokens) and ":" in tokens[idx]:
            result["IMU1"]["Time"] = tokens[idx]
            idx += 1

        if idx < len(tokens) and tokens[idx] == "IMU1":
            idx += 1
            imu1_map = [
                ("Forward/backwards", "Forward/backwards Tilt"),
                ("Side-to-Side", "Side-to-Side Tilt"),
                ("Yaw", "Yaw"),
                ("Pitch Rate", "Pitch Rate"),
                ("Roll Rate", "Roll Rate"),
                ("Rotational Velocity", "Rotational Velocity"),
            ]
            for label, key in imu1_map:
                if idx + 1 < len(tokens) and tokens[idx] == label:
                    result["IMU1"][key] = num2sig(tokens[idx + 1])
                    idx += 2

        if idx < len(tokens) and tokens[idx] == "IMU1 Linear Velocity":
            idx += 1
            if idx < len(tokens):
                result["IMU1Linear"]["Linear Velocity"] = num2sig(tokens[idx])
                idx += 1

        if idx < len(tokens) and tokens[idx] == "IMU1's X-velocity":
            idx += 1
            if idx < len(tokens):
                result["IMU1Linear"]["X velocity"] = num2sig(tokens[idx])
                idx += 1

        if idx < len(tokens) and tokens[idx] == "IMU1's Y-velocity":
            idx += 1
            if idx < len(tokens):
                result["IMU1Linear"]["Y velocity"] = num2sig(tokens[idx])
                idx += 1

        if idx < len(tokens) and tokens[idx] == "Robot Yaw Rate":
            idx += 1
            if idx < len(tokens):
                result["Robot"]["Yaw Rate"] = num2sig(tokens[idx])
                idx += 1

        if idx < len(tokens) and tokens[idx] == "Pendulum Angular Velocity":
            idx += 1
            if idx < len(tokens):
                result["Pendulum"]["Angular Velocity"] = num2sig(tokens[idx])
                idx += 1

        if idx < len(tokens) and tokens[idx] == "Pendulum Angle":
            idx += 1
            if idx < len(tokens):
                result["Pendulum"]["Angle"] = num2sig(tokens[idx])
                idx += 1

        if idx < len(tokens) and tokens[idx] == "Pendulum Angle (deg)":
            idx += 1
            if idx < len(tokens):
                result["Pendulum"]["AngleDeg"] = num2sig(tokens[idx])
                idx += 1

        if idx < len(tokens) and tokens[idx] == "EncoderL":
            idx += 1
            if idx < len(tokens):
                result["EncoderL"]["Speed"] = num2sig(tokens[idx])
                idx += 1
            if idx < len(tokens) and tokens[idx] == "Direction":
                idx += 1
                if idx < len(tokens):
                    result["EncoderL"]["Direction"] = tokens[idx]
                    idx += 1

        if idx < len(tokens) and tokens[idx] == "EncoderR":
            idx += 1
            if idx < len(tokens):
                result["EncoderR"]["Speed"] = num2sig(tokens[idx])
                idx += 1
            if idx < len(tokens) and tokens[idx] == "Direction":
                idx += 1
                if idx < len(tokens):
                    result["EncoderR"]["Direction"] = tokens[idx]
                    idx += 1

        if idx < len(tokens) and tokens[idx] == "Ultrasonic Right":
            idx += 1
            if idx < len(tokens):
                val_num = tokens[idx].replace("cm", "").strip()
                result["Ultrasonic"]["Right"] = num2sig(val_num)
                idx += 1

        if idx < len(tokens) and tokens[idx] == "Ultrasonic Left":
            idx += 1
            if idx < len(tokens):
                val_num = tokens[idx].replace("cm", "").strip()
                result["Ultrasonic"]["Left"] = num2sig(val_num)
                idx += 1

    except Exception as e:
        log_to_file("ERROR", f"parse_sensor_line error: {e}")

    return result


# ==============
# FLASK ROUTES
# ==============
@app.route("/sensor_feed")
def sensor_feed():
    try:
        with open(os.path.join(_DIR, "sensor_data.txt"), "r") as f:
            line = f.readline().strip()
            return jsonify(parse_sensor_line(line))
    except Exception:
        return jsonify(parse_sensor_line(""))


@app.route("/sensor_on", methods=["POST"])
def sensor_on():
    ok = start_imu()
    return jsonify({"status": "ON" if ok else "ERROR"})


@app.route("/sensor_off", methods=["POST"])
def sensor_off():
    stop_imu()
    return jsonify({"status": "OFF"})


@app.route("/get_number_feed")
def get_number_feed():
    return jsonify({"value": 42})


@app.route("/direction_ajax", methods=["POST"])
def direction_ajax():
    global last_joystick_send
    current_time = time.time()
    data = request.get_json()
    x = float(data.get("x", 0.0))
    y = float(data.get("y", 0.0))
    msg = f"Joystick x={x:.2f}, y={y:.2f}"
    if current_time - last_joystick_send >= joystick_throttle_interval:
        last_joystick_send = current_time
        log_to_file("JOYSTICK", msg)
    # Write to motor_command.json so motorwasd picks it up when running
    try:
        speed = float(data.get("speed", 0.5))
        cmd = {"fwd": y, "turn": x, "speed": speed, "ts": time.monotonic()}
        tmp = MOTOR_CMD_FILE + ".tmp"
        with open(tmp, "w") as f:
            json.dump(cmd, f)
        os.replace(tmp, MOTOR_CMD_FILE)
    except Exception:
        pass
    return jsonify({"message": msg})


@app.route("/get_top_row", methods=["POST"])
def get_top_row():
    log_to_file("ACTION", "Remove Row pressed")
    return jsonify({"message": "Removed top row!"})


@app.route("/start_pigpiod", methods=["POST"])
def start_pigpiod_route():
    try:
        result = subprocess.run(
            ["sudo", "pigpiod"],
            capture_output=True, text=True, timeout=5
        )
        if result.returncode == 0 or "already running" in result.stderr.lower():
            log_to_file("SYSTEM", "pigpiod started")
            return jsonify({"status": "started"})
        else:
            msg = result.stderr.strip() or result.stdout.strip() or "unknown error"
            log_to_file("ERROR", f"pigpiod failed: {msg}")
            return jsonify({"status": "failed", "reason": msg})
    except Exception as e:
        log_to_file("ERROR", f"pigpiod exception: {e}")
        return jsonify({"status": "failed", "reason": str(e)})


@app.route("/start_motor_test", methods=["POST"])
def start_motor_test_route():
    ok = start_motor_test()
    return jsonify({"status": "started" if ok else "failed"})


@app.route("/stop_motor_test", methods=["POST"])
def stop_motor_test_route():
    stop_motor_test()
    return jsonify({"status": "stopped"})


@app.route("/start_autonav", methods=["POST"])
def start_autonav_route():
    ok = start_program("autonav")
    return jsonify({
        "status": "started" if ok else "failed",
        "reason": None if ok else _start_failure_reason(),
    })


@app.route("/stop_autonav", methods=["POST"])
def stop_autonav_route():
    stop_program("autonav")
    return jsonify({"status": "stopped"})


# ==================================
# SENSOR PIPELINE ROUTES
# ==================================

@app.route("/sensor_data")
def sensor_data():
    """
    Observation vector via hardware_interface_3_28.get_sensor_data (SHM → cache → DB).
    Ultrasonic distances still read from ultrasonic_cache.bin (not in obs vector).
    """
    sensors_running = is_program_running("sensors")
    result = {
        "linear_velocity":    None,
        "pitch_deg":          None,
        "pitch_rate":         None,
        "yaw_rate":           None,
        "wheel_left_rads":    None,
        "wheel_right_rads":   None,
        "velocity_error":     None,
        "rotation_error":     None,
        "yaw_deg":            None,
        "cache_age_ms":       None,
        "ultrasonic_right_cm": None,
        "ultrasonic_left_cm":  None,
        "us_age_ms":          None,
        "sensors_running":    sensors_running,
        "ultrasonic_running": is_program_running("ultrasonic"),
        "pid_running":        is_program_running("pid"),
        "motorwasd_running":  is_program_running("motorwasd"),
        "autonav_running":    is_program_running("autonav"),
        "calibrated":         False,
        "obs_source":         None,
        "motor_left":         0.0,
        "motor_right":        0.0,
        "pid_elapsed_s":      None,
    }

    # Use SHM age as the calibrated signal — SHM is zeroed (ts=0.0) the instant
    # sensors starts, so get_shm_age_ms() returns None throughout calibration and
    # only returns a value once the main loop is actively writing post-calibration.
    # This eliminates the race condition with sensor_stats.bin persisting on disk
    # from the previous session during the Python startup window (~200-400 ms on Pi).
    shm_age = get_shm_age_ms() if sensors_running else None
    is_calibrated = shm_age is not None
    result["calibrated"] = is_calibrated

    tv, tr = 0.0, 0.0
    obs = None
    if is_calibrated:
        obs = get_sensor_data(target_velocity=tv, target_rotation_rate=tr)
        result["obs_source"] = "hardware_interface" if obs is not None else None

        if obs is None:
            obs = _read_obs_cache_relaxed(tv, tr, max_age_s=5.0)
            if obs is not None:
                result["obs_source"] = "cache_relaxed"

    if obs is not None and len(obs) >= 9:
        result["linear_velocity"]  = round(float(obs[0]), 4)
        result["pitch_deg"]        = round(math.degrees(float(obs[1])), 3)
        result["pitch_rate"]       = round(float(obs[2]), 4)
        result["yaw_rate"]         = round(float(obs[3]), 4)
        result["wheel_left_rads"]  = round(float(obs[4]), 4)
        result["wheel_right_rads"] = round(float(obs[5]), 4)
        result["velocity_error"]   = round(float(obs[6]), 4)
        result["rotation_error"]   = round(float(obs[7]), 4)
        result["yaw_deg"]          = round(math.degrees(float(obs[8])), 3)

    if shm_age is not None:
        result["cache_age_ms"] = round(shm_age, 2)

    # --- ultrasonic_cache.bin (16 bytes: float64 ts + 2 × float32) ---
    try:
        with open(ULTRASONIC_CACHE_FILE, "rb") as f:
            raw = f.read()
        if len(raw) >= 16:
            us_ts,      = struct.unpack_from("<d",  raw, 0)
            right, left  = struct.unpack_from("<ff", raw, 8)
            us_age_ms = (time.monotonic() - us_ts) * 1000.0
            result["ultrasonic_right_cm"] = round(float(right), 1) if right >= 0 else None
            result["ultrasonic_left_cm"]  = round(float(left),  1) if left  >= 0 else None
            result["us_age_ms"]           = round(us_age_ms, 2)
    except Exception:
        pass

    # --- motor_state.bin (float64 ts + 2 × float32) ---
    try:
        with open(MOTOR_STATE_FILE, "rb") as f:
            raw = f.read()
        if len(raw) >= 16:
            ms_ts, ml, mr = struct.unpack_from("<dff", raw, 0)
            if time.monotonic() - ms_ts < 0.5:
                result["motor_left"]  = round(float(ml), 4)
                result["motor_right"] = round(float(mr), 4)
    except Exception:
        pass

    # --- sensor_stats.bin ---
    try:
        with open(SENSOR_STATS_FILE, "rb") as f:
            raw = f.read()
        if len(raw) >= 44:
            ts, main_hz, imu_hz, enc_hz, pass_rate, _ = struct.unpack_from("<dffffd", raw, 0)
            succ, fail, total = struct.unpack_from("<III", raw, 32)
            age_s = time.monotonic() - ts
            if age_s < 5.0:
                result["sensor_stats"] = {
                    "main_hz":   round(main_hz, 1),
                    "imu_hz":    round(imu_hz, 1),
                    "enc_hz":    int(enc_hz),
                    "pass_rate": round(pass_rate, 1),
                    "successful": int(succ),
                    "failed":     int(fail),
                    "total":      int(total),
                    "age_s":      round(age_s, 1),
                }
    except Exception:
        pass

    # --- us_stats.bin ---
    try:
        with open(US_STATS_FILE, "rb") as f:
            raw = f.read()
        if len(raw) >= 28:
            ts, loop_hz, r_pct, l_pct, _ = struct.unpack_from("<dffff", raw, 0)
            cyc, = struct.unpack_from("<I", raw, 24)
            age_s = time.monotonic() - ts
            if age_s < 5.0:
                result["us_stats"] = {
                    "loop_hz":     round(loop_hz, 1),
                    "right_ok":    round(r_pct, 1),
                    "left_ok":     round(l_pct, 1),
                    "cycles":      int(cyc),
                    "age_s":       round(age_s, 1),
                }
    except Exception:
        pass

    # --- PID run timer ---
    if _pid_start_time is not None and is_program_running("pid"):
        result["pid_elapsed_s"] = round(time.monotonic() - _pid_start_time, 1)

    # --- PID loop timing stats ---
    result["pid_timing"] = _read_pid_timing_stats() if is_program_running("pid") else None

    return jsonify(result)


@app.route("/start_sensor_suite", methods=["POST"])
def start_sensor_suite_route():
    if is_program_running("sensors") and is_program_running("ultrasonic"):
        return jsonify({"status": "already_running", "sensors_running": True, "ultrasonic_running": True})
    ok_sensors = start_program("sensors")
    if not ok_sensors:
        return jsonify({
            "status": "failed", "reason": _start_failure_reason(),
            "sensors_running": False, "ultrasonic_running": False,
        })
    ok_ultrasonic = start_program("ultrasonic")
    return jsonify({
        "status": "started",
        "sensors_running": is_program_running("sensors"),
        "ultrasonic_running": is_program_running("ultrasonic"),
        "reason": None if ok_ultrasonic else _start_failure_reason(),
    })


@app.route("/stop_sensor_suite", methods=["POST"])
def stop_sensor_suite_route():
    stop_program("sensors")
    stop_program("ultrasonic")
    return jsonify({"status": "stopped", "sensors_running": False, "ultrasonic_running": False})


@app.route("/start_sensors", methods=["POST"])
def start_sensors_route():
    if is_program_running("sensors"):
        return jsonify({"status": "already_running", "running": True, "reason": None})
    ok = start_program("sensors")
    return jsonify({
        "status": "started" if ok else "failed",
        "running": is_program_running("sensors"),
        "reason": None if ok else _start_failure_reason(),
    })


@app.route("/stop_sensors", methods=["POST"])
def stop_sensors_route():
    stop_program("sensors")
    return jsonify({"status": "stopped", "running": False})


@app.route("/start_ultrasonic", methods=["POST"])
def start_ultrasonic_route():
    ok = start_program("ultrasonic")
    return jsonify({
        "status": "started" if ok else "failed",
        "running": is_program_running("ultrasonic"),
        "reason": None if ok else _start_failure_reason(),
    })


@app.route("/stop_ultrasonic", methods=["POST"])
def stop_ultrasonic_route():
    stop_program("ultrasonic")
    return jsonify({"status": "stopped", "running": False})


@app.route("/start_pid", methods=["POST"])
def start_pid_route():
    global _pid_start_time
    if is_program_running("motorwasd"):
        return jsonify({"status": "blocked", "reason": "Stop manual motor control first",
                        "running": False})
    ok = start_program("pid")
    if ok:
        _pid_start_time = time.monotonic()
    return jsonify({
        "status": "started" if ok else "failed",
        "running": is_program_running("pid"),
        "reason": None if ok else _start_failure_reason(),
    })


@app.route("/reset_motor_state", methods=["POST"])
def reset_motor_state_route():
    """Write a zeroed motor_state.bin so the UI shows 0 after a manual reset."""
    try:
        buf = struct.pack("<dff", 0.0, 0.0, 0.0)   # ts=0 → will be treated as stale by poller
        tmp = MOTOR_STATE_FILE + ".tmp"
        with open(tmp, "wb") as f:
            f.write(buf)
        os.replace(tmp, MOTOR_STATE_FILE)
    except Exception:
        pass
    return jsonify({"status": "ok"})


@app.route("/stop_pid", methods=["POST"])
def stop_pid_route():
    global _pid_start_time
    stop_program("pid")
    _pid_start_time = None
    # Zero the motors immediately so they don't coast after PID exits
    try:
        from hardware_interface_3_28 import set_motor_velocities
        set_motor_velocities(0.0, 0.0)
    except Exception:
        pass
    return jsonify({"status": "stopped", "running": False})


@app.route("/get_pid_gains", methods=["GET"])
def get_pid_gains_route():
    try:
        with open(PID_GAINS_FILE, "r") as f:
            gains = json.load(f)
        return jsonify({"status": "ok", "gains": gains})
    except FileNotFoundError:
        return jsonify({"status": "defaults", "gains": {
            "kp": 60.0, "kd": 12.0, "ki": 0.0, "trim_deg": 0.0, "tip_deg": 35.0
        }})
    except Exception as e:
        return jsonify({"status": "error", "reason": str(e)})


@app.route("/set_pid_gains", methods=["POST"])
def set_pid_gains_route():
    data = request.get_json(silent=True) or {}
    gains = {
        "kp":       float(data.get("kp",       60.0)),
        "kd":       float(data.get("kd",       12.0)),
        "ki":       float(data.get("ki",        0.0)),
        "trim_deg": float(data.get("trim_deg",  0.0)),
        "tip_deg":  float(data.get("tip_deg",  35.0)),
    }
    try:
        tmp = PID_GAINS_FILE + ".tmp"
        with open(tmp, "w") as f:
            json.dump(gains, f)
        os.replace(tmp, PID_GAINS_FILE)
        log_to_file("PID", f"Gains updated: {gains}")
        return jsonify({"status": "ok", "gains": gains})
    except Exception as e:
        return jsonify({"status": "error", "reason": str(e)})


# ==================================
# MANUAL MOTOR CONTROL (WASD)
# ==================================

@app.route("/motor_on", methods=["POST"])
def motor_on_route():
    if is_program_running("pid"):
        return jsonify({"status": "blocked", "reason": "Stop PID first",
                        "running": False})
    _write_motor_cmd(0.0, 0.0, 0.5)
    ok = start_program("motorwasd")
    return jsonify({
        "status": "started" if ok else "failed",
        "running": is_program_running("motorwasd"),
        "reason": None if ok else _start_failure_reason(),
    })


@app.route("/motor_off", methods=["POST"])
def motor_off_route():
    _write_motor_cmd(0.0, 0.0, 0.0)
    stop_program("motorwasd")
    return jsonify({"status": "stopped", "running": False})


@app.route("/motor_cmd", methods=["POST"])
def motor_cmd_route():
    data  = request.get_json(silent=True) or {}
    fwd   = float(data.get("fwd",   0.0))
    turn  = float(data.get("turn",  0.0))
    speed = float(data.get("speed", 0.5))
    _write_motor_cmd(fwd, turn, speed)
    return jsonify({"fwd": fwd, "turn": turn, "speed": speed})


def _write_motor_cmd(fwd: float, turn: float, speed: float):
    cmd = {"fwd": fwd, "turn": turn, "speed": speed, "ts": time.monotonic()}
    try:
        tmp = MOTOR_CMD_FILE + ".tmp"
        with open(tmp, "w") as f:
            json.dump(cmd, f)
        os.replace(tmp, MOTOR_CMD_FILE)
    except Exception as e:
        log_to_file("ERROR", f"motor_cmd write failed: {e}")


@app.route("/robot.png")
def serve_robot_png():
    return send_from_directory(_DIR, "robot.png")

@app.route("/background.jpg")
def serve_background():
    return send_from_directory(_DIR, "background.jpg")

@app.route("/presentation.pdf")
def serve_presentation():
    return send_from_directory(_DIR, "404 Status Update 5.pdf")

def _serve_video(filename):
    """Serve a video file with HTTP range request support for browser playback."""
    from flask import Response
    path = os.path.join(_DIR, filename)
    file_size = os.path.getsize(path)
    range_header = request.headers.get("Range", None)
    if range_header:
        byte_start, byte_end = 0, file_size - 1
        match = __import__("re").search(r"bytes=(\d+)-(\d*)", range_header)
        if match:
            byte_start = int(match.group(1))
            byte_end   = int(match.group(2)) if match.group(2) else file_size - 1
        length = byte_end - byte_start + 1
        with open(path, "rb") as f:
            f.seek(byte_start)
            data = f.read(length)
        resp = Response(data, 206, mimetype="video/mp4")
        resp.headers["Content-Range"]  = f"bytes {byte_start}-{byte_end}/{file_size}"
        resp.headers["Accept-Ranges"]  = "bytes"
        resp.headers["Content-Length"] = str(length)
        return resp
    with open(path, "rb") as f:
        data = f.read()
    resp = Response(data, 200, mimetype="video/mp4")
    resp.headers["Accept-Ranges"]  = "bytes"
    resp.headers["Content-Length"] = str(file_size)
    return resp

@app.route("/vid1mp4.mp4")
def serve_vid1():
    return _serve_video("vid1mp4.mp4")

@app.route("/vid2mp4.mp4")
def serve_vid2():
    return _serve_video("vid2mp4.mp4")

@app.route("/graph1.jpg")
def serve_graph1():
    return send_from_directory(_DIR, "graph1.jpg")

@app.route("/graph2.jpg")
def serve_graph2():
    return send_from_directory(_DIR, "graph2.jpg")

@app.route("/")
def home():
    log_to_file("SYSTEM", "Web interface loaded")
    return """
<!DOCTYPE html>
<html>
<head>
    <meta charset="utf-8">
    <title>Inverted Pendulum Robot Control Hub</title>
    <script src="https://cdn.jsdelivr.net/npm/chart.js@4.4.0/dist/chart.umd.min.js"></script>
    <style>
        :root {
            --panel: #101820;
            --accent: #29f0ff;
            --text: #d7f8ff;
            --warn: #ffdd57;
            --ok: #8dff8a;
            --bad: #ff5560;
            --bg: #0b0f14;
            --grid-gap: 10px;
        }

        body {
            margin: 0; padding: 0;
            background: var(--bg);
            color: var(--text);
            font-family: system-ui, sans-serif;
            height: 100vh;
            display: flex; flex-direction: column;
        }

        .top-bar {
            background: #111922;
            padding: 8px 12px;
            box-shadow: 0 2px 10px #000a;
            display: flex; flex-direction: column; gap: 4px;
        }

        .top-row, .bottom-row {
            display: flex; align-items: center;
            justify-content: space-between; gap: 8px;
        }

        .top-row-left, .top-row-right,
        .bottom-row-left, .bottom-row-right {
            display: flex; align-items: center; gap: 8px;
        }

        .top-title {
            font-size: 1.5rem; font-weight: 600; letter-spacing: 0.03em;
            color: var(--accent); text-shadow: 0 0 8px #29f0ff55;
        }

        .top-btn {
            background: #101820; border-radius: 7px;
            border: 1px solid #18303c; padding: 4px 8px;
            font-size: 12px; color: var(--text); cursor: pointer;
            text-transform: uppercase; font-weight: 600;
        }
        .top-btn:hover { border-color: var(--accent); background: #11262c; }
        .top-btn.lit  { opacity: 1.0; font-weight: 700; box-shadow: 0 0 8px currentColor; }
        .top-btn.dim  { opacity: 0.25; }
        .top-btn.danger  { color: var(--bad);  border-color: #a8444e; }
        .top-btn.success { color: var(--ok);   border-color: #229953; }
        .top-btn.neutral { color: var(--warn); border-color: #bfa23a; }
        .estop-btn {
            background: #3a0a0a; border: 2px solid #cc2222; border-radius: 8px;
            color: #ff3333; font-size: 11px; font-weight: 800; cursor: pointer;
            padding: 3px 10px; letter-spacing: 0.06em; text-transform: uppercase;
            box-shadow: 0 0 10px #ff222244;
        }
        .estop-btn:hover { background: #cc2222; color: #fff; box-shadow: 0 0 18px #ff2222aa; }

        .top-status { font-size: 12px; font-family: ui-monospace; }
        .top-status span strong.ON  { color: var(--ok);  }
        .top-status span strong.OFF { color: var(--bad); }

        .main-layout {
            flex: 1; padding: 10px;
            display: flex; gap: var(--grid-gap);
            box-sizing: border-box;
        }

        .panel {
            background: var(--panel); border-radius: 9px;
            padding: 11px; box-shadow: 0 2px 12px #0008;
            display: flex; flex-direction: column; min-width: 0;
        }

        .col-left, .col-center, .col-right {
            flex: 1; min-width: 0;
            display: flex; flex-direction: column; gap: var(--grid-gap);
        }

        .panel-title { font-size: 0.9rem; margin-bottom: 6px; color: var(--accent); }

        /* ── About Modal ── */
        #about-overlay {
            display: none; position: fixed; inset: 0; z-index: 9999;
            background: rgba(0,0,0,0.72); backdrop-filter: blur(4px);
            align-items: center; justify-content: center;
        }
        #about-overlay.open { display: flex; }
        #about-modal {
            background: #111922; border: 1px solid #1e3a46;
            border-radius: 14px; box-shadow: 0 8px 40px #000c;
            max-width: 980px; width: 94vw; max-height: 94vh;
            overflow: hidden; padding: 20px 22px 18px;
            position: relative; display: flex; flex-direction: row; gap: 20px;
        }
        #about-modal-close {
            position: absolute; top: 14px; right: 16px;
            background: none; border: 1px solid #2a4a56; border-radius: 6px;
            color: #aaa; font-size: 16px; cursor: pointer; padding: 2px 9px;
            line-height: 1.4;
        }
        #about-modal-close:hover { background: #1e3040; color: #fff; border-color: var(--accent); }
        .pres-btn {
            font-size:11px; padding:3px 10px; background:#1a3a4a;
            border:1px solid #29f0ff; color:#29f0ff; border-radius:4px;
            cursor:pointer; white-space:nowrap; flex-shrink:0;
            transition: box-shadow 0.2s, background 0.2s;
        }
        .pres-btn:hover {
            background:#1e4a5a;
            box-shadow: 0 0 10px rgba(41,240,255,0.6), 0 0 20px rgba(41,240,255,0.3);
        }
        /* Slideshow */
        #slideshow-overlay {
            display: none; position: fixed; inset: 0;
            background: rgba(0,0,0,0.92); z-index: 10000;
            align-items: center; justify-content: center;
        }
        #slideshow-overlay.open { display: flex; }
        #slideshow-modal {
            position: relative; width: 92vw; max-width: 1200px;
            height: 90vh; background: #111;
            border: 1px solid #1e3a4a; border-radius: 10px;
            display: flex; flex-direction: column; align-items: center;
            overflow: hidden;
        }
        #slideshow-modal:fullscreen,
        #slideshow-modal:-webkit-full-screen {
            width: 100vw; max-width: 100vw;
            height: 100vh; border-radius: 0; border: none;
        }
        #slideshow-close {
            position: absolute; top: 10px; right: 12px; z-index: 20;
            background: #0d2530; border: 1px solid #2a4a5a;
            color: #aaa; font-size: 16px; cursor: pointer;
            padding: 2px 9px; border-radius: 4px; line-height: 1.4;
        }
        #slideshow-close:hover { background: #1e3040; color: #fff; border-color: var(--accent); }
        #pdf-canvas-wrap {
            flex: 1; width: 100%; display: flex; align-items: center;
            justify-content: center; overflow: hidden; padding: 10px 60px;
            box-sizing: border-box;
        }
        #pdf-canvas { display: block; max-width: 100%; max-height: 100%; border-radius: 4px; }
        #pdf-controls {
            display: flex; align-items: center; gap: 14px;
            padding: 8px 0 10px; font-family: ui-monospace; font-size: 12px; color: #5a8a9a;
        }
        .pdf-nav-btn {
            background: #0d2530; border: 1px solid #2a4a5a; color: #29f0ff;
            font-size: 20px; cursor: pointer; padding: 4px 14px;
            border-radius: 6px; line-height: 1.3; user-select: none;
            transition: background 0.15s;
        }
        .pdf-nav-btn:hover { background: #1e3a4a; }
        .pdf-nav-btn:disabled { color: #2a4a5a; cursor: default; background: #0d2530; }
        .pdf-arrow {
            position: absolute; top: 50%; transform: translateY(-50%);
            background: rgba(13,37,48,0.85); border: 1px solid #2a4a5a;
            color: #29f0ff; font-size: 26px; cursor: pointer;
            padding: 10px 13px; border-radius: 6px; z-index: 10;
            user-select: none; transition: background 0.15s; line-height: 1;
        }
        .pdf-arrow:hover { background: #1e3a4a; }
        .pdf-arrow:disabled { color: #2a4a5a; cursor: default; }
        #pdf-arrow-prev { left: 10px; }
        #pdf-arrow-next { right: 10px; }
        .about-left-col {
            display: flex; flex-direction: column; align-items: center;
            gap: 12px; flex-shrink: 0; width: 200px;
        }
        .about-robot-img {
            width: 190px; height: 320px;
            object-fit: contain; border-radius: 10px;
            background: #0b0f14; border: 1px solid #1e3a46; padding: 8px;
        }
        .about-right-col {
            flex: 1; display: flex; flex-direction: column; gap: 12px;
            overflow: hidden; min-width: 0;
        }
        .about-intro h2 { margin: 0 0 4px; font-size: 1.1rem; color: var(--accent); }
        .about-intro p  { margin: 0 0 6px; font-size: 14px; color: #b0ccd4; line-height: 1.5; }
        .about-intro .badge {
            display: inline-block; background: #1a2e38; border: 1px solid #2a4a56;
            border-radius: 5px; padding: 2px 8px; font-size: 11px;
            color: var(--accent); margin-right: 6px; margin-bottom: 4px;
        }
        .subsystem-grid {
            display: grid; grid-template-columns: repeat(auto-fill, minmax(200px, 1fr)); gap: 9px;
        }
        .subsystem-card {
            background: #0d1820; border: 1px solid #1e3a46; border-radius: 9px;
            padding: 9px 12px; display: flex; flex-direction: column; gap: 4px;
        }
        .subsystem-card h3 {
            margin: 0; font-size: 11px; text-transform: uppercase;
            letter-spacing: 0.06em; color: var(--accent);
        }
        .subsystem-card p { margin: 0; font-size: 11px; color: #8ab0bc; line-height: 1.5; }
        .about-divider { border: none; border-top: 1px solid #1e3a46; margin: 0; }
        .about-section-title { font-size: 11px; text-transform: uppercase;
            letter-spacing: 0.08em; color: #4a7a8a; margin: 0; }

        .joystick-wrapper { display: flex; flex-direction: column; align-items: center; gap: 6px; }

        .joystick-area {
            position: relative; width: 120px; height: 120px;
            border-radius: 50%; margin-top: 4px;
            background: radial-gradient(circle at 30% 30%, #1e2b36, #05090e);
            border: 2px solid #18303c; box-shadow: 0 0 10px #000a;
            touch-action: none;
        }

        .joystick-base {
            position: absolute; left: 50%; top: 50%;
            width: 94px; height: 94px;
            margin-left: -47px; margin-top: -47px;
            border-radius: 50%; border: 1px dashed #2a4f60;
        }

        .joystick-knob {
            position: absolute; left: 50%; top: 50%;
            width: 38px; height: 38px;
            margin-left: -19px; margin-top: -19px;
            border-radius: 50%;
            background: radial-gradient(circle at 30% 30%, #3cf0ff, #0e4f5b);
            box-shadow: 0 0 14px #29f0ffaa;
        }

        .joy-readouts { display: flex; gap: 8px; font-family: ui-monospace; font-size: 12px; }
        .joy-readouts span strong { color: var(--ok); }

        .console {
            flex: 1 1 auto; background: #061016;
            border: 1px solid #17303b; border-radius: 7px;
            padding: 8px 7px; overflow: auto;
            font-family: ui-monospace; font-size: 13px; color: #b2f3ff;
        }
        .console .line { margin-bottom: 2px; }

        .tag {
            display: inline-block; padding: 0 7px; border-radius: 3px;
            margin-right: 5px; border: 1px solid #234a54;
            color: #baf9ff; background: #141d20; font-size: 12px;
        }

        .kv {
            border: 1px dashed #18303c; border-radius: 7px;
            padding: 2px 7px; font-family: ui-monospace; font-size: 12px;
            color: #aee9ef; margin-bottom: 2px; min-width: 0;
            line-height: 1.3; display: flex; justify-content: space-between;
        }
        .kv strong { color: var(--ok); font-weight: 600; font-size: 13px; }

        .sensor-section-title { margin-top: 4px; margin-bottom: 2px; text-decoration: underline; }
        .sensor-avg { font-size: 11px; color: #c2f2ff; margin-left: 6px; }
        .sensor-status-line { margin-top: 4px; font-size: 12px; }

        @media (max-width: 960px) { .main-layout { flex-direction: column; } }

        .led {
            display: inline-block; width: 9px; height: 9px;
            border-radius: 50%; margin-right: 4px; vertical-align: middle;
            background: var(--bad); box-shadow: 0 0 4px #ff555088;
        }
        .led.on   { background: var(--ok);  box-shadow: 0 0 6px #8dff8aaa; }
        .led.warn { background: #ffdd57;   box-shadow: 0 0 6px #ffdd57aa; }

        /* Calibration status bar */
        .calib-bar {
            font-size: 12px; font-family: system-ui, sans-serif;
            color: #fff;
        }
        .calib-bar.waiting { color: #fff; }
        .calib-bar.live    { color: var(--ok); }

        .obs-table { width: 100%; border-collapse: collapse; font-size: 12px; }
        .obs-table td { padding: 3px 6px; border-bottom: 1px solid #18303c; font-family: system-ui, sans-serif; }
        .obs-table td:first-child { color: #29f0ff; }
        .obs-table td:last-child  { color: #29f0ff; text-align: right; font-weight: 600; font-family: ui-monospace; }
        .obs-age { font-size: 11px; color: #aaa; margin-top: 4px; }
        .obs-age.stale { color: var(--bad); }

        .wasd-wrapper { display: flex; flex-direction: column; align-items: center; gap: 4px; margin-top: 4px; }
        .wasd-row { display: flex; gap: 6px; }
        .wasd-btn {
            width: 34px; height: 30px; background: #0d1a22;
            border: 1px solid #18303c; border-radius: 5px;
            font-size: 18px; cursor: pointer; color: var(--text);
            transition: background 0.08s, border-color 0.08s; user-select: none;
        }
        .wasd-btn:active, .wasd-btn.pressed {
            background: #0e3040; border-color: var(--accent);
            color: var(--accent); box-shadow: 0 0 8px #29f0ff55;
        }
        .speed-row {
            display: flex; align-items: center; gap: 8px;
            font-family: ui-monospace; font-size: 12px; margin-top: 6px;
        }
        .speed-row input[type=range] { flex: 1; accent-color: var(--accent); }
        input[type=number]::-webkit-inner-spin-button,
        input[type=number]::-webkit-outer-spin-button { -webkit-appearance: none; margin: 0; }
        input[type=number] { -moz-appearance: textfield; }
        .motor-status-row { display: flex; align-items: center; gap: 8px; font-size: 12px; margin-top: 4px; }
    </style>
</head>
<body>
<div class="top-bar">
    <div class="top-row">
        <div class="top-row-left">
            <div class="top-title">Inverted Pendulum Robot Control Hub</div>
            <button class="top-btn neutral" onclick="document.getElementById('about-overlay').classList.add('open')" style="margin-left:10px;padding:3px 10px;font-size:11px;">ℹ About</button>
            <button id="btn-test-mode" onclick="toggleTestMode()" style="margin-left:6px;padding:3px 10px;font-size:11px;background:#7a3a00;border:1px solid #ff8c00;color:#ff8c00;border-radius:4px;cursor:pointer;font-weight:600;">⚙ Test</button>
        </div>
        <div class="top-row-right" style="display:flex;align-items:center;gap:6px;flex-wrap:nowrap;">
            <button class="estop-btn" onclick="emergencyStop()">⛔ E-STOP</button>
            <button id="btn-tr-sensors-on"  class="top-btn success" onclick="startSensorSuite()">Sensors ON</button>
            <button id="btn-tr-sensors-off" class="top-btn danger"  onclick="stopSensorSuite()">Sensors OFF</button>
            <button id="btn-tr-motors-on"   class="top-btn success" onclick="startPigpiod()"    style="margin-left:6px;">Motors ON</button>
            <button id="btn-tr-motors-off"  class="top-btn danger"  onclick="stopMotorTest()">Motors OFF</button>
            <button id="btn-tr-autonav-on"  class="top-btn success" onclick="startAutonav()"    style="margin-left:6px;">Auto Nav ON</button>
            <button id="btn-tr-autonav-off" class="top-btn danger"  onclick="stopAutonav()">Auto Nav OFF</button>
        </div>
    </div>
    <!-- Pipeline controls row -->
    <div class="bottom-row" style="border-top:1px solid #18303c; padding-top:5px; margin-top:2px;">
        <div class="bottom-row-left" style="gap:12px; flex-wrap:wrap;">
            <span class="top-status">
                <span id="led-sensors" class="led"></span>
                <span style="font-size:14px;color:#fff;">IMU & Encoders</span>&nbsp;
                <button id="btn-sensors-on"  class="top-btn success" onclick="startSensors()" style="padding:2px 7px;">ON</button>
                <button id="btn-sensors-off" class="top-btn danger"  onclick="stopSensors()"  style="padding:2px 7px;">OFF</button>
            </span>
            <span class="top-status">
                <span id="led-ultrasonic" class="led"></span>
                <span style="font-size:14px;color:#fff;">Ultrasonic</span>&nbsp;
                <button id="btn-ultrasonic-on"  class="top-btn success" onclick="startUltrasonic()" style="padding:2px 7px;">ON</button>
                <button id="btn-ultrasonic-off" class="top-btn danger"  onclick="stopUltrasonic()"  style="padding:2px 7px;">OFF</button>
            </span>
            <span class="top-status">
                <span id="led-pid" class="led"></span>
                <span style="font-size:14px;color:#fff;">PID</span>&nbsp;
                <button id="btn-pid-on"  class="top-btn success" onclick="startPID()" style="padding:2px 7px;">ON</button>
                <button id="btn-pid-off" class="top-btn danger"  onclick="stopPID()"  style="padding:2px 7px;">OFF</button>
            </span>
            <span class="top-status">
                <span id="led-motorwasd" class="led"></span>
                <span style="font-size:14px;color:#fff;">Manual Motors</span>&nbsp;
                <button id="btn-motorwasd-on"  class="top-btn success" onclick="motorOn()"  style="padding:2px 7px;">ON</button>
                <button id="btn-motorwasd-off" class="top-btn danger"  onclick="motorOff()" style="padding:2px 7px;">OFF</button>
            </span>
        </div>
    </div>
</div>

<!-- ══ About / Project Info Modal ══ -->
<div id="about-overlay" onclick="if(event.target===this)this.classList.remove('open')">
    <div id="about-modal">
        <button id="about-modal-close" onclick="document.getElementById('about-overlay').classList.remove('open')">✕</button>

        <!-- LEFT: big robot image + badges -->
        <div class="about-left-col">
            <img src="/robot.png" alt="Robot" class="about-robot-img">
            <div style="text-align:center;">
                <span class="badge" style="font-size:18px;color:#29f0ff;font-weight:700;">ECEN 404 / Team 55</span><br style="margin-bottom:4px;">
                <span class="badge" style="margin-top:4px;font-size:18px;color:#29f0ff;font-weight:700;">Texas A&M University</span>
            </div>
            <div style="font-size:16px;color:#d7f8ff;line-height:2.0;text-align:center;">
                <span style="font-weight:600;color:#29f0ff;">Team Lead:</span> Sean Wall<br>
                Stephen Darwin<br>
                Dylan Leroy<br>
                Arian Golkarfard<br>
                <div style="height:0.5em;"></div>
                <span style="font-weight:600;color:#29f0ff;">TA:</span> Sabyasachi Gupta<br>
                <span style="font-weight:600;color:#29f0ff;">Sponsor:</span> Aniruddha Datta
            </div>
        </div>

        <!-- RIGHT: title, description, subsystem cards -->
        <div class="about-right-col">
            <div class="about-intro">
                <div style="display:flex;align-items:center;gap:10px;flex-wrap:wrap;">
                    <h2 style="margin:0;">Inverted Pendulum Self-Balancing Robot 3</h2>
                    <button onclick="openSlideshow()" class="pres-btn">Final Presentation</button>
                    <a href="https://github.com/ECEN404-2026SP-Inverted-Pendulum-Team-3" target="_blank" class="pres-btn" style="text-decoration:none;">GitHub</a>
                </div>
                <p>A two-wheeled self-balancing robot designed and built for the ECEN 404 Senior Capstone at Texas A&M University. The robot maintains upright balance using real-time PID control driven by IMU feedback, with manual and autonomous navigation modes accessible through this web dashboard.</p>
            </div>

            <hr class="about-divider">
            <p class="about-section-title">Subsystems</p>

            <!-- Subsystem cards -->
            <div class="subsystem-grid">
                <div class="subsystem-card">
                    <h3>Chassis</h3>
                    <p>Custom 3D-printed chassis with two-wheeled differential drive. The center-of-mass is positioned above the wheel axis to create the natural instability the control loop corrects.</p>
                </div>
                <div class="subsystem-card">
                    <h3>Connector PCB</h3>
                    <p>Custom-designed PCB that centralizes all wiring between the Raspberry Pi 4, motor controller, power subsystem, and sensors. Includes logic level converters to bridge the Pi's 3.3V GPIO with 5V signals. JST-XH connectors throughout allow quick removal of individual components.</p>
                </div>
                <div class="subsystem-card">
                    <h3>Sensors</h3>
                    <p>BNO085 IMU provides pitch, pitch rate, yaw, and yaw rate at 100 Hz via I²C. AMT10 Incremental Encoders on the wheels supply left/right velocity feedback. 2 HC-SR04 ultrasonic sensors (left and right) provide obstacle distance for autonomous navigation. All data shared via POSIX shared memory for minimal latency.</p>
                </div>
                <div class="subsystem-card">
                    <h3>Balance Control (PID)</h3>
                    <p>Pitch PD loop corrects tilt at 100 Hz. Heading hold uses yaw integration with proportional correction. Forward drive ramps with acceleration limiting and exponential decay. Tip-over cutoff at +-35 degrees.</p>
                </div>
                <div class="subsystem-card">
                    <h3>Motor Drive</h3>
                    <p>Sabertooth 2x12 motor controller driven by PWM servo signals from a Raspberry Pi 4 via pigpio. Each channel rated 12 A continuous (24 A total). Commands delivered as 1000 to 2000 us pulse widths.</p>
                </div>
                <div class="subsystem-card">
                    <h3>Power System</h3>
                    <p>Onboard 24V rechargeable battery pack with a battery management system powers the entire robot. Two DC-DC buck converters step down the 24V supply to 5V and 3.3V for devices and sensors. The Sabertooth motor controller receives 24V directly for motor drive.</p>
                </div>
                <div class="subsystem-card">
                    <h3>Computing</h3>
                    <p>Raspberry Pi 4 (4 GB) running Raspberry Pi OS. Sensor, PID, and web processes run concurrently. Flask webserver provides real-time telemetry and remote control from any browser on the local network.</p>
                </div>
                <div class="subsystem-card">
                    <h3>Web Interface</h3>
                    <p>This dashboard is built with Flask and Chart.js. Features include live sensor telemetry, PID gain sliders, manual WASD and joystick control, odometry path tracker, stability timer, and an event console for system status.</p>
                </div>
            </div>
        </div>
    </div>
</div>

<!-- Slideshow overlay -->
<div id="slideshow-overlay" onclick="if(event.target===this)closeSlideshow()">
    <div id="slideshow-modal">
        <button id="slideshow-close" onclick="closeSlideshow()">✕</button>
        <button id="slideshow-fs" onclick="toggleFullscreen()" style="position:absolute;bottom:10px;right:12px;z-index:20;background:#0d2530;border:1px solid #2a4a5a;color:#aaa;font-size:14px;cursor:pointer;padding:2px 8px;border-radius:4px;line-height:1.4;" title="Fullscreen">&#x26F6;</button>
        <button class="pdf-arrow" id="pdf-arrow-prev" onclick="pdfPrev()" disabled>&#8249;</button>
        <button class="pdf-arrow" id="pdf-arrow-next" onclick="pdfNext()" disabled>&#8250;</button>
        <div id="pdf-canvas-wrap">
            <canvas id="pdf-canvas"></canvas>
        </div>
        <div id="pdf-controls">
            <button class="pdf-nav-btn" onclick="pdfPrev()" id="pdf-btn-prev" disabled>&#8249;</button>
            <span id="pdf-page-label">— / —</span>
            <button class="pdf-nav-btn" onclick="pdfNext()" id="pdf-btn-next" disabled>&#8250;</button>
        </div>
        <!-- Video buttons shown only on slide 8 -->
        <div id="pdf-video-btns" style="display:none;position:absolute;bottom:52px;left:50%;transform:translateX(-50%);display:flex;gap:10px;z-index:15;">
            <button onclick="openVideo('/vid1mp4.mp4')" style="background:#1a3a4a;border:1px solid #29f0ff;color:#29f0ff;padding:5px 14px;border-radius:5px;cursor:pointer;font-size:12px;font-family:ui-monospace;">&#9654; Video 1</button>
            <button onclick="openVideo('/vid2mp4.mp4')" style="background:#1a3a4a;border:1px solid #29f0ff;color:#29f0ff;padding:5px 14px;border-radius:5px;cursor:pointer;font-size:12px;font-family:ui-monospace;">&#9654; Video 2</button>
        </div>
        <!-- Video player overlay -->
        <div id="video-overlay" style="display:none;position:absolute;inset:0;background:rgba(0,0,0,0.88);z-index:30;align-items:center;justify-content:center;flex-direction:column;gap:12px;">
            <video id="video-player" controls style="max-width:90%;max-height:80vh;border-radius:6px;background:#000;"></video>
            <button onclick="closeVideo()" style="background:#1a3a4a;border:1px solid #2a4a5a;color:#aaa;padding:4px 16px;border-radius:4px;cursor:pointer;font-size:13px;">&#10005; Close</button>
        </div>
    </div>
</div>

<div class="main-layout">
    <div class="col-left">
        <!-- Combined Manual Motor Control -->
        <div class="panel" style="min-height:270px;">
            <div class="panel-title">Manual Motor Control</div>
            <!-- Status row -->
            <div class="motor-status-row" style="margin-bottom:8px;">
                <span id="led-motorwasd-panel" class="led"></span>
                <span id="wasd-status-txt" style="font-size:12px;color:#aaa;">Motors OFF — press ON above to start</span>
            </div>
            <!-- Three-column layout: WASD | Joystick | Log -->
            <div style="display:flex;align-items:flex-start;gap:10px;">
                <!-- LEFT: Arrow buttons -->
                <div style="flex-shrink:0;display:flex;flex-direction:column;align-items:center;justify-content:center;gap:4px;padding-top:44px;">
                    <div class="wasd-row">
                        <button class="wasd-btn" id="btn-fwd"
                            onclick="wasdToggle('fwd')"
                            ontouchstart="e=>{e.preventDefault();wasdToggle('fwd')}">&#x2191;</button>
                    </div>
                    <div class="wasd-row">
                        <button class="wasd-btn" id="btn-left"
                            onclick="wasdToggle('left')"
                            ontouchstart="e=>{e.preventDefault();wasdToggle('left')}">&#x2190;</button>
                        <button class="wasd-btn" id="btn-rev"
                            onclick="wasdToggle('rev')"
                            ontouchstart="e=>{e.preventDefault();wasdToggle('rev')}">&#x2193;</button>
                        <button class="wasd-btn" id="btn-right"
                            onclick="wasdToggle('right')"
                            ontouchstart="e=>{e.preventDefault();wasdToggle('right')}">&#x2192;</button>
                    </div>
                    <button onclick="wasdStop()" style="margin-top:6px;padding:4px 18px;background:#3a1a1a;border:1px solid #7a2a2a;color:#ff7070;border-radius:4px;font-size:13px;cursor:pointer;font-family:ui-monospace;letter-spacing:0.05em;">&#x25A0; STOP</button>
                    <div style="font-family:ui-monospace;font-size:10px;color:#fff;margin-top:4px;text-align:center;">Arrow keys</div>
                </div>
                <!-- MIDDLE: Joystick -->
                <div class="joystick-wrapper" style="flex:1;display:flex;flex-direction:column;align-items:center;padding-top:16px;">
                    <div id="joystick-area" class="joystick-area">
                        <div id="joystick-base" class="joystick-base"></div>
                        <div id="joystick-knob" class="joystick-knob"></div>
                    </div>
                    <div class="joy-readouts">
                        <span>X: <strong id="joy-x">0.00</strong></span>
                        <span>Y: <strong id="joy-y">0.00</strong></span>
                    </div>
                </div>
                <!-- RIGHT: Motor Log -->
                <div style="width:38%;flex-shrink:0;display:flex;flex-direction:column;gap:4px;">
                    <div style="font-size:11px;color:#fff;">Motor Log</div>
                    <div id="joy-log" style="height:150px;overflow-y:auto;background:#0d1f28;border-radius:4px;padding:5px 7px;font-family:ui-monospace;font-size:12.5px;color:#a0c8d8;"></div>
                </div>
            </div>
            <!-- Speed slider -->
            <div class="speed-row" style="margin-top:10px;">
                <span>Speed</span>
                <input type="range" id="speed-slider" min="0" max="100" value="50"
                    oninput="document.getElementById('speed-val').value=parseInt(this.value)">
                <input type="number" id="speed-val" min="0" max="100" step="1" value="50"
                    style="width:32px;text-align:center;font-size:12px;font-family:ui-monospace;background:#0d2530;border:1px solid #2a4a5a;color:#e0f0f0;border-radius:3px;padding:1px 3px;"
                    oninput="syncSlider('speed-slider',this,0,100)">
                <span style="font-size:12px;color:#e0f0f0;">%</span>
            </div>
        </div>

        <!-- Motor Command Indicators -->
        <div class="panel">
            <div class="panel-title" style="display:flex;justify-content:space-between;align-items:baseline;">
                <span>Motor Command Indicators</span>
                <button onclick="resetMotorIndicators()" style="font-size:10px;padding:2px 8px;background:#1a3a4a;border:1px solid #2a4a5a;color:#5a8a9a;border-radius:3px;cursor:pointer;">Reset</button>
            </div>
            <div style="display:flex;flex-direction:column;gap:10px;margin-top:6px;">
                <div>
                    <div style="display:flex;justify-content:space-between;font-size:11px;color:#fff;margin-bottom:3px;">
                        <span>Left Motor</span><span id="motor-left-val">0.00</span>
                    </div>
                    <div style="position:relative;height:18px;background:#18303c;border-radius:4px;overflow:hidden;">
                        <div style="position:absolute;left:50%;top:0;width:1px;height:100%;background:#2a4a5a;"></div>
                        <div id="motor-left-bar" style="position:absolute;top:2px;height:14px;border-radius:3px;background:#29f0ff;transition:width 0.1s,left 0.1s;width:0%;left:50%;"></div>
                    </div>
                </div>
                <div>
                    <div style="display:flex;justify-content:space-between;font-size:11px;color:#fff;margin-bottom:3px;">
                        <span>Right Motor</span><span id="motor-right-val">0.00</span>
                    </div>
                    <div style="position:relative;height:18px;background:#18303c;border-radius:4px;overflow:hidden;">
                        <div style="position:absolute;left:50%;top:0;width:1px;height:100%;background:#2a4a5a;"></div>
                        <div id="motor-right-bar" style="position:absolute;top:2px;height:14px;border-radius:3px;background:#29f0ff;transition:width 0.1s,left 0.1s;width:0%;left:50%;"></div>
                    </div>
                </div>
                <div style="display:flex;justify-content:space-between;font-size:10px;color:#fff;margin-top:-4px;">
                    <span>-1.0</span><span>0</span><span>+1.0</span>
                </div>
                <div style="border-top:1px solid #18303c;padding-top:8px;margin-top:2px;">
                    <div style="display:flex;justify-content:space-between;align-items:center;margin-bottom:4px;">
                        <span style="font-size:11px;color:#fff;">Motor Stress</span>
                        <span id="motor-stress-label" style="font-size:11px;font-family:ui-monospace;color:#fff;">IDLE</span>
                    </div>
                    <div style="position:relative;height:10px;background:#18303c;border-radius:4px;overflow:hidden;">
                        <div id="motor-stress-bar" style="height:100%;border-radius:4px;background:#29f0ff;width:0%;transition:width 0.5s,background 0.5s;"></div>
                    </div>
                    <div style="display:flex;justify-content:space-between;font-size:10px;color:#fff;margin-top:2px;">
                        <span>Cool</span><span id="motor-stress-time" style="font-family:ui-monospace;">0s high</span><span>Hot</span>
                    </div>
                </div>
                <!-- Est. Current Draw -->
                <div style="border-top:1px solid #18303c;padding-top:8px;margin-top:8px;">
                    <div style="display:flex;justify-content:space-between;align-items:center;margin-bottom:4px;">
                        <span style="font-size:11px;color:#fff;">Estimated Motors Current Draw</span>
                        <span id="current-val" style="font-size:11px;font-family:ui-monospace;color:var(--ok);">0.0 A</span>
                    </div>
                    <div style="position:relative;height:10px;background:#18303c;border-radius:4px;overflow:hidden;">
                        <div id="current-bar" style="height:100%;border-radius:4px;background:#29f0ff;width:0%;transition:width 0.15s,background 0.15s;"></div>
                    </div>
                    <div style="display:flex;justify-content:space-between;font-size:10px;color:#fff;margin-top:2px;">
                        <span>0 A</span><span style="font-family:ui-monospace;color:#fff;font-size:10px;">Sabertooth 2×12</span><span>24 A</span>
                    </div>
                </div>
            </div>
        </div>

        <!-- Live Pitch Chart -->
        <div class="panel">
            <div class="panel-title" style="display:flex;justify-content:space-between;align-items:baseline;">
                <span>Pitch Angle vs Time</span>
                <button onclick="resetPitchChart()" style="font-size:10px;padding:2px 8px;background:#1a3a4a;border:1px solid #2a4a5a;color:#5a8a9a;border-radius:3px;cursor:pointer;">Reset</button>
            </div>
            <canvas id="pitch-chart" height="290"></canvas>
            <div style="font-family:ui-monospace;font-size:11px;color:#fff;margin-top:4px;display:flex;justify-content:space-between;">
                <span>Last 30 s</span>
                <span>Red lines = ±<span id="tip-limit-label">35</span>° tip-over limit</span>
            </div>
        </div>

    </div>

    <div class="col-center">
        <div class="panel console-panel">
            <div class="panel-title" style="display:flex;justify-content:space-between;align-items:baseline;">
                <span>Event Console</span>
                <button onclick="document.getElementById('console').innerHTML=''" style="font-size:10px;padding:2px 8px;background:#1a3a4a;border:1px solid #2a4a5a;color:#5a8a9a;border-radius:3px;cursor:pointer;">Clear</button>
            </div>
            <div class="console" id="console"></div>
        </div>

        <!-- Stability Timer + Best Run Tracker -->
        <div class="panel">
            <div class="panel-title" style="display:flex;justify-content:space-between;align-items:baseline;">
                <span>Stability Timer &amp; Best Run</span>
                <button onclick="clearRunHistory()" style="font-size:10px;padding:2px 8px;background:#1a3a4a;border:1px solid #2a4a5a;color:#5a8a9a;border-radius:3px;cursor:pointer;">Clear</button>
            </div>
            <!-- Current run live display -->
            <div style="display:flex;align-items:center;gap:10px;margin-bottom:8px;">
                <div id="stab-dot" style="width:10px;height:10px;border-radius:50%;background:#2a4a5a;flex-shrink:0;"></div>
                <div>
                    <div style="font-size:11px;color:#fff;">Current Run</div>
                    <div id="stab-current" style="font-family:ui-monospace;font-size:20px;color:#e0f0f0;letter-spacing:0.05em;">--:--</div>
                </div>
                <div style="margin-left:auto;text-align:right;">
                    <div style="font-size:11px;color:#fff;">Best Run</div>
                    <div id="stab-best" style="font-family:ui-monospace;font-size:16px;color:#ffd060;letter-spacing:0.05em;">--:--</div>
                </div>
            </div>
            <!-- Run history table -->
            <div style="font-size:11px;color:#fff;margin-bottom:4px;">Run History</div>
            <div id="stab-history" style="max-height:140px;overflow-y:auto;background:#0d1f28;border-radius:4px;padding:4px 6px;">
                <div style="font-family:ui-monospace;font-size:11px;color:#aaa;text-align:center;padding:8px 0;">No runs yet — start PID to begin</div>
            </div>
        </div>

        <!-- Robot Path Tracker + PID Gain Sliders side by side -->
        <div style="display:flex;gap:10px;align-items:stretch;">
            <div class="panel" style="width:200px;flex-shrink:0;">
                <div class="panel-title" style="display:flex;justify-content:space-between;align-items:baseline;">
                    <span>Robot Path Tracker</span>
                    <button onclick="resetOdometry()" style="font-size:10px;padding:2px 8px;background:#1a3a4a;border:1px solid #2a4a5a;color:#5a8a9a;border-radius:3px;cursor:pointer;">Reset</button>
                </div>
                <canvas id="odom-canvas" width="160" height="160" style="width:100%;aspect-ratio:1/1;border-radius:4px;background:#0d1f28;display:block;margin-top:6px;"></canvas>
                <div style="display:flex;justify-content:space-between;font-size:11px;color:#5a8a9a;margin-top:4px;">
                    <span>X: <span id="odom-x" style="font-family:ui-monospace;color:#e0f0f0;">0.00</span> m</span>
                    <span>Y: <span id="odom-y" style="font-family:ui-monospace;color:#e0f0f0;">0.00</span> m</span>
                    <span>Hdg: <span id="odom-hdg" style="font-family:ui-monospace;color:#e0f0f0;">0.0</span>°</span>
                </div>
            </div>
            <div class="panel" style="flex:1;min-width:0;display:flex;flex-direction:column;">
                <div class="panel-title">PID Gain Sliders</div>
                <div style="display:flex;flex-direction:column;justify-content:space-between;flex:1;gap:6px;margin-top:6px;">
                    <div style="display:flex;align-items:center;gap:8px;">
                        <label style="width:70px;font-size:12px;color:#29f0ff;">KP</label>
                        <input type="range" id="slider-kp" min="0" max="150" step="0.5" value="60" style="flex:1;" oninput="document.getElementById('val-kp').value=parseFloat(this.value).toFixed(1)">
                        <input type="number" id="val-kp" min="0" max="150" step="0.5" value="60.0" style="width:52px;text-align:center;font-size:12px;font-family:ui-monospace;background:#0d2530;border:1px solid #2a4a5a;color:#e0f0f0;border-radius:3px;padding:2px 4px;" oninput="syncSlider('slider-kp',this,0,150)">
                    </div>
                    <div style="display:flex;align-items:center;gap:8px;">
                        <label style="width:70px;font-size:12px;color:#29f0ff;">KD</label>
                        <input type="range" id="slider-kd" min="0" max="30" step="0.1" value="12" style="flex:1;" oninput="document.getElementById('val-kd').value=parseFloat(this.value).toFixed(1)">
                        <input type="number" id="val-kd" min="0" max="30" step="0.1" value="12.0" style="width:52px;text-align:center;font-size:12px;font-family:ui-monospace;background:#0d2530;border:1px solid #2a4a5a;color:#e0f0f0;border-radius:3px;padding:2px 4px;" oninput="syncSlider('slider-kd',this,0,30)">
                    </div>
                    <div style="display:flex;align-items:center;gap:8px;">
                        <label style="width:70px;font-size:12px;color:#29f0ff;">KI</label>
                        <input type="range" id="slider-ki" min="0" max="5" step="0.05" value="0" style="flex:1;" oninput="document.getElementById('val-ki').value=parseFloat(this.value).toFixed(2)">
                        <input type="number" id="val-ki" min="0" max="5" step="0.05" value="0.00" style="width:52px;text-align:center;font-size:12px;font-family:ui-monospace;background:#0d2530;border:1px solid #2a4a5a;color:#e0f0f0;border-radius:3px;padding:2px 4px;" oninput="syncSlider('slider-ki',this,0,5)">
                    </div>
                    <div style="display:flex;align-items:center;gap:8px;">
                        <label style="width:70px;font-size:12px;color:#29f0ff;">Trim (°)</label>
                        <input type="range" id="slider-trim" min="-10" max="10" step="0.1" value="0" style="flex:1;" oninput="document.getElementById('val-trim').value=parseFloat(this.value).toFixed(1)">
                        <input type="number" id="val-trim" min="-10" max="10" step="0.1" value="0.0" style="width:52px;text-align:center;font-size:12px;font-family:ui-monospace;background:#0d2530;border:1px solid #2a4a5a;color:#e0f0f0;border-radius:3px;padding:2px 4px;" oninput="syncSlider('slider-trim',this,-10,10)">
                    </div>
                    <div style="display:flex;align-items:center;gap:8px;">
                        <label style="width:70px;font-size:12px;color:#29f0ff;">Tip-over (°)</label>
                        <input type="range" id="slider-tip" min="5" max="50" step="1" value="35" style="flex:1;" oninput="document.getElementById('val-tip').value=parseInt(this.value);updateTipLimit(parseInt(this.value));">
                        <input type="number" id="val-tip" min="5" max="50" step="1" value="35" style="width:52px;text-align:center;font-size:12px;font-family:ui-monospace;background:#0d2530;border:1px solid #2a4a5a;color:#e0f0f0;border-radius:3px;padding:2px 4px;" oninput="syncSlider('slider-tip',this,5,50);updateTipLimit(parseInt(this.value));">
                    </div>
                    <button onclick="applyPIDGains()" style="margin-top:4px;padding:6px 0;background:#1a3a4a;border:1px solid #29f0ff;color:#29f0ff;border-radius:4px;cursor:pointer;font-size:12px;width:100%;">Apply Gains</button>
                    <div id="pid-gains-status" style="font-size:11px;color:#5a8a9a;text-align:center;min-height:14px;"></div>
                </div>
            </div>
        </div>

        <!-- Live Pitch Gauge -->
        <div class="panel" style="display:flex;flex-direction:column;align-items:center;">
            <div class="panel-title" style="width:100%;">Live Pitch Gauge</div>
            <canvas id="pitch-gauge" height="180" style="width:100%;display:block;margin-top:4px;margin-bottom:auto;"></canvas>
        </div>
    </div>

    <div class="col-right">
        <!-- Live obs_cache sensor panel -->
        <div class="panel">
            <div class="panel-title" style="display:flex;justify-content:space-between;align-items:baseline;">
                <span>Live Sensor Data</span>
                <span id="imu-time" style="font-size:11px;color:#5a8a9a;font-weight:400;"></span>
            </div>

            <!-- Sensor status bar -->
            <div style="display:flex;align-items:center;gap:6px;margin-bottom:8px;">
                <span id="led-sensors-panel" class="led"></span>
                <span id="calib-bar" style="font-size:12px;color:#aaa;">Sensors not running — press ON</span>
            </div>

            <table class="obs-table">
                <tr><td colspan="2" style="padding-top:6px;color:#fff;font-size:11px;text-align:left;border-bottom:none;">IMU &amp; Encoders</td></tr>
                <tr><td>Linear Velocity</td><td><span id="obs-linvel">--</span> m/s</td></tr>
                <tr><td>Pitch</td><td><span id="obs-pitch">--</span> deg</td></tr>
                <tr><td>Pitch Rate</td><td><span id="obs-pitchrate">--</span> rad/s</td></tr>
                <tr><td>Yaw Rate</td><td><span id="obs-yawrate">--</span> rad/s</td></tr>
                <tr><td>Wheel Left</td><td><span id="obs-whl">--</span> rad/s</td></tr>
                <tr><td>Wheel Right</td><td><span id="obs-whr">--</span> rad/s</td></tr>
                <tr><td>Yaw</td><td><span id="obs-yaw">--</span> deg</td></tr>
                <tr><td colspan="2" style="padding-top:6px;color:#fff;font-size:11px;text-align:left;border-bottom:none;">Ultrasonic Sensors</td></tr>
                <tr><td>Right</td><td><span id="obs-us-r">--</span> cm</td></tr>
                <tr><td>Left</td><td><span id="obs-us-l">--</span> cm</td></tr>
                <tr><td colspan="2" style="padding-top:6px;color:#fff;font-size:11px;text-align:left;border-bottom:none;">PID Loop Timing</td></tr>
                <tr><td>Loop Hz</td><td><span id="pid-hz">--</span></td></tr>
                <tr><td>Average dt</td><td><span id="pid-avg-ms">--</span> ms</td></tr>
                <tr><td>Min / Max dt</td><td><span id="pid-minmax-ms">-- / --</span> ms</td></tr>
                <tr><td colspan="2" style="padding-top:6px;color:#fff;font-size:11px;text-align:left;border-bottom:none;">Sensor Pipeline</td></tr>
                <tr><td>SHM Age</td><td><span id="sp-cache-age">0</span> ms</td></tr>
                <tr><td>Data Source</td><td><span id="sp-source">0</span></td></tr>
                <tr><td>Main Loop</td><td><span id="sp-main-hz">0</span> Hz</td></tr>
                <tr><td>IMU Thread</td><td><span id="sp-imu-hz">0</span> Hz</td></tr>
                <tr><td>Encoder Poll</td><td><span id="sp-enc-hz">0</span> Hz</td></tr>
                <tr><td>Ultrasonic</td><td><span id="us-loop-hz">0</span> Hz</td></tr>
                <tr><td>Pass Rate</td><td><span id="sp-pass-rate">0%</span></td></tr>
                <tr><td>Reads</td><td><span id="sp-reads">0</span></td></tr>
            </table>
            <div class="obs-age" id="obs-age-line">waiting for data...</div>
        </div>

        <!-- Encoder RPM vs Time -->
        <div class="panel" style="margin-top:10px;">
            <div class="panel-title" style="display:flex;justify-content:space-between;align-items:baseline;">
                <span>Encoder RPM vs Time</span>
                <button onclick="resetRpmChart()" style="font-size:10px;padding:2px 8px;background:#1a3a4a;border:1px solid #2a4a5a;color:#5a8a9a;border-radius:3px;cursor:pointer;">Reset</button>
            </div>
            <canvas id="rpm-chart" height="245"></canvas>
            <div style="font-family:ui-monospace;font-size:11px;color:#fff;margin-top:4px;display:flex;justify-content:space-between;">
                <span>Last 30 s</span>
                <span style="display:flex;gap:12px;">
                    <span><span style="color:#29f0ff;">&#9135;</span> Left</span>
                    <span><span style="color:#ffd060;">&#9135;</span> Right</span>
                </span>
            </div>
        </div>
    </div>
</div>

<script>

    // =====================================================================
    // LIVE SENSOR DATA POLLING  (obs_cache.bin)
    // =====================================================================
    let _sensorsWereRunning = false;
    let _calibStartTime = null;          // monotonic timestamp when calibration started
    const _CALIB_DURATION_S = 10;        // countdown length (matches sensor flush + calibrate)

    function updateLiveSensorData() {
        fetch('/sensor_data?t=' + Date.now(), { cache: 'no-store' }).then(r => r.json()).then(d => {
            function fmt(v, dec) {
                return (v === null || v === undefined) ? (0).toFixed(dec) : parseFloat(v).toFixed(dec);
            }
            const sensOff      = !d.sensors_running;
            const sensNotReady = !d.sensors_running || !d.calibrated;  // off OR still calibrating
            const usOff        = !d.ultrasonic_running;

            // Feed pitch + RPM charts — skip stale or pre-calibration samples
            const cacheStale = d.cache_age_ms === null || d.cache_age_ms === undefined || d.cache_age_ms > 200;
            if (!sensOff && d.calibrated && !cacheStale) {
                if (d.pitch_deg !== null && d.pitch_deg !== undefined)
                    pushPitchSample(parseFloat(d.pitch_deg));
                if (d.wheel_left_rads !== null && d.wheel_right_rads !== null &&
                    d.wheel_left_rads !== undefined && d.wheel_right_rads !== undefined)
                    pushRpmSample(parseFloat(d.wheel_left_rads), parseFloat(d.wheel_right_rads));
            }
            document.getElementById('obs-linvel').textContent    = sensNotReady ? '0.000' : fmt(d.linear_velocity, 3);
            document.getElementById('obs-pitch').textContent     = sensNotReady ? '0.00'  : fmt(d.pitch_deg, 2);
            drawPitchGauge(sensNotReady ? 0 : (d.pitch_deg || 0));
            document.getElementById('obs-pitchrate').textContent = sensNotReady ? '0.000' : fmt(d.pitch_rate, 3);
            document.getElementById('obs-yawrate').textContent   = sensNotReady ? '0.000' : fmt(d.yaw_rate, 3);
            document.getElementById('obs-whl').textContent       = sensNotReady ? '0.000' : fmt(d.wheel_left_rads, 3);
            document.getElementById('obs-whr').textContent       = sensNotReady ? '0.000' : fmt(d.wheel_right_rads, 3);
            document.getElementById('obs-yaw').textContent       = sensNotReady ? '0.00'  : fmt(d.yaw_deg, 2);
            document.getElementById('obs-us-r').textContent      = usOff   ? '0.0'   : (d.ultrasonic_right_cm !== null ? fmt(d.ultrasonic_right_cm, 1) : '0.0');
            document.getElementById('obs-us-l').textContent      = usOff   ? '0.0'   : (d.ultrasonic_left_cm  !== null ? fmt(d.ultrasonic_left_cm,  1) : '0.0');

            // PID loop timing
            const pt = d.pid_timing;
            document.getElementById('pid-hz').textContent        = pt ? pt.hz + ' Hz'                   : '0 Hz';
            document.getElementById('pid-avg-ms').textContent    = pt ? pt.avg_ms                        : '0';
            document.getElementById('pid-minmax-ms').textContent = pt ? pt.min_ms + ' / ' + pt.max_ms   : '0 / 0';

            // Sensor pipeline stats — all zero when sensors are off or still calibrating
            const cacheAge = (!sensNotReady && d.cache_age_ms !== null && d.cache_age_ms !== undefined) ? d.cache_age_ms : null;
            const cacheAgeEl = document.getElementById('sp-cache-age');
            if (cacheAge !== null) {
                cacheAgeEl.textContent = cacheAge.toFixed(1);
                cacheAgeEl.style.color = cacheAge < 15 ? '#8dff8a' : cacheAge < 35 ? '#ffdd57' : '#ff5555';
            } else {
                cacheAgeEl.textContent = '0'; cacheAgeEl.style.color = '';
            }
            const srcMap = { 'hardware_interface': 'SHM', 'cache_relaxed': 'File', null: '0' };
            document.getElementById('sp-source').textContent = sensNotReady ? '0' : (srcMap[d.obs_source] || d.obs_source || '0');
            const ss = (!sensNotReady) ? d.sensor_stats : null;
            if (ss) {
                document.getElementById('sp-main-hz').textContent  = ss.main_hz;
                document.getElementById('sp-imu-hz').textContent   = ss.imu_hz;
                document.getElementById('sp-enc-hz').textContent   = ss.enc_hz.toLocaleString();
                const prEl = document.getElementById('sp-pass-rate');
                prEl.textContent  = ss.pass_rate.toFixed(1) + '%';
                prEl.style.color  = ss.pass_rate >= 99 ? '#8dff8a' : ss.pass_rate >= 95 ? '#ffdd57' : '#ff5555';
                document.getElementById('sp-reads').textContent = ss.successful.toLocaleString() + ' / ' + ss.total.toLocaleString();
            } else {
                ['sp-main-hz','sp-imu-hz','sp-enc-hz','sp-reads'].forEach(id => {
                    const el = document.getElementById(id); if (el) { el.textContent = '0'; el.style.color = ''; }
                });
                const prFb = document.getElementById('sp-pass-rate');
                if (prFb) { prFb.textContent = '0%'; prFb.style.color = ''; }
            }

            // Ultrasonic loop Hz
            const usHz = document.getElementById('us-loop-hz');
            if (usHz) usHz.textContent = (d.us_stats ? d.us_stats.loop_hz : '0');

            // Stability timer + best run tracker
            updateStabilityTimer(d.pid_running, d.pid_elapsed_s);

            // Update timestamp in Live Sensor Data title
            document.getElementById('imu-time').textContent = new Date().toLocaleTimeString();

            // Motor command indicator bars + stress + current (skip if reset lock is active)
            if (Date.now() > _motorResetUntil) {
                updateMotorBar('motor-left',  d.motor_left);
                updateMotorBar('motor-right', d.motor_right);
                updateMotorStress(d.motor_left, d.motor_right);
                updateCurrentDraw(d.motor_left, d.motor_right);
            }

            // Odometry
            updateOdometry(d.wheel_left_rads, d.wheel_right_rads, d.sensors_running && d.calibrated);

            const ageLine  = document.getElementById('obs-age-line');
            const calibBar = document.getElementById('calib-bar');
            function setCalibBars(color, text) {
                calibBar.style.color = color; calibBar.textContent = text;
            }

            const hasObs = d.sensors_running && d.calibrated && d.linear_velocity !== null && d.linear_velocity !== undefined;
            const ageOk = d.cache_age_ms !== null && d.cache_age_ms !== undefined;

            if (hasObs) {
                if (!_sensorsWereRunning && d.sensors_running) {
                    _sensorsWereRunning = true;
                    _calibStartTime = null;
                    logLine('SENSORS', 'Data flowing — calibration complete');
                }
                const src = d.obs_source === 'cache_relaxed' ? ' (file)' : '';
                setCalibBars('var(--ok)', 'Live — calibrated' + src);
                setLed('led-sensors-panel', true);
                const ageStale = ageOk && d.cache_age_ms > 200;
                ageLine.className = ageStale ? 'obs-age stale' : 'obs-age';
                ageLine.style.color = '';
                let line = '';
                if (ageOk && d.sensors_running && d.cache_age_ms < 1000) {
                    line = 'SHM age: ' + d.cache_age_ms.toFixed(1) + ' ms';
                } else {
                    line = 'Live data';
                }
                if (d.us_age_ms !== null && d.us_age_ms !== undefined) {
                    line += '  |  US age: ' + d.us_age_ms.toFixed(0) + ' ms';
                }
                ageLine.textContent = line;
            } else if (!d.sensors_running) {
                setCalibBars('#aaa', 'Sensors not running — press ON');
                setLed('led-sensors-panel', false);
                ageLine.className = 'obs-age';
                ageLine.style.color = '';
                ageLine.textContent = '';
                _sensorsWereRunning = false;
                _calibStartTime = null;
            } else {
                // Sensors running but not calibrated yet — show countdown
                if (_calibStartTime === null) _calibStartTime = Date.now();
                const elapsed = (Date.now() - _calibStartTime) / 1000;
                const remaining = Math.max(0, Math.ceil(_CALIB_DURATION_S - elapsed));
                setCalibBars('#ffdd57', 'Sensors Calibrating — keep robot still... ' + remaining + 's');
                const el = document.getElementById('led-sensors-panel');
                if (el) { el.className = 'led warn'; }
                ageLine.className = 'obs-age';
                ageLine.style.color = '#ffdd57';
                ageLine.textContent = 'Warmup — waiting for obs_cache / SHM…';
            }

            // Update LEDs
            setLed('led-sensors',         d.sensors_running);
            setLed('led-ultrasonic',      d.ultrasonic_running);
            setLed('led-pid',             d.pid_running);
            setLed('led-motorwasd',       d.motorwasd_running);
            setLed('led-motorwasd-panel', d.motorwasd_running);


            // WASD panel status text
            const wasdTxt = document.getElementById('wasd-status-txt');
            if (d.motorwasd_running) {
                wasdTxt.style.color = '#8dff8a';
                wasdTxt.textContent = 'Motors LIVE — use keys or buttons';
            } else if (d.pid_running) {
                wasdTxt.style.color = '#ffdd57';
                wasdTxt.textContent = 'PID running — stop PID to use manual control';
            } else {
                wasdTxt.style.color = '#aaa';
                wasdTxt.textContent = 'Motors OFF — press ON above to start';
            }
        }).catch(() => {});
    }

    function setLed(id, on) {
        const el = document.getElementById(id);
        if (!el) return;
        el.className = on ? 'led on' : 'led';
    }

    function setButtonPair(onId, offId, isOn) {
        const onBtn  = document.getElementById(onId);
        const offBtn = document.getElementById(offId);
        if (!onBtn || !offBtn) return;
        if (isOn) {
            onBtn.classList.add('lit');     onBtn.classList.remove('dim');
            offBtn.classList.add('dim');    offBtn.classList.remove('lit');
        } else {
            offBtn.classList.add('lit');    offBtn.classList.remove('dim');
            onBtn.classList.add('dim');     onBtn.classList.remove('lit');
        }
    }

    function updateMotorBar(prefix, value) {
        const bar = document.getElementById(prefix + '-bar');
        const val = document.getElementById(prefix + '-val');
        if (!bar || !val) return;
        const v = (value === null || value === undefined) ? 0.0 : parseFloat(value);
        val.textContent = v.toFixed(2);
        const pct = Math.abs(v) * 50; // 50% of half-bar width
        if (v >= 0) {
            bar.style.left  = '50%';
            bar.style.width = pct + '%';
            bar.style.background = v > 0.05 ? '#29f0ff' : '#2a4a5a';
        } else {
            bar.style.left  = (50 - pct) + '%';
            bar.style.width = pct + '%';
            bar.style.background = v < -0.05 ? '#ff7b54' : '#2a4a5a';
        }
    }

    // =====================================================================
    // MOTOR STRESS TRACKER
    // =====================================================================
    const STRESS_HIGH_THRESHOLD = 0.7;
    const STRESS_DECAY_S = 60;   // seconds until fully "cool" after no high output
    let _stressAccum = 0;        // accumulated high-output seconds
    let _lastStressUpdate = null;

    function updateMotorStress(motorLeft, motorRight) {
        const now = Date.now() / 1000;
        const dt = _lastStressUpdate === null ? 0 : Math.min(now - _lastStressUpdate, 1.0);
        _lastStressUpdate = now;

        const ml = Math.abs(motorLeft  || 0);
        const mr = Math.abs(motorRight || 0);
        const maxOut = Math.max(ml, mr);

        if (maxOut >= STRESS_HIGH_THRESHOLD) {
            _stressAccum += dt * maxOut;
        } else {
            _stressAccum = Math.max(0, _stressAccum - dt * 0.5);
        }
        _stressAccum = Math.min(_stressAccum, STRESS_DECAY_S);

        const pct = (_stressAccum / STRESS_DECAY_S) * 100;
        const bar   = document.getElementById('motor-stress-bar');
        const label = document.getElementById('motor-stress-label');
        const time  = document.getElementById('motor-stress-time');
        if (!bar) return;

        bar.style.width = pct.toFixed(1) + '%';
        let stressColor, stressLabel;
        if (pct < 33) {
            stressColor = '#29f0ff'; stressLabel = 'IDLE';
        } else if (pct < 66) {
            stressColor = '#ffdd57'; stressLabel = 'WARM';
        } else {
            stressColor = '#ff5555'; stressLabel = 'HOT';
        }
        bar.style.background = stressColor;
        label.style.color    = stressColor;
        label.textContent    = stressLabel;
        time.textContent = _stressAccum.toFixed(0) + 's high';
    }

    // =====================================================================
    // ODOMETRY / ROBOT PATH TRACKER
    // =====================================================================
    const WHEEL_RADIUS_M  = ((8.0 * 25.4) / 1000.0) / 2.0;   // 8" wheel
    const TRACK_WIDTH_M   = 13.0 * 0.0254;                    // 13" track
    const ODOM_TRAIL_MAX  = 800;

    let odomX = 0, odomY = 0, odomTheta = 0;
    let odomTrail = [];
    let _lastOdomTime = null;

    function resetOdometry() {
        odomX = 0; odomY = 0; odomTheta = 0;
        odomTrail = [];
        _lastOdomTime = null;
        document.getElementById('odom-x').textContent   = '0.00';
        document.getElementById('odom-y').textContent   = '0.00';
        document.getElementById('odom-hdg').textContent = '0.0';
        drawOdometry();
    }

    // Sabertooth 2x12: 12 A continuous per channel = 24 A total max
    const MOTOR_MAX_A = 12.0;

    // Fixed overhead at 22.2V battery level (P = V_comp * I_comp, I_batt = P / (V_batt * eff))
    // Buck converter efficiency ~85%, V_battery = 22.2V
    // Pi 4 @ 2.0A/5V, 2x HC-SR04 @ 15mA/5V, 2x AMT10 @ 6mA/5V, BNO085 @ 10mA/3.3V, PCB ~50mA/5V
    const FIXED_OVERHEAD_A = (
        (2.000 * 5.0) +          // Raspberry Pi 4
        (2 * 0.015 * 5.0) +      // 2x HC-SR04
        (2 * 0.006 * 5.0) +      // 2x AMT10 encoder
        (0.010 * 3.3) +          // BNO085 IMU
        (0.050 * 5.0)            // PCB / misc
    ) / (22.2 * 0.85);           // convert to battery-level amps

    function updateCurrentDraw(motorLeft, motorRight) {
        const bar = document.getElementById('current-bar');
        const val = document.getElementById('current-val');
        if (!bar || !val) return;
        const l = Math.abs(motorLeft  || 0);
        const r = Math.abs(motorRight || 0);
        const estA = (l + r) * MOTOR_MAX_A;           // 0–24 A (motors only)
        const pct  = Math.min((estA / (MOTOR_MAX_A * 2)) * 100, 100);
        const color = estA < 8 ? '#29f0ff' : estA < 16 ? '#ffdd57' : '#ff5555';
        bar.style.width      = pct + '%';
        bar.style.background = color;
        val.style.color      = color;
        val.textContent      = estA.toFixed(1) + ' A';

        // Total draw: motors + fixed overhead — only show when motors are active
        const tBar = document.getElementById('total-current-bar');
        const tVal = document.getElementById('total-current-val');
        if (tBar && tVal) {
            if (estA > 0) {
                const totalA   = estA + FIXED_OVERHEAD_A;
                const totalMax = 25.0;
                const totalPct = Math.min((totalA / totalMax) * 100, 100);
                const totalColor = totalA < 9 ? '#29f0ff' : totalA < 18 ? '#ffdd57' : '#ff5555';
                tBar.style.width      = totalPct + '%';
                tBar.style.background = totalColor;
                tVal.style.color      = totalColor;
                tVal.textContent      = totalA.toFixed(1) + ' A';
            } else {
                tBar.style.width      = '0%';
                tBar.style.background = '#29f0ff';
                tVal.style.color      = '#29f0ff';
                tVal.textContent      = '0.0 A';
            }
        }
    }

    let _motorResetUntil = 0;   // epoch ms — block motor indicator updates until this time

    function resetMotorIndicators() {
        fetch('/reset_motor_state', { method: 'POST' });  // zero the file on disk
        _motorResetUntil = Date.now() + 2000;   // freeze live updates for 2 s
        _stressAccum = 0;

        // Force all bars and values directly to zero
        updateMotorBar('motor-left',  0);
        updateMotorBar('motor-right', 0);
        updateMotorStress(0, 0);
        updateCurrentDraw(0, 0);

        // Also zero the total current bar directly in case polling fires before lock expires
        const tBar = document.getElementById('total-current-bar');
        const tVal = document.getElementById('total-current-val');
        if (tBar) { tBar.style.width = '0%'; tBar.style.background = '#29f0ff'; }
        if (tVal) { tVal.style.color = '#29f0ff'; tVal.textContent = '0.0 A'; }
    }

    const ODOM_MOVE_DEADBAND = 0.05;  // rad/s — below this both wheels treated as stopped

    function updateOdometry(wl, wr, sensorsRunning) {
        const now = Date.now() / 1000;
        if (!sensorsRunning) { _lastOdomTime = null; drawOdometry(); return; }
        if (_lastOdomTime === null) { _lastOdomTime = now; drawOdometry(); return; }
        const dt = Math.min(now - _lastOdomTime, 0.5);
        _lastOdomTime = now;

        const wlC = Math.abs(wl || 0) < ODOM_MOVE_DEADBAND ? 0 : (wl || 0);
        const wrC = Math.abs(wr || 0) < ODOM_MOVE_DEADBAND ? 0 : (wr || 0);

        // Only integrate if at least one wheel is actually moving
        if (wlC !== 0 || wrC !== 0) {
            const vl = wlC * WHEEL_RADIUS_M;
            const vr = wrC * WHEEL_RADIUS_M;
            const v     = (vl + vr) / 2;
            const omega = (vr - vl) / TRACK_WIDTH_M;

            odomTheta += omega * dt;
            odomX += v * Math.sin(odomTheta) * dt;
            odomY += v * Math.cos(odomTheta) * dt;

            odomTrail.push({ x: odomX, y: odomY });
            if (odomTrail.length > ODOM_TRAIL_MAX) odomTrail.shift();
        }

        document.getElementById('odom-x').textContent   = odomX.toFixed(2);
        document.getElementById('odom-y').textContent   = odomY.toFixed(2);
        document.getElementById('odom-hdg').textContent = ((odomTheta * 180 / Math.PI) % 360).toFixed(1);
        drawOdometry();
    }

    // =====================================================================
    // LIVE PITCH GAUGE — semicircular, 0° at top, +right / -left
    // =====================================================================
    let PITCH_LIMIT_DEG = 35;
    let lastGaugePitch = 0;
    function drawPitchGauge(pitchDeg) {
        lastGaugePitch = pitchDeg || 0;
        const canvas = document.getElementById('pitch-gauge');
        if (!canvas) return;
        const dpr  = window.devicePixelRatio || 1;
        const cssW = canvas.clientWidth  || 260;
        const cssH = canvas.clientHeight || 140;
        canvas.width  = Math.round(cssW * dpr);
        canvas.height = Math.round(cssH * dpr);
        const ctx = canvas.getContext('2d');
        ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
        const W = cssW, H = cssH;
        ctx.clearRect(0, 0, W, H);

        const cx = W / 2, cy = H * 0.90;
        const r  = Math.min(W * 0.42, H - 20);
        const TIP_DEG = PITCH_LIMIT_DEG;

        // Angle mapping: pitch=0 → top of arc (270° canvas), +pitch → right, -pitch → left
        // canvas_angle = (270 + pitch) * π/180
        const pitchToAng = d => (270 + d) * Math.PI / 180;

        const MAX_DEG = 50;

        // Remap: ±50° spans the full semicircle (±90° canvas range)
        // pitch=0 → top (270°), pitch=±50 → ends (180°/360°)
        const pitchToAngScaled = d => (270 + (d / MAX_DEG) * 90) * Math.PI / 180;

        // Arc background — full semicircle same size as before
        ctx.beginPath();
        ctx.arc(cx, cy, r, Math.PI, 0, false);
        ctx.strokeStyle = '#1a3040';
        ctx.lineWidth   = 14;
        ctx.stroke();

        // Colored zones using scaled mapping
        function drawArc(startDeg, endDeg, color) {
            ctx.beginPath();
            ctx.arc(cx, cy, r, pitchToAngScaled(startDeg), pitchToAngScaled(endDeg), false);
            ctx.strokeStyle = color;
            ctx.lineWidth   = 14;
            ctx.stroke();
        }
        drawArc(-TIP_DEG,   0,         '#ffdd5766');  // left warn
        drawArc(0,           TIP_DEG,  '#ffdd5766');  // right warn
        drawArc(-MAX_DEG,   -TIP_DEG,  '#ff555566');  // left danger
        drawArc(TIP_DEG,     MAX_DEG,  '#ff555566');  // right danger

        // Tick marks AND labels at every 5°
        for (let d = -MAX_DEG; d <= MAX_DEG; d += 5) {
            const ang     = pitchToAngScaled(d);
            const isMajor = d % 10 === 0;
            const r1      = r - (isMajor ? 16 : 9);
            ctx.beginPath();
            ctx.moveTo(cx + r  * Math.cos(ang), cy + r  * Math.sin(ang));
            ctx.lineTo(cx + r1 * Math.cos(ang), cy + r1 * Math.sin(ang));
            ctx.strokeStyle = isMajor ? '#4a8a9a' : '#2a5060';
            ctx.lineWidth   = isMajor ? 2 : 1;
            ctx.stroke();
            ctx.fillStyle = '#ffffff'; ctx.font = '8px ui-monospace'; ctx.textAlign = 'center';
            ctx.fillText(d + '°', cx + (r - 24) * Math.cos(ang), cy + (r - 24) * Math.sin(ang) + 3);
        }

        // ±35° dashed limit lines
        [-TIP_DEG, TIP_DEG].forEach(d => {
            const ang = pitchToAngScaled(d);
            ctx.beginPath();
            ctx.moveTo(cx, cy - 18);
            ctx.lineTo(cx + (r + 4) * Math.cos(ang), cy + (r + 4) * Math.sin(ang));
            ctx.strokeStyle = '#ff555544'; ctx.lineWidth = 1.5;
            ctx.setLineDash([4, 3]); ctx.stroke(); ctx.setLineDash([]);
        });

        // Blue vertical center line — thinner, starts at pivot
        const pivotY = cy - 18;
        ctx.beginPath();
        ctx.moveTo(cx, pivotY);
        ctx.lineTo(cx, cy - r - 4);
        ctx.strokeStyle = '#29f0ff'; ctx.lineWidth = 1.5;
        ctx.stroke();

        // Needle from pivot
        const clampedPitch = Math.max(-MAX_DEG, Math.min(MAX_DEG, pitchDeg || 0));
        const needleAng    = pitchToAngScaled(clampedPitch);
        const needleColor  = Math.abs(clampedPitch) > TIP_DEG ? '#ff5555' :
                             Math.abs(clampedPitch) > TIP_DEG * 0.7 ? '#ffdd57' : '#29f0ff';
        ctx.beginPath();
        ctx.moveTo(cx, pivotY);
        ctx.lineTo(cx + (r - 8) * Math.cos(needleAng), cy + (r - 8) * Math.sin(needleAng));
        ctx.strokeStyle = needleColor; ctx.lineWidth = 3;
        ctx.stroke();

        // Center pivot — smaller circle
        ctx.beginPath();
        ctx.arc(cx, pivotY, 3, 0, Math.PI * 2);
        ctx.fillStyle = needleColor; ctx.fill();

        // Pitch value text directly below pivot
        ctx.fillStyle = needleColor; ctx.font = 'bold 17px ui-monospace';
        ctx.textAlign = 'center';
        ctx.fillText((clampedPitch >= 0 ? '+' : '') + clampedPitch.toFixed(1) + '°', cx, cy + 1);
    }

    drawPitchGauge(0);

    function drawOdometry() {
        const canvas = document.getElementById('odom-canvas');
        if (!canvas) return;
        const dpr  = window.devicePixelRatio || 1;
        const cssW = canvas.clientWidth  || 260;
        const cssH = canvas.clientHeight || 260;
        if (canvas.width !== Math.round(cssW * dpr) || canvas.height !== Math.round(cssH * dpr)) {
            canvas.width  = Math.round(cssW * dpr);
            canvas.height = Math.round(cssH * dpr);
        }
        const ctx = canvas.getContext('2d');
        ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
        const W = cssW, H = cssH;
        ctx.clearRect(0, 0, W, H);

        // Grid
        ctx.strokeStyle = '#18303c'; ctx.lineWidth = 1;
        const gridM = 0.5;
        // Auto-scale: symmetric view centered on data, minimum ±1.5m
        const VIEW_MIN = 1.5;
        const allX = odomTrail.map(p => p.x).concat([odomX, 0]);
        const allY = odomTrail.map(p => p.y).concat([odomY, 0]);
        const dataMinX = Math.min(...allX), dataMaxX = Math.max(...allX);
        const dataMinY = Math.min(...allY), dataMaxY = Math.max(...allY);
        const ctrX = (dataMinX + dataMaxX) / 2;
        const ctrY = (dataMinY + dataMaxY) / 2;
        const halfX = Math.max((dataMaxX - dataMinX) / 2, VIEW_MIN);
        const halfY = Math.max((dataMaxY - dataMinY) / 2, VIEW_MIN);
        const minX = ctrX - halfX, maxX = ctrX + halfX;
        const minY = ctrY - halfY, maxY = ctrY + halfY;
        const spanX = maxX - minX, spanY = maxY - minY;
        const margin = 24;
        const scaleX = (W - margin * 2) / spanX;
        const scaleY = (H - margin * 2) / spanY;
        const scale  = Math.min(scaleX, scaleY);
        const cx = W / 2 - ctrX * scale;
        const cy = H / 2 + ctrY * scale;

        const toC = (x, y) => ({ px: cx + x * scale, py: cy - y * scale });

        // Grid lines
        const gStart = Math.floor(minX / gridM) * gridM;
        for (let gx = gStart; gx <= maxX + gridM; gx += gridM) {
            const p = toC(gx, minY - gridM);
            const p2 = toC(gx, maxY + gridM);
            ctx.beginPath(); ctx.moveTo(p.px, p.py); ctx.lineTo(p2.px, p2.py); ctx.stroke();
        }
        const gStartY = Math.floor(minY / gridM) * gridM;
        for (let gy = gStartY; gy <= maxY + gridM; gy += gridM) {
            const p = toC(minX - gridM, gy);
            const p2 = toC(maxX + gridM, gy);
            ctx.beginPath(); ctx.moveTo(p.px, p.py); ctx.lineTo(p2.px, p2.py); ctx.stroke();
        }

        // Origin marker
        const o = toC(0, 0);
        ctx.strokeStyle = '#2a4a5a'; ctx.lineWidth = 1;
        ctx.beginPath(); ctx.moveTo(o.px - 5, o.py); ctx.lineTo(o.px + 5, o.py); ctx.stroke();
        ctx.beginPath(); ctx.moveTo(o.px, o.py - 5); ctx.lineTo(o.px, o.py + 5); ctx.stroke();

        // Trail
        if (odomTrail.length > 1) {
            ctx.strokeStyle = '#29f0ff'; ctx.lineWidth = 1.5;
            ctx.globalAlpha = 0.5;
            ctx.beginPath();
            const first = toC(odomTrail[0].x, odomTrail[0].y);
            ctx.moveTo(first.px, first.py);
            for (let i = 1; i < odomTrail.length; i++) {
                const p = toC(odomTrail[i].x, odomTrail[i].y);
                ctx.lineTo(p.px, p.py);
            }
            ctx.stroke();
            ctx.globalAlpha = 1.0;
        }

        // Robot image + heading arrow (arrow behind image)
        const rp = toC(odomX, odomY);
        const imgSize = 60;
        const fwdAngle = Math.PI / 2 - odomTheta;

        // Draw robot image first
        ctx.save();
        ctx.translate(rp.px, rp.py);
        ctx.rotate(odomTheta);
        if (_robotImg.complete && _robotImg.naturalWidth > 0) {
            ctx.imageSmoothingEnabled = true;
            ctx.imageSmoothingQuality = 'high';
            ctx.drawImage(_robotImg, -imgSize / 2, -imgSize / 2, imgSize, imgSize);
        } else {
            ctx.fillStyle = '#29f0ff';
            ctx.beginPath(); ctx.arc(0, 0, 5, 0, Math.PI * 2); ctx.fill();
        }
        ctx.restore();

        // Small arrow above robot image (drawn on top)
        const arrowGap  = imgSize / 2 + 3;   // start just above robot edge
        const arrowLen  = 10;                  // short shaft
        const headSize  = 4;
        const arrowBase = { x: rp.px + arrowGap * Math.cos(fwdAngle),
                            y: rp.py - arrowGap * Math.sin(fwdAngle) };
        const ax = arrowBase.x + arrowLen * Math.cos(fwdAngle);
        const ay = arrowBase.y - arrowLen * Math.sin(fwdAngle);
        const lineAngle = Math.atan2(ay - arrowBase.y, ax - arrowBase.x);
        ctx.strokeStyle = '#ffdd57'; ctx.fillStyle = '#ffdd57'; ctx.lineWidth = 1.5;
        ctx.beginPath();
        ctx.moveTo(arrowBase.x, arrowBase.y);
        ctx.lineTo(ax, ay);
        ctx.stroke();
        ctx.beginPath();
        ctx.moveTo(ax, ay);
        ctx.lineTo(ax - headSize * 1.6 * Math.cos(lineAngle - Math.PI / 6), ay - headSize * 1.6 * Math.sin(lineAngle - Math.PI / 6));
        ctx.lineTo(ax - headSize * 1.6 * Math.cos(lineAngle + Math.PI / 6), ay - headSize * 1.6 * Math.sin(lineAngle + Math.PI / 6));
        ctx.closePath();
        ctx.fill();
    }

    const _robotImg = new Image();
    _robotImg.onload = () => drawOdometry();
    _robotImg.src = '/robot.png';

    drawOdometry();

    function syncSlider(sliderId, numInput, minVal, maxVal) {
        let v = parseFloat(numInput.value);
        if (isNaN(v)) return;
        v = Math.min(Math.max(v, minVal), maxVal);
        document.getElementById(sliderId).value = v;
    }

    function updateTipLimit(deg) {
        deg = Math.round(Math.max(5, Math.min(50, deg)));
        PITCH_LIMIT_DEG = deg;
        const lbl = document.getElementById('tip-limit-label');
        if (lbl) lbl.textContent = deg;
        drawPitchGauge(lastGaugePitch);
        // Update all existing points in the pitch chart limit lines
        const d1 = pitchChart.data.datasets[1].data;
        const d2 = pitchChart.data.datasets[2].data;
        for (let i = 0; i < d1.length; i++) { d1[i] =  deg; }
        for (let i = 0; i < d2.length; i++) { d2[i] = -deg; }
        pitchChart.data.datasets[1].label = '+' + deg + '\u00b0';
        pitchChart.data.datasets[2].label = '-' + deg + '\u00b0';
        pitchChart.update('none');
    }

    function _setPIDSlider(id, sliderId, value) {
        const inp = document.getElementById(id);
        const sld = document.getElementById(sliderId);
        if (inp) inp.value = value;
        if (sld) sld.value = value;
    }

    function loadPIDGains() {
        fetch('/get_pid_gains').then(r => r.json()).then(d => {
            if (!d.gains) return;
            const g = d.gains;
            _setPIDSlider('val-kp',   'slider-kp',   (g.kp   ?? 60.0).toFixed(1));
            _setPIDSlider('val-kd',   'slider-kd',   (g.kd   ?? 12.0).toFixed(1));
            _setPIDSlider('val-ki',   'slider-ki',   (g.ki   ??  0.0).toFixed(2));
            _setPIDSlider('val-trim', 'slider-trim',  (g.trim_deg ?? 0.0).toFixed(1));
            _setPIDSlider('val-tip',  'slider-tip',   (g.tip_deg  ?? 35).toFixed(0));
            const status = document.getElementById('pid-gains-status');
            if (status && d.status !== 'defaults') {
                status.style.color = '#5a8a9a';
                status.textContent = 'Last saved: KP=' + (g.kp??60).toFixed(1) + ' KD=' + (g.kd??12).toFixed(1) + ' trim=' + (g.trim_deg??0).toFixed(1) + '\u00b0';
            }
        }).catch(() => {});
    }

    function applyPIDGains() {
        const kp   = parseFloat(document.getElementById('val-kp').value);
        const kd   = parseFloat(document.getElementById('val-kd').value);
        const ki   = parseFloat(document.getElementById('val-ki').value);
        const trim = parseFloat(document.getElementById('val-trim').value);
        const tip  = parseInt(document.getElementById('val-tip').value);
        const status = document.getElementById('pid-gains-status');
        fetch('/set_pid_gains', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ kp, kd, ki, trim_deg: trim, tip_deg: tip })
        }).then(r => r.json()).then(d => {
            if (d.status === 'ok') {
                status.style.color = '#8dff8a';
                status.textContent = 'Gains applied \u2713';
                logLine('PID', 'Gains: KP=' + kp.toFixed(1) + ' KD=' + kd.toFixed(1) + ' KI=' + ki.toFixed(2) + ' trim=' + trim.toFixed(1) + '\u00b0 tip=' + tip + '\u00b0');
            } else {
                status.style.color = '#ff7b54';
                status.textContent = 'Failed: ' + (d.reason || d.status);
            }
        }).catch(() => {
            status.style.color = '#ff7b54';
            status.textContent = 'Request failed';
        });
    }

    // =====================================================================
    // TEST MODE — feeds simulated sensor + motor data for 30 s
    // =====================================================================
    let _testActive  = false;
    let _testStart   = null;
    let _testTimer   = null;
    let _testDemoTimers = [];
    const TEST_DURATION_S = 30;

    function _clearTestDemoTimers() {
        _testDemoTimers.forEach(id => clearTimeout(id));
        _testDemoTimers = [];
    }

    /** Event-console sequence similar to a real sensors → calibrate → live → PID run. */
    function _startTestModeEventSequence() {
        _clearTestDemoTimers();
        const T = (ms, fn) => { _testDemoTimers.push(setTimeout(fn, ms)); };
        T(120,  () => logLine('SENSOR', 'sensors_3_28 ON'));
        T(350,  () => logLine('SENSORS', 'sensors_3_28 + ultrasonic_bg started — calibrating...'));
        T(600,  () => logLine('SENSORS', 'sensors_3_28.py started — auto-calibrating'));
        T(900,  () => logLine('SENSORS', 'Sensors Calibrating — keep robot still...'));
        T(2600, () => logLine('SENSORS', 'Data flowing — calibration complete'));
        T(3000, () => logLine('PID', 'deployPID_3_28.py started'));
    }

    function toggleTestMode() {
        if (_testActive) { _stopTestMode(); return; }
        _testActive = true;
        _testStart  = Date.now();
        const btn   = document.getElementById('btn-test-mode');
        btn.style.background  = '#004a00';
        btn.style.borderColor = '#00ff88';
        btn.style.color       = '#00ff88';
        btn.textContent       = '⚙ Test ON';
        logLine('TEST', 'UI test mode — 30 s simulated stream (stability + console demo)');
        _startTestModeEventSequence();
        _testTimer = setTimeout(_stopTestMode, TEST_DURATION_S * 1000);
    }

    function _stopTestMode() {
        _testActive = false;
        clearTimeout(_testTimer);
        _clearTestDemoTimers();
        const btn = document.getElementById('btn-test-mode');
        btn.style.background  = '#7a3a00';
        btn.style.borderColor = '#ff8c00';
        btn.style.color       = '#ff8c00';
        btn.textContent       = '⚙ Test';
        // Do not log fake PID/sensor OFF — real processes may still be running on the Pi.
        logLine('TEST', 'UI test mode ended — live polling restored');
    }

    function _makeTestData() {
        const t   = (Date.now() - _testStart) / 1000;
        // Pitch: small noisy oscillation like a robot actively balancing
        const pitchDeg  = 4 * Math.sin(t * 1.8) + 2.5 * Math.sin(t * 3.1) + 1.2 * Math.sin(t * 5.7 + 1.0);
        const pitchRad  = pitchDeg * Math.PI / 180;
        const pitchRate = (4 * 1.8 * Math.cos(t * 1.8) + 2.5 * 3.1 * Math.cos(t * 3.1) + 1.2 * 5.7 * Math.cos(t * 5.7 + 1.0)) * Math.PI / 180;
        // Yaw: slow drift
        const yawDeg    = 30 * Math.sin(t * 0.15);
        const yawRad    = yawDeg * Math.PI / 180;
        const yawRate   = 0.05 * Math.cos(t * 0.15);
        // Wheels: spin proportional to pitch
        const wl = 2.5 * Math.sin(t * 0.6 + 0.3);
        const wr = 2.5 * Math.sin(t * 0.6 - 0.3);
        // Linear velocity
        const linVel = 0.3 * Math.sin(t * 0.4);
        // Motor commands: proportional to pitch
        const motorL = Math.max(-1, Math.min(1,  pitchDeg / 8));
        const motorR = Math.max(-1, Math.min(1,  pitchDeg / 8 + 0.05 * Math.sin(t * 2.3)));

        return {
            sensors_running:    true,
            ultrasonic_running: false,
            pid_running:        true,
            motorwasd_running:  false,
            autonav_running:    false,
            calibrated:         true,
            cache_age_ms:       10,
            obs_source:         'test',
            linear_velocity:    linVel,
            pitch_deg:          pitchDeg,
            pitch_rate:         pitchRate,
            yaw_rate:           yawRate,
            wheel_left_rads:    wl,
            wheel_right_rads:   wr,
            velocity_error:     -linVel,
            rotation_error:     -yawRate,
            yaw_deg:            yawDeg,
            motor_left:         motorL,
            motor_right:        motorR,
            ultrasonic_right_cm: null,
            ultrasonic_left_cm:  null,
            us_age_ms:          null,
            pid_elapsed_s:      t,
            pid_timing:         { hz: '100', avg_ms: '9.8', min_ms: '8.1', max_ms: '12.4' },
            sensor_stats: {
                main_hz: 100, imu_hz: 200, enc_hz: 2000,
                pass_rate: 99.1, successful: Math.round(t * 100),
                failed: 0, total: Math.round(t * 100), age_s: 0.1,
            },
        };
    }

    const _origUpdateLive = updateLiveSensorData;
    updateLiveSensorData = function() {
        if (!_testActive) { _origUpdateLive(); return; }
        // Remaining time countdown
        const elapsed = (Date.now() - _testStart) / 1000;
        if (elapsed >= TEST_DURATION_S) { _stopTestMode(); _origUpdateLive(); return; }
        const btn = document.getElementById('btn-test-mode');
        btn.textContent = '⚙ Test ' + Math.ceil(TEST_DURATION_S - elapsed) + 's';
        // Build fake data and pass directly into the same handler used by fetch
        const d = _makeTestData();
        _handleSensorData(d);
    };

    // Extract the inner handler from updateLiveSensorData so test mode can call it directly.
    // We wrap the fetch callback into a named function _handleSensorData below.
    // To do this cleanly, redefine _origUpdateLive to use _handleSensorData.
    function _handleSensorData(d) {
        function fmt(v, dec) {
            return (v === null || v === undefined) ? (0).toFixed(dec) : parseFloat(v).toFixed(dec);
        }
        // Re-fetch the real data handler by triggering a synthetic call via the original path,
        // but since we can't easily extract the inner closure, we directly replicate the
        // key UI updates here for test mode only — pitch chart, rpm chart, motor bars, gauge.
        const sensOff      = false;
        const sensNotReady = false;

        // Sensor status bar
        const calibBar = document.getElementById('calib-bar');
        if (calibBar) { calibBar.style.color = 'var(--ok)'; calibBar.textContent = 'Live — calibrated (test)'; }
        setLed('led-sensors-panel', true);
        const ageLine = document.getElementById('obs-age-line');
        if (ageLine) { ageLine.textContent = 'Test mode — simulated data'; ageLine.className = 'obs-age'; }

        // Charts
        if (d.pitch_deg !== null && d.pitch_deg !== undefined)
            pushPitchSample(parseFloat(d.pitch_deg));
        if (d.wheel_left_rads !== null && d.wheel_right_rads !== null)
            pushRpmSample(parseFloat(d.wheel_left_rads), parseFloat(d.wheel_right_rads));

        // Live sensor data fields
        document.getElementById('obs-linvel').textContent    = fmt(d.linear_velocity, 3);
        document.getElementById('obs-pitch').textContent     = fmt(d.pitch_deg, 2);
        drawPitchGauge(d.pitch_deg || 0);
        document.getElementById('obs-pitchrate').textContent = fmt(d.pitch_rate, 3);
        document.getElementById('obs-yawrate').textContent   = fmt(d.yaw_rate, 3);
        document.getElementById('obs-whl').textContent       = fmt(d.wheel_left_rads, 3);
        document.getElementById('obs-whr').textContent       = fmt(d.wheel_right_rads, 3);
        document.getElementById('obs-yaw').textContent       = fmt(d.yaw_deg, 2);

        // Motor bars
        const ml = d.motor_left  || 0;
        const mr = d.motor_right || 0;
        function setBar(valId, barId, v) {
            document.getElementById(valId).textContent = v.toFixed(2);
            const pct = Math.abs(v) * 50;
            const bar = document.getElementById(barId);
            bar.style.width = pct + '%';
            bar.style.left  = v >= 0 ? '50%' : (50 - pct) + '%';
            bar.style.background = Math.abs(v) > 0.75 ? '#ff5560' : Math.abs(v) > 0.4 ? '#ffd060' : '#29f0ff';
        }
        setBar('motor-left-val',  'motor-left-bar',  ml);
        setBar('motor-right-val', 'motor-right-bar', mr);

        // Motor stress bar
        const stress = Math.min(100, (Math.abs(ml) + Math.abs(mr)) / 2 * 100);
        const stressBar = document.getElementById('motor-stress-bar');
        stressBar.style.width      = stress + '%';
        stressBar.style.background = stress > 75 ? '#ff5560' : stress > 40 ? '#ffd060' : '#29f0ff';
        document.getElementById('motor-stress-label').textContent =
            stress > 75 ? 'HIGH' : stress > 40 ? 'MED' : stress > 5 ? 'LOW' : 'IDLE';

        // Estimated current
        const cur = (Math.abs(ml) + Math.abs(mr)) * 6;
        const curBar = document.getElementById('current-bar');
        if (curBar) {
            curBar.style.width = Math.min(100, cur / 24 * 100) + '%';
            document.getElementById('current-val').textContent = cur.toFixed(1) + ' A';
        }

        // Odometry
        updateOdometry(d.wheel_left_rads || 0, d.wheel_right_rads || 0, true);

        // PID timing
        if (d.pid_timing) {
            document.getElementById('pid-hz').textContent        = d.pid_timing.hz + ' Hz';
            document.getElementById('pid-avg-ms').textContent    = d.pid_timing.avg_ms;
            document.getElementById('pid-minmax-ms').textContent = d.pid_timing.min_ms + ' / ' + d.pid_timing.max_ms;
        }

        // Sensor pipeline stats
        if (d.sensor_stats) {
            const ss = d.sensor_stats;
            const sp = document.getElementById('sp-cache-age');
            if (sp) sp.textContent = fmt(d.cache_age_ms, 0);
            const src = document.getElementById('sp-source');
            if (src) src.textContent = d.obs_source || '--';
            const mhz = document.getElementById('sp-main-hz');
            if (mhz) mhz.textContent = ss.main_hz;
            const ihz = document.getElementById('sp-imu-hz');
            if (ihz) ihz.textContent = ss.imu_hz;
            const ehz = document.getElementById('sp-enc-hz');
            if (ehz) ehz.textContent = ss.enc_hz;
            const pr = document.getElementById('sp-pass-rate');
            if (pr) pr.textContent = ss.pass_rate + '%';
            const rd = document.getElementById('sp-reads');
            if (rd) rd.textContent = ss.total;
        }

        // Same as live path: stability timer / best run + top-bar LEDs
        setLed('led-sensors', true);
        setLed('led-pid', d.pid_running);
        setLed('led-ultrasonic', !!d.ultrasonic_running);
        updateStabilityTimer(d.pid_running, d.pid_elapsed_s);

        document.getElementById('imu-time').textContent = new Date().toLocaleTimeString();
    }

    setInterval(updateLiveSensorData, 100);
    updateLiveSensorData();
    loadPIDGains();

    // =====================================================================
    // LEGACY SENSOR TOGGLE (sensors.py)
    // =====================================================================
    function toggleSensor(on) {
        fetch(on ? '/start_sensors' : '/stop_sensors', { method: 'POST' })
            .then(r => r.json())
            .then(data => {
                const s = document.getElementById('sensor-switch');
                const why = data.reason ? String(data.reason).replace(/\\s+/g, ' ').trim().slice(0, 400) : '';
                if (on && (data.status === "started" || data.status === "already_running")) {
                    s.textContent = "ON"; s.className = "ON";
                    logLine("SENSOR", "sensors_3_28 ON");
                } else if (!on && data.status === "stopped") {
                    s.textContent = "OFF"; s.className = "OFF";
                    logLine("SENSOR", "sensors_3_28 OFF");
                } else {
                    logLine("SENSOR", "Error toggling sensors" + (why ? (": " + why) : (" — " + (data.status || "?"))));
                }
            })
            .catch(() => logLine("SENSOR", "Error toggling sensors (no response)"));
    }

    // =====================================================================
    // SENSOR SUITE (top-bar Sensors ON / OFF)
    // =====================================================================
    function startSensorSuite() {
        fetch('/start_sensor_suite', { method: 'POST' })
            .then(r => r.json())
            .then(d => {
                if (d.status === 'started' || d.status === 'already_running') {
                    setButtonPair('btn-tr-sensors-on', 'btn-tr-sensors-off', true);
                    logLine('SENSORS', 'sensors_3_28 + ultrasonic_bg started — calibrating...');
                } else {
                    const msg = d.reason ? String(d.reason).replace(/[\\s]+/g, ' ').trim().slice(0, 400) : d.status;
                    logLine('SENSORS', 'Failed to start: ' + msg);
                }
            })
            .catch(() => logLine('SENSORS', 'Failed to start (no response)'));
    }
    function stopSensorSuite() {
        fetch('/stop_sensor_suite', { method: 'POST' })
            .then(() => {
                setButtonPair('btn-tr-sensors-on', 'btn-tr-sensors-off', false);
                logLine('SENSORS', 'sensors_3_28 + ultrasonic_bg stopped');
            });
    }

    // =====================================================================
    // PIPELINE CONTROL BUTTONS
    // =====================================================================
    function startSensors() {
        fetch('/start_sensors', { method: 'POST' })
            .then(r => r.json())
            .then(d => {
                if (d.status === 'started' || d.status === 'already_running') {
                    setButtonPair('btn-sensors-on', 'btn-sensors-off', true);
                    logLine('SENSORS', 'sensors_3_28.py started — auto-calibrating');
                } else {
                    const msg = d.reason ? String(d.reason).replace(/[\s]+/g, ' ').trim().slice(0, 500) : (d.status || 'failed');
                    logLine('SENSORS', 'Failed to start: ' + msg);
                }
            })
            .catch(() => logLine('SENSORS', 'Failed to start (no JSON response)'));
    }
    function stopSensors() {
        fetch('/stop_sensors', { method: 'POST' })
            .then(() => {
                setButtonPair('btn-sensors-on', 'btn-sensors-off', false);
                logLine('SENSORS', 'sensors_3_28.py stopped');
                _sensorsWereRunning = false;
                _calibStartTime = null;
            });
    }
    function startUltrasonic() {
        fetch('/start_ultrasonic', { method: 'POST' })
            .then(r => r.json())
            .then(d => {
                if (d.status === 'started') {
                    setButtonPair('btn-ultrasonic-on', 'btn-ultrasonic-off', true);
                    logLine('ULTRASON', 'ultrasonic_bg.py started');
                } else {
                    logLine('ULTRASON', 'Failed: ' + (d.reason || d.status));
                }
            });
    }
    function stopUltrasonic() {
        fetch('/stop_ultrasonic', { method: 'POST' })
            .then(() => {
                setButtonPair('btn-ultrasonic-on', 'btn-ultrasonic-off', false);
                logLine('ULTRASON', 'ultrasonic_bg.py stopped');
            });
    }
    function startPID() {
        fetch('/start_pid', { method: 'POST' })
            .then(r => r.json())
            .then(d => {
                if (d.status === 'blocked') logLine('PID', 'Blocked: ' + d.reason);
                else if (d.status === 'started') {
                    setButtonPair('btn-pid-on', 'btn-pid-off', true);
                    logLine('PID', 'deployPID_3_28.py started');
                } else {
                    const msg = d.reason ? String(d.reason).replace(/[\s]+/g, ' ').trim().slice(0, 500) : 'Failed to start';
                    logLine('PID', 'Failed to start: ' + msg);
                }
            })
            .catch(() => logLine('PID', 'Failed to start (no JSON response)'));
    }
    function stopPID() {
        fetch('/stop_pid', { method: 'POST' })
            .then(() => {
                setButtonPair('btn-pid-on', 'btn-pid-off', false);
                logLine('PID', 'deployPID_3_28.py stopped');
            });
    }

    function emergencyStop() {
        wasdStop();
        Promise.all([
            fetch('/stop_pid',   { method: 'POST' }),
            fetch('/motor_off',  { method: 'POST' }),
            fetch('/motor_cmd',  { method: 'POST', headers: {'Content-Type':'application/json'}, body: JSON.stringify({fwd:0,turn:0,speed:0}) }),
        ]).then(() => {
            setButtonPair('btn-pid-on',      'btn-pid-off',      false);
            setButtonPair('btn-motorwasd-on','btn-motorwasd-off', false);
            logLine('ESTOP', '⛔ EMERGENCY STOP — PID and motors halted');
        });
    }
    function motorOn() {
        fetch('/motor_on', { method: 'POST' })
            .then(r => r.json())
            .then(d => {
                if (d.status === 'blocked') logLine('MOTOR', 'Blocked: ' + d.reason);
                else if (d.status === 'started') {
                    setButtonPair('btn-motorwasd-on', 'btn-motorwasd-off', true);
                    logLine('MOTOR', 'Manual motors started');
                } else {
                    logLine('MOTOR', 'Failed to start motorwasd');
                }
            });
    }
    function motorOff() {
        fetch('/motor_off', { method: 'POST' })
            .then(() => {
                setButtonPair('btn-motorwasd-on', 'btn-motorwasd-off', false);
                logLine('MOTOR', 'Manual motors stopped');
            });
    }
    function startPigpiod() {
        fetch('/start_pigpiod', { method: 'POST' })
            .then(r => r.json())
            .then(d => {
                if (d.status === 'started') {
                    setButtonPair('btn-tr-motors-on', 'btn-tr-motors-off', true);
                    logLine('MOTORS', 'pigpiod started');
                } else {
                    logLine('MOTORS', 'pigpiod failed: ' + (d.reason || d.status));
                }
            });
    }
    function startMotorTest() {
        fetch('/start_motor_test', { method: 'POST' })
            .then(r => r.json())
            .then(d => logLine("MOTOR", d.status === "started" ? "Motor Test started" : "Motor Test failed to start"));
    }
    function stopMotorTest() {
        fetch('/stop_motor_test', { method: 'POST' })
            .then(() => logLine("MOTOR", "Motor Test stopped"));
    }
    function startAutonav() {
        fetch('/start_autonav', { method: 'POST' })
            .then(r => r.json())
            .then(d => {
                if (d.status === 'started') {
                    setButtonPair('btn-tr-autonav-on', 'btn-tr-autonav-off', true);
                    logLine('AUTONAV', 'AutoNav started');
                } else {
                    logLine('AUTONAV', 'AutoNav failed to start');
                }
            });
    }
    function stopAutonav() {
        fetch('/stop_autonav', { method: 'POST' })
            .then(() => {
                setButtonPair('btn-tr-autonav-on', 'btn-tr-autonav-off', false);
                logLine('AUTONAV', 'AutoNav stopped');
            });
    }

    // =====================================================================
    // SLIDESHOW (PDF.js)
    // =====================================================================
    let _pdfDoc     = null;
    let _pdfPage    = 1;
    let _pdfLoaded  = false;
    let _pdfRendering = false;

    async function _initPDF() {
        if (_pdfLoaded) return;
        try {
            const pdfjsLib = await import('https://cdn.jsdelivr.net/npm/pdfjs-dist@4.4.168/build/pdf.min.mjs');
            pdfjsLib.GlobalWorkerOptions.workerSrc = 'https://cdn.jsdelivr.net/npm/pdfjs-dist@4.4.168/build/pdf.worker.min.mjs';
            _pdfDoc   = await pdfjsLib.getDocument('/presentation.pdf').promise;
            _pdfLoaded = true;
            _pdfPage   = 1;
            _pdfRender(_pdfPage);
        } catch(e) {
            console.error('PDF.js failed:', e);
        }
    }

    async function _pdfRender(pageNum) {
        if (!_pdfDoc || _pdfRendering) return;
        _pdfRendering = true;
        const page    = await _pdfDoc.getPage(pageNum);
        const wrap    = document.getElementById('pdf-canvas-wrap');
        const canvas  = document.getElementById('pdf-canvas');
        const ctx     = canvas.getContext('2d');
        const scale   = Math.min(
            (wrap.clientWidth  - 0) / page.getViewport({scale:1}).width,
            (wrap.clientHeight - 0) / page.getViewport({scale:1}).height
        );
        const vp = page.getViewport({scale});
        canvas.width  = vp.width;
        canvas.height = vp.height;
        await page.render({canvasContext: ctx, viewport: vp}).promise;
        _pdfRendering = false;

        document.getElementById('pdf-page-label').textContent = pageNum + ' / ' + _pdfDoc.numPages;
        const atFirst = pageNum <= 1;
        const atLast  = pageNum >= _pdfDoc.numPages;
        ['pdf-arrow-prev','pdf-btn-prev'].forEach(id => document.getElementById(id).disabled = atFirst);
        ['pdf-arrow-next','pdf-btn-next'].forEach(id => document.getElementById(id).disabled = atLast);
        // Show video buttons only on slide 8
        const vbtns = document.getElementById('pdf-video-btns');
        if (vbtns) vbtns.style.display = pageNum === 8 ? 'flex' : 'none';
    }

    function openVideo(src) {
        const ov = document.getElementById('video-overlay');
        const vp = document.getElementById('video-player');
        vp.src = src;
        ov.style.display = 'flex';
        vp.play();
    }

    function closeVideo() {
        const vp = document.getElementById('video-player');
        vp.pause();
        vp.src = '';
        document.getElementById('video-overlay').style.display = 'none';
    }

    function toggleFullscreen() {
        const modal = document.getElementById('slideshow-modal');
        if (!document.fullscreenElement) {
            modal.requestFullscreen().catch(() => {});
        } else {
            document.exitFullscreen();
        }
    }

    document.addEventListener('fullscreenchange', () => {
        const btn = document.getElementById('slideshow-fs');
        if (btn) btn.textContent = document.fullscreenElement ? '\u29C5' : '\u26F6';
        // Re-render at new size after fullscreen transition completes
        setTimeout(() => _pdfRender(_pdfPage), 150);
    });

    function openSlideshow() {
        document.getElementById('slideshow-overlay').classList.add('open');
        _initPDF();
    }

    function closeSlideshow() {
        document.getElementById('slideshow-overlay').classList.remove('open');
    }

    function pdfPrev() {
        if (_pdfPage > 1) { _pdfPage--; _pdfRender(_pdfPage); }
    }

    function pdfNext() {
        if (_pdfDoc && _pdfPage < _pdfDoc.numPages) { _pdfPage++; _pdfRender(_pdfPage); }
    }

    document.addEventListener('keydown', e => {
        const ov = document.getElementById('slideshow-overlay');
        if (!ov.classList.contains('open')) return;
        if (e.key === 'Escape') {
            const vov = document.getElementById('video-overlay');
            if (vov.style.display === 'flex') { closeVideo(); return; }
            closeSlideshow();
        }
        if (e.key === 'ArrowRight') pdfNext();
        if (e.key === 'ArrowLeft')  pdfPrev();
    });

    function logLine(tag, text) {
        const c = document.getElementById('console');
        if (!c) return;
        const div = document.createElement('div');
        div.className = 'line';
        const t = document.createElement('span');
        t.className = 'tag'; t.textContent = tag;
        const span = document.createElement('span');
        span.textContent = text;
        div.appendChild(t); div.appendChild(span);
        c.appendChild(div);
        c.scrollTop = c.scrollHeight;
    }

    // =====================================================================
    // LIVE PITCH CHART
    // =====================================================================
    const PITCH_WINDOW_S  = 30;
    const pitchTimes  = [];
    const pitchValues = [];
    let   pitchT0     = null;

    function resetPitchChart() {
        pitchTimes.length  = 0;
        pitchValues.length = 0;
        pitchChart.data.datasets[1].data.length = 0;
        pitchChart.data.datasets[2].data.length = 0;
        pitchT0 = null;
        pitchChart.update('none');
    }

    const pitchChart = new Chart(document.getElementById('pitch-chart'), {
        type: 'line',
        data: {
            labels: pitchTimes,
            datasets: [
                {
                    label: 'Pitch (°)',
                    data: pitchValues,
                    borderColor: '#29f0ff',
                    borderWidth: 1.5,
                    pointRadius: 0,
                    tension: 0.2,
                    fill: false,
                },
                {
                    label: '+35°',
                    data: [],
                    borderColor: '#ff5560',
                    borderWidth: 1,
                    borderDash: [4, 3],
                    pointRadius: 0,
                    fill: false,
                },
                {
                    label: '-35°',
                    data: [],
                    borderColor: '#ff5560',
                    borderWidth: 1,
                    borderDash: [4, 3],
                    pointRadius: 0,
                    fill: false,
                },
            ]
        },
        options: {
            animation: false,
            responsive: true,
            plugins: { legend: { display: false } },
            scales: {
                x: {
                    ticks: { color: '#ffffff', maxTicksLimit: 6,
                             callback: (v, i) => pitchTimes[i] !== undefined ? pitchTimes[i].toFixed(0) + 's' : '' },
                    grid:  { color: '#18303c' },
                },
                y: {
                    min: -40, max: 40,
                    ticks: {
                        color: '#ffffff',
                        stepSize: 5,
                        padding: 4,
                        callback: function(value) {
                            if (value % 10 === 0) return value + '°';
                            return '';
                        },
                        font: function(context) {
                            return { size: context.tick && context.tick.value % 10 === 0 ? 11 : 9 };
                        }
                    },
                    grid: {
                        color: function(context) {
                            return context.tick.value % 10 === 0 ? '#243d4a' : '#18303c';
                        }
                    },
                    title: { display: true, text: 'Degree', color: '#ffffff', font: { size: 11 } },
                }
            }
        }
    });

    function pushPitchSample(pitchDeg) {
        const now = Date.now() / 1000;
        if (pitchT0 === null) pitchT0 = now;
        const t = now - pitchT0;

        pitchTimes.push(t);
        pitchValues.push(pitchDeg);
        pitchChart.data.datasets[1].data.push(PITCH_LIMIT_DEG);
        pitchChart.data.datasets[2].data.push(-PITCH_LIMIT_DEG);

        // Drop points older than the window
        while (pitchTimes.length > 0 && t - pitchTimes[0] > PITCH_WINDOW_S) {
            pitchTimes.shift();
            pitchValues.shift();
            pitchChart.data.datasets[1].data.shift();
            pitchChart.data.datasets[2].data.shift();
        }

        pitchChart.update('none');
    }

    // =====================================================================
    // ENCODER RPM CHART
    // =====================================================================
    const RPM_WINDOW_S  = 30;
    const rpmTimes      = [];
    const rpmLeftVals   = [];
    const rpmRightVals  = [];
    let   rpmT0         = null;

    function resetRpmChart() {
        rpmTimes.length     = 0;
        rpmLeftVals.length  = 0;
        rpmRightVals.length = 0;
        rpmT0 = null;
        rpmChart.update('none');
    }

    const rpmChart = new Chart(document.getElementById('rpm-chart'), {
        type: 'line',
        data: {
            labels: rpmTimes,
            datasets: [
                {
                    label: 'Left RPM',
                    data: rpmLeftVals,
                    borderColor: '#29f0ff',
                    borderWidth: 1.5,
                    pointRadius: 0,
                    tension: 0.2,
                    fill: false,
                },
                {
                    label: 'Right RPM',
                    data: rpmRightVals,
                    borderColor: '#ffd060',
                    borderWidth: 1.5,
                    pointRadius: 0,
                    tension: 0.2,
                    fill: false,
                },
            ]
        },
        options: {
            animation: false,
            responsive: true,
            plugins: { legend: { display: false } },
            scales: {
                x: {
                    ticks: { color: '#ffffff', maxTicksLimit: 6,
                             callback: (v, i) => rpmTimes[i] !== undefined ? rpmTimes[i].toFixed(0) + 's' : '' },
                    grid:  { color: '#18303c' },
                },
                y: {
                    ticks: { color: '#ffffff' },
                    grid:  { color: '#18303c' },
                    title: { display: true, text: 'RPM', color: '#ffffff', font: { size: 11 } },
                }
            }
        }
    });

    // Rolling average buffers for spike suppression (5-sample ~= 500ms at 10Hz)
    const RPM_SMOOTH = 5;
    const _rpmLBuf = [], _rpmRBuf = [];

    function _rollingAvg(buf, val) {
        buf.push(val);
        if (buf.length > RPM_SMOOTH) buf.shift();
        return buf.reduce((a, b) => a + b, 0) / buf.length;
    }

    function pushRpmSample(leftRadS, rightRadS) {
        const now = Date.now() / 1000;
        if (rpmT0 === null) rpmT0 = now;
        const t = now - rpmT0;
        const TO_RPM = 60 / (2 * Math.PI);
        const leftRpm  = _rollingAvg(_rpmLBuf, leftRadS  * TO_RPM);
        const rightRpm = _rollingAvg(_rpmRBuf, rightRadS * TO_RPM);
        rpmTimes.push(t);
        rpmLeftVals.push(parseFloat(leftRpm.toFixed(1)));
        rpmRightVals.push(parseFloat(rightRpm.toFixed(1)));
        while (rpmTimes.length > 0 && t - rpmTimes[0] > RPM_WINDOW_S) {
            rpmTimes.shift();
            rpmLeftVals.shift();
            rpmRightVals.shift();
        }
        rpmChart.update('none');
    }

    // =====================================================================
    // JOYSTICK
    // =====================================================================
    const area = document.getElementById('joystick-area');
    const knob = document.getElementById('joystick-knob');
    let joyCenter = { x: 60, y: 60 };
    let joyRadius = 44, joyActive = false, lastJoystickLog = 0;
    let joyLastX = 0, joyLastY = 0;
    const joystickThrottleMs = 250;

    function setKnob(x, y) { knob.style.left = x+'px'; knob.style.top = y+'px'; }

    function motorLog(msg) {
        const log = document.getElementById('joy-log');
        if (!log) return;
        const div = document.createElement('div');
        const t = new Date().toLocaleTimeString('en-GB', { hour12: false });
        div.textContent = t + '  ' + msg;
        div.style.cssText = 'border-bottom:1px solid #18303c;padding-bottom:3px;margin-bottom:3px;';
        log.insertBefore(div, log.firstChild);
        while (log.children.length > 200) log.removeChild(log.lastChild);
    }

    function sendJoystick(x, y) {
        const now = Date.now();
        const shouldLog = now - lastJoystickLog > joystickThrottleMs;
        if (shouldLog) { lastJoystickLog = now; motorLog('joy  x:'+x.toFixed(2)+'  y:'+y.toFixed(2)); }
        fetch('/direction_ajax', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ x, y, speed: getSpeed() })
        }).then(r => r.json()).then(data => {
        }).catch(() => {});
    }

    function updateJoystick(clientX, clientY) {
        const rect = area.getBoundingClientRect();
        let dx = clientX - rect.left - joyCenter.x;
        let dy = clientY - rect.top  - joyCenter.y;
        const dist = Math.sqrt(dx*dx + dy*dy);
        if (dist > joyRadius) { const s = joyRadius/dist; dx*=s; dy*=s; }
        setKnob(joyCenter.x + dx, joyCenter.y + dy);
        const normX = Math.max(-1, Math.min(1, dx/joyRadius));
        const normY = Math.max(-1, Math.min(1,  -dy/joyRadius));
        document.getElementById('joy-x').textContent = normX.toFixed(2);
        document.getElementById('joy-y').textContent = normY.toFixed(2);
        joyLastX = normX; joyLastY = normY;
        sendJoystick(normX, normY);
    }

    function resetJoystick() {
        knob.style.left = '';
        knob.style.top = '';
        document.getElementById('joy-x').textContent = "0.00";
        document.getElementById('joy-y').textContent = "0.00";
        sendJoystick(0, 0);
    }

    area.addEventListener('mousedown', e => { joyActive=true; updateJoystick(e.clientX, e.clientY); });
    window.addEventListener('mousemove', e => { if (joyActive) updateJoystick(e.clientX, e.clientY); });
    window.addEventListener('mouseup', () => { if (joyActive) { joyActive=false; resetJoystick(); } });
    area.addEventListener('touchstart', e => { e.preventDefault(); joyActive=true; updateJoystick(e.touches[0].clientX, e.touches[0].clientY); }, {passive:false});
    area.addEventListener('touchmove',  e => { e.preventDefault(); if (joyActive) updateJoystick(e.touches[0].clientX, e.touches[0].clientY); }, {passive:false});
    area.addEventListener('touchend',   e => { e.preventDefault(); if (joyActive) { joyActive=false; resetJoystick(); } }, {passive:false});

    // =====================================================================
    // WASD MANUAL MOTOR CONTROL
    // =====================================================================
    let wasdActive = null;
    const wasdBtnMap = { fwd:'btn-fwd', rev:'btn-rev', left:'btn-left', right:'btn-right' };
    const keyDirMap  = { ArrowUp:'fwd', ArrowDown:'rev', ArrowLeft:'left', ArrowRight:'right' };

    function getSpeed() { return parseInt(document.getElementById('speed-slider').value, 10) / 100.0; }

    function dirToCmd(dir) {
        const speed = getSpeed();
        if (dir === 'fwd')   return { fwd:  1.0, turn:  0.0, speed };
        if (dir === 'rev')   return { fwd: -1.0, turn:  0.0, speed };
        if (dir === 'left')  return { fwd:  0.0, turn: -1.0, speed };
        if (dir === 'right') return { fwd:  0.0, turn:  1.0, speed };
        return { fwd: 0.0, turn: 0.0, speed };
    }

    function sendMotorCmd(dir) {
        fetch('/motor_cmd', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify(dirToCmd(dir))
        }).catch(() => {});
    }

    // Heartbeat: re-send active command every 150 ms so motor_wasd.py
    // never sees a stale timestamp (stale threshold = 300 ms)
    setInterval(() => {
        if (wasdActive) sendMotorCmd(wasdActive);
        if (joyActive)  sendJoystick(joyLastX, joyLastY);
    }, 150);

    const dirLabel = { fwd: '▲ fwd', rev: '▼ rev', left: '◄ left', right: '► right' };

    function wasdToggle(dir) {
        if (wasdActive === dir) { wasdStop(); return; }
        // Deactivate previous button highlight
        if (wasdActive) {
            const prev = document.getElementById(wasdBtnMap[wasdActive]);
            if (prev) prev.classList.remove('pressed');
        }
        wasdActive = dir;
        const el = document.getElementById(wasdBtnMap[dir]);
        if (el) el.classList.add('pressed');
        motorLog('key  ' + (dirLabel[dir] || dir));
        sendMotorCmd(dir);
    }

    function wasdStop() {
        if (wasdActive) {
            const el = document.getElementById(wasdBtnMap[wasdActive]);
            if (el) el.classList.remove('pressed');
            wasdActive = null;
        }
        motorLog('key  ■ stop');
        sendMotorCmd(null);
    }

    window.addEventListener('keydown', e => {
        const d = keyDirMap[e.key];
        if (d) { e.preventDefault(); if (!e.repeat) wasdToggle(d); }
    });

    // ── Stability Timer + Best Run Tracker ───────────────────────────────
    let _stabWasRunning = false;
    let _stabRunStart   = null;   // Date.now() when current run began
    let _stabRunNo      = 0;
    let _stabHistory    = [];     // [{run, duration_s, startedAt}]
    let _stabBest       = 0;      // seconds

    function _fmtDur(s) {
        const m = Math.floor(s / 60), sec = Math.floor(s % 60);
        return (m > 0 ? m + 'm ' : '') + sec + 's';
    }

    function _fmtTime(ts) {
        return new Date(ts).toLocaleTimeString('en-GB', { hour12: false });
    }

    function _rebuildHistory() {
        const el = document.getElementById('stab-history');
        if (!el) return;
        if (_stabHistory.length === 0) {
            el.innerHTML = '<div style="font-family:ui-monospace;font-size:11px;color:#aaa;text-align:center;padding:8px 0;">No runs yet — start PID to begin</div>';
            return;
        }
        el.innerHTML = '';
        for (let i = _stabHistory.length - 1; i >= 0; i--) {
            const r = _stabHistory[i];
            const isBest = r.duration_s === _stabBest && _stabBest > 0;
            const row = document.createElement('div');
            row.style.cssText = 'display:flex;justify-content:space-between;align-items:center;padding:3px 4px;border-bottom:1px solid #18303c;font-family:ui-monospace;font-size:11px;' + (isBest ? 'background:#1a2a10;' : '');
            row.innerHTML =
                '<span style="color:#5a8a9a;">Run ' + r.run + '</span>' +
                '<span style="color:' + (isBest ? '#ffd060' : '#a0c8d8') + ';font-weight:' + (isBest ? 'bold' : 'normal') + ';">' + _fmtDur(r.duration_s) + (isBest ? ' ★' : '') + '</span>' +
                '<span style="color:#3a5a6a;">' + _fmtTime(r.startedAt) + '</span>';
            el.appendChild(row);
        }
    }

    function updateStabilityTimer(pidRunning, elapsedS) {
        const dot  = document.getElementById('stab-dot');
        const curr = document.getElementById('stab-current');
        const best = document.getElementById('stab-best');

        if (pidRunning) {
            if (!_stabWasRunning) {
                // New run started
                _stabRunStart   = Date.now();
                _stabRunNo++;
                _stabWasRunning = true;
            }
            const secs = elapsedS !== null && elapsedS !== undefined ? elapsedS : (Date.now() - _stabRunStart) / 1000;
            if (dot)  { dot.style.background = '#29f0ff'; dot.style.boxShadow = '0 0 6px #29f0ff88'; }
            if (curr) curr.textContent = _fmtDur(secs);
        } else {
            if (_stabWasRunning && _stabRunStart !== null) {
                // Run just ended — record it
                const dur = Math.round((Date.now() - _stabRunStart) / 1000);
                if (dur > 0) {
                    _stabHistory.push({ run: _stabRunNo, duration_s: dur, startedAt: _stabRunStart });
                    if (dur > _stabBest) _stabBest = dur;
                    _rebuildHistory();
                }
                _stabRunStart   = null;
                _stabWasRunning = false;
            }
            if (dot)  { dot.style.background = '#2a4a5a'; dot.style.boxShadow = 'none'; }
            if (curr) curr.textContent = '--:--';
        }
        if (best) best.textContent = _stabBest > 0 ? _fmtDur(_stabBest) : '--:--';
    }

    function clearRunHistory() {
        _stabHistory = []; _stabBest = 0; _stabRunNo = 0;
        _rebuildHistory();
        document.getElementById('stab-best').textContent = '--:--';
    }
</script>
</body>
</html>
"""


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=8080, debug=False)
