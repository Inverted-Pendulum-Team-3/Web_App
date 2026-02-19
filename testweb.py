from flask import Flask, jsonify, request
import subprocess
import os
import signal
from datetime import datetime

app = Flask(__name__)

# ================================
# PROGRAM DEFINITIONS (VARIABLES)
# ================================
IMU_SCRIPT = "imuencoderv4.py"
MOTOR_TEST_SCRIPT = "motortest.py"
ML_SCRIPT = "ml.py"
AUTONAV_SCRIPT = "autonav.py"

PROGRAMS = {
    "imu": IMU_SCRIPT,
    "motortest": MOTOR_TEST_SCRIPT,
    "ml": ML_SCRIPT,
    "autonav": AUTONAV_SCRIPT,
}

PROGRAM_PROCS = {name: None for name in PROGRAMS}

# Joystick throttling
last_joystick_send = 0
joystick_throttle_interval = 0.25  # seconds


def log_to_file(tag, message):
    """Log events to numbers.txt with timestamp"""
    try:
        with open("numbers.txt", "a") as f:
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


def start_program(name):
    if name not in PROGRAMS:
        log_to_file("ERROR", f"Unknown program '{name}'")
        return False
    if is_program_running(name):
        return True
    script = PROGRAMS[name]
    try:
        PROGRAM_PROCS[name] = subprocess.Popen(
            ["python3", script],
            preexec_fn=os.setsid,
        )
        log_to_file("SYSTEM", f"Program '{name}' ({script}) started")
        return True
    except Exception as e:
        log_to_file("ERROR", f"Failed to start program '{name}': {e}")
        PROGRAM_PROCS[name] = None
        return False


def stop_program(name):
    if not is_program_running(name):
        return
    try:
        proc = PROGRAM_PROCS[name]
        os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        log_to_file("SYSTEM", f"Program '{name}' stopped")
    except Exception as e:
        log_to_file("ERROR", f"Failed to stop program '{name}': {e}")
    finally:
        PROGRAM_PROCS[name] = None


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
# SENSOR LINE PARSER
# =======================
def parse_sensor_line(line):
    """
    Parse lines like:

    18:22:42, IMU1, Forward/backwards Tilt in rad/s, 0.0000, ...,
    IMU1 Linear Velocity, 0.0000, IMU1's X-velocity, 0.0000, ...
    Robot Yaw Rate, 0.0000, Pendulum Angular Velocity, 0.0000, ...
    EncoderL, 0.0000, Direction, stopped, EncoderR, 0.0000, Direction, stopped | Ultrasonic Right: 99.4 cm   Ultrasonic Left: --- cm
    """
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
        "IMU2": {
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
        "Extra": {
            "Robot Yaw Rate": "",
            "Pendulum Angular Velocity": "",
            "Pendulum Angle": "",
            "Pendulum Angle Deg": "",
            "Ultrasonic Right": "",
            "Ultrasonic Left": "",
        },
        "EncoderL": {
            "Speed": "",
            "Direction": "",
        },
        "EncoderR": {
            "Speed": "",
            "Direction": "",
        },
    }

    def fmt(val):
        try:
            return f"{float(val):.1f}"
        except Exception:
            return val

    try:
        main_part = line
        ultrasonic_part = ""
        if "|" in line:
            main_part, ultrasonic_part = [s.strip() for s in line.split("|", 1)]

        tokens = [tok.strip() for tok in main_part.split(",")]
        idx = 0

        if idx < len(tokens) and ":" in tokens[idx]:
            result["IMU1"]["Time"] = tokens[idx]
            idx += 1

        if idx < len(tokens) and tokens[idx] == "IMU1":
            idx += 1
            imu1_map = {
                "Forward/backwards Tilt in rad/s": "Forward/backwards Tilt",
                "Side-to-Side tilt in rad/s": "Side-to-Side Tilt",
                "Yaw in rad/s": "Yaw",
                "Pitch Rate in rad/s": "Pitch Rate",
                "Roll Rate in rad/s": "Roll Rate",
                "Rotational Velocity in rad/s": "Rotational Velocity",
            }
            while idx + 1 < len(tokens) and tokens[idx] in imu1_map:
                label = tokens[idx]
                value = tokens[idx + 1]
                result["IMU1"][imu1_map[label]] = fmt(value)
                idx += 2

        if idx < len(tokens) and tokens[idx] == "IMU2":
            idx += 1
            imu2_map = {
                "Tilt in rad/s": "Forward/backwards Tilt",
                "Side-to-Side tilt in rad/s": "Side-to-Side Tilt",
                "Yaw in rad/s": "Yaw",
                "Pitch Rate in rad/s": "Pitch Rate",
                "Roll Rate in rad/s": "Roll Rate",
                "Rotational Velocity in rad/s": "Rotational Velocity",
            }
            while idx + 1 < len(tokens) and tokens[idx] in imu2_map:
                label = tokens[idx]
                value = tokens[idx + 1]
                result["IMU2"][imu2_map[label]] = fmt(value)
                idx += 2

        if idx < len(tokens) and tokens[idx] == "IMU1 Linear Velocity":
            idx += 1
            if idx < len(tokens):
                result["IMU1Linear"]["Linear Velocity"] = fmt(tokens[idx])
                idx += 1

        if idx < len(tokens) and tokens[idx] == "IMU1's X-velocity":
            idx += 1
            if idx < len(tokens):
                result["IMU1Linear"]["X velocity"] = fmt(tokens[idx])
                idx += 1

        if idx < len(tokens) and tokens[idx] == "IMU1's Y-velocity":
            idx += 1
            if idx < len(tokens):
                result["IMU1Linear"]["Y velocity"] = fmt(tokens[idx])
                idx += 1

        if idx < len(tokens) and tokens[idx] == "Robot Yaw Rate":
            idx += 1
            if idx < len(tokens):
                result["Extra"]["Robot Yaw Rate"] = fmt(tokens[idx])
                idx += 1

        if idx < len(tokens) and tokens[idx] == "Pendulum Angular Velocity":
            idx += 1
            if idx < len(tokens):
                result["Extra"]["Pendulum Angular Velocity"] = fmt(tokens[idx])
                idx += 1

        if idx < len(tokens) and tokens[idx] == "Pendulum Angle":
            idx += 1
            if idx < len(tokens):
                result["Extra"]["Pendulum Angle"] = fmt(tokens[idx])
                idx += 1

        if idx < len(tokens) and tokens[idx] == "Pendulum Angle (deg)":
            idx += 1
            if idx < len(tokens):
                result["Extra"]["Pendulum Angle Deg"] = fmt(tokens[idx])
                idx += 1

        if idx < len(tokens) and tokens[idx] == "EncoderL":
            idx += 1
            if idx < len(tokens):
                result["EncoderL"]["Speed"] = fmt(tokens[idx])
                idx += 1
            if idx < len(tokens) and tokens[idx] == "Direction":
                idx += 1
                if idx < len(tokens):
                    result["EncoderL"]["Direction"] = tokens[idx]
                    idx += 1

        if idx < len(tokens) and tokens[idx] == "EncoderR":
            idx += 1
            if idx < len(tokens):
                result["EncoderR"]["Speed"] = fmt(tokens[idx])
                idx += 1
            if idx < len(tokens) and tokens[idx] == "Direction":
                idx += 1
                if idx < len(tokens):
                    result["EncoderR"]["Direction"] = tokens[idx]
                    idx += 1

        if ultrasonic_part:
            parts = ultrasonic_part.split("Ultrasonic")
            for p in parts:
                p = p.strip()
                if p.startswith("Right:"):
                    val = p.replace("Right:", "").replace("cm", "").strip()
                    result["Extra"]["Ultrasonic Right"] = (
                        (fmt(val) + " cm") if val != "---" else val + " cm"
                    )
                elif p.startswith("Left:"):
                    val = p.replace("Left:", "").replace("cm", "").strip()
                    result["Extra"]["Ultrasonic Left"] = (
                        (fmt(val) + " cm") if val != "---" else val + " cm"
                    )
    except Exception:
        pass

    return result


# ==============
# FLASK ROUTES
# ==============
@app.route("/sensor_feed")
def sensor_feed():
    try:
        with open("sensor_data.txt", "r") as f:
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
    import time

    current_time = time.time()
    data = request.get_json()
    x = float(data.get("x", 0.0))
    y = float(data.get("y", 0.0))
    msg = f"Joystick x={x:.2f}, y={y:.2f}"

    if current_time - last_joystick_send >= joystick_throttle_interval:
        last_joystick_send = current_time
        log_to_file("JOYSTICK", msg)

    return jsonify({"message": msg})


@app.route("/get_top_row", methods=["POST"])
def get_top_row():
    log_to_file("ACTION", "Remove Row pressed")
    return jsonify({"message": "Removed top row!"})


@app.route("/start_motor_test", methods=["POST"])
def start_motor_test_route():
    ok = start_motor_test()
    status = "started" if ok else "failed"
    return jsonify({"status": status})


@app.route("/stop_motor_test", methods=["POST"])
def stop_motor_test_route():
    stop_motor_test()
    return jsonify({"status": "stopped"})


@app.route("/start_ml", methods=["POST"])
def start_ml_route():
    ok = start_program("ml")
    return jsonify({"status": "started" if ok else "failed"})


@app.route("/stop_ml", methods=["POST"])
def stop_ml_route():
    stop_program("ml")
    return jsonify({"status": "stopped"})


@app.route("/start_autonav", methods=["POST"])
def start_autonav_route():
    ok = start_program("autonav")
    return jsonify({"status": "started" if ok else "failed"})


@app.route("/stop_autonav", methods=["POST"])
def stop_autonav_route():
    stop_program("autonav")
    return jsonify({"status": "stopped"})


@app.route("/")
def home():
    log_to_file("SYSTEM", "Web interface loaded")
    return """
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <title>Control Hub</title>
    <style>
        :root {
            --bg: #050811;
            --panel-bg: #0c1020;
            --panel-border: #272b3b;
            --accent: #29f0ff;
            --accent-soft: #1cb6c7;
            --text: #e1f6ff;
            --muted: #96a4c0;
            --danger: #ff5560;
            --ok: #8dff8a;
            --warn: #ffdd57;
            --console-bg: #070b16;
            --console-border: #1c2233;
            --grid-gap: 14px;
        }

        * {
            box-sizing: border-box;
        }

        body {
            margin: 0;
            padding: 14px;
            background: radial-gradient(circle at top, #141a33 0, #050811 55%);
            color: var(--text);
            font-family: system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
            font-size: 14px;
        }

        h1 {
            font-size: 1.1rem;
            margin: 0 0 10px 2px;
            color: var(--accent);
            letter-spacing: 0.08em;
            text-transform: uppercase;
            text-shadow: 0 0 8px rgba(41,240,255,0.45);
        }

        .hub-shell {
            max-width: 1240px;
            margin: 0 auto;
        }

        .hub-grid {
            display: grid;
            grid-template-columns: 1.15fr 1.6fr 1.25fr;
            gap: var(--grid-gap);
        }

        .panel {
            background: linear-gradient(145deg, #0b0f1f, #060817);
            border-radius: 11px;
            border: 1px solid rgba(112,145,190,0.25);
            box-shadow:
                0 14px 30px rgba(0,0,0,0.75),
                0 0 0 1px rgba(10,18,40,0.9),
                0 0 42px rgba(15,180,220,0.08);
            padding: 10px 11px 11px;
            position: relative;
        }

        .panel::before {
            content: "";
            position: absolute;
            inset: 0;
            border-radius: inherit;
            background: radial-gradient(circle at top left, rgba(41,240,255,0.08), transparent 60%);
            pointer-events: none;
        }

        .panel-inner {
            position: relative;
            z-index: 1;
        }

        .panel-title-row {
            display: flex;
            justify-content: space-between;
            align-items: baseline;
            margin-bottom: 6px;
        }

        .panel-title {
            font-size: 0.88rem;
            font-weight: 600;
            text-transform: uppercase;
            letter-spacing: 0.08em;
            color: var(--accent-soft);
        }

        .panel-subtitle {
            font-size: 0.7rem;
            color: var(--muted);
            text-transform: uppercase;
            letter-spacing: 0.14em;
        }

        .left-column-stack {
            display: grid;
            gap: var(--grid-gap);
            grid-template-rows: auto 1.05fr;
        }

        .joystick-section {
            display: flex;
            flex-direction: column;
            gap: 8px;
        }

        .joystick-wrapper {
            display: flex;
            gap: 10px;
        }

        .joystick-area-shell {
            flex: 0 0 auto;
        }

        .joystick-area {
            position: relative;
            width: 170px;
            height: 170px;
            border-radius: 50%;
            margin-top: 4px;
            background:
                radial-gradient(circle at 24% 18%, #273454 0, #0b0f1f 40%, #050714 80%);
            border: 1px solid rgba(85,120,174,0.9);
            box-shadow:
                inset 0 0 18px rgba(0,0,0,0.7),
                0 0 18px rgba(16,187,232,0.18);
            touch-action: none;
        }

        .joystick-grid-h,
        .joystick-grid-v {
            position: absolute;
            left: 50%;
            top: 50%;
            width: 86%;
            height: 86%;
            border-radius: 50%;
            border: 1px dashed rgba(104,147,196,0.45);
            transform: translate(-50%, -50%);
        }

        .joystick-grid-v {
            width: 44%;
            height: 102%;
        }

        .joystick-base-ring {
            position: absolute;
            left: 50%;
            top: 50%;
            width: 120px;
            height: 120px;
            margin-left: -60px;
            margin-top: -60px;
            border-radius: 50%;
            background: radial-gradient(circle at 30% 25%, #2df9ff33, transparent 60%);
            box-shadow: inset 0 0 12px rgba(0,0,0,0.7);
        }

        .joystick-knob {
            position: absolute;
            left: 50%;
            top: 50%;
            width: 60px;
            height: 60px;
            margin-left: -30px;
            margin-top: -30px;
            border-radius: 50%;
            background:
                radial-gradient(circle at 25% 20%, #4cffff, #0f6ba0 56%, #03152d 92%);
            box-shadow:
                0 0 16px rgba(41,240,255,0.88),
                0 0 32px rgba(11,150,210,0.8);
            border: 1px solid rgba(200,255,255,0.7);
        }

        .joystick-lights {
            position: absolute;
            inset: 5px;
            border-radius: 50%;
            background:
                radial-gradient(circle at 16% 18%, rgba(255,255,255,0.09), transparent 72%),
                radial-gradient(circle at 80% 80%, rgba(6,207,255,0.2), transparent 72%);
            mix-blend-mode: screen;
            pointer-events: none;
        }

        .joy-readouts-block {
            flex: 1;
            display: flex;
            flex-direction: column;
            justify-content: space-between;
        }

        .joy-readouts {
            display: flex;
            gap: 10px;
            font-family: ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, "Liberation Mono", "Courier New", monospace;
            font-size: 12px;
        }

        .joy-chip {
            border-radius: 8px;
            padding: 3px 10px;
            border: 1px solid rgba(87,140,196,0.65);
            background: radial-gradient(circle at 0 0, #1d3344 0, #07101f 70%);
            box-shadow:
                0 0 12px rgba(0,0,0,0.8),
                inset 0 0 6px rgba(0,0,0,0.7);
            color: #d5f4ff;
        }

        .joy-chip-label {
            color: #8ea0c0;
            margin-right: 6px;
        }

        .joy-chip-value {
            color: #aef8ff;
            font-weight: 600;
        }

        .joy-trash {
            font-size: 11px;
            color: #7f8dad;
            margin-top: 8px;
        }

        .joy-trash span {
            opacity: 0.8;
        }

        .sensor-panel-layout {
            display: grid;
            grid-template-columns: 1.2fr 1.05fr;
            gap: 10px;
            margin-top: 3px;
        }

        .sensor-block {
            border-radius: 9px;
            border: 1px solid rgba(92,132,201,0.5);
            padding: 6px 7px 7px;
            background: radial-gradient(circle at top left, #192338, #080b14);
            box-shadow: inset 0 0 10px rgba(0,0,0,0.9);
        }

        .sensor-header {
            display: flex;
            justify-content: space-between;
            align-items: center;
            margin-bottom: 4px;
        }

        .sensor-header-label {
            font-size: 0.78rem;
            text-transform: uppercase;
            letter-spacing: 0.12em;
            color: var(--muted);
        }

        .sensor-chip {
            padding: 2px 9px;
            border-radius: 999px;
            border: 1px solid rgba(78,140,214,0.8);
            background: linear-gradient(120deg, rgba(12,190,255,0.3), rgba(2,12,28,0.3));
            font-size: 11px;
            color: #e6fbff;
        }

        .kv {
            border-radius: 6px;
            padding: 1px 7px 2px;
            font-family: ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, "Liberation Mono", "Courier New", monospace;
            font-size: 11.5px;
            color: #bae7ff;
            margin-bottom: 2px;
            display: flex;
            justify-content: space-between;
            align-items: baseline;
        }

        .kv-label {
            color: #8291b4;
        }

        .kv strong {
            color: #bafcff;
            font-weight: 600;
        }

        .sensor-status-row {
            display: flex;
            justify-content: space-between;
            align-items: center;
            margin-top: 6px;
            font-size: 11.5px;
        }

        .sensor-status-label {
            color: #8793b0;
            text-transform: uppercase;
            letter-spacing: 0.14em;
            font-size: 10px;
        }

        .sensor-status-pill {
            padding: 2px 9px;
            border-radius: 999px;
            border: 1px solid rgba(80,120,183,0.8);
            background: linear-gradient(120deg, rgba(10,160,220,0.2), rgba(2,8,18,0.6));
            font-size: 11px;
        }

        .sensor-status-pill strong {
            color: #ff5560;
        }

        .calib-countdown {
            font-weight: 600;
            font-size: 12px;
            color: #ffdd57;
            margin-left: 6px;
        }

        .sensor-status-pill strong.sensor-switch.ON {
            color: var(--ok);
        }

        .sensor-status-pill strong.sensor-switch.OFF {
            color: var(--danger);
        }

        .sensor-status-pill strong.sensor-switch.CALIB {
            color: var(--warn);
        }

        .action-panel-grid {
            display: grid;
            grid-template-columns: repeat(3, minmax(0, 1fr));
            gap: 8px;
            margin-top: 2px;
        }

        .btn {
            border-radius: 9px;
            border: 1px solid rgba(90,130,200,0.75);
            background: radial-gradient(circle at top, #1e273d 0, #060815 60%);
            color: var(--text);
            cursor: pointer;
            font-weight: 600;
            text-transform: uppercase;
            font-size: 12px;
            outline: none;
            box-shadow:
                0 2px 4px rgba(0,0,0,0.75),
                0 0 12px rgba(12,30,66,0.9);
            padding: 6px 4px;
            letter-spacing: 0.08em;
        }

        .btn:hover {
            border-color: var(--accent);
            background: radial-gradient(circle at top, #273d54 0, #070a18 60%);
            box-shadow:
                0 0 16px rgba(29,240,255,0.18),
                0 0 0 1px rgba(21,120,170,0.75);
        }

        .btn:active {
            transform: translateY(1px) scale(0.99);
            box-shadow:
                0 1px 3px rgba(0,0,0,0.9),
                0 0 10px rgba(17,150,210,0.65);
        }

        .btn.success {
            color: var(--ok);
            border-color: rgba(80,192,120,0.85);
        }

        .btn.danger {
            color: var(--danger);
            border-color: rgba(190,80,110,0.9);
        }

        .right-column-stack {
            display: grid;
            grid-template-rows: auto auto 1fr;
            gap: var(--grid-gap);
        }

        .robot-status-panel {
            display: grid;
            grid-template-columns: 1.1fr 1fr;
            gap: 10px;
            margin-top: 3px;
        }

        .robot-main {
            border-radius: 9px;
            border: 1px solid rgba(93,132,198,0.75);
            padding: 6px 8px 7px;
            background: radial-gradient(circle at top left, #171f33, #070a15);
            box-shadow: inset 0 0 10px rgba(0,0,0,0.9);
        }

        .robot-secondary {
            border-radius: 9px;
            border: 1px solid rgba(84,126,188,0.65);
            padding: 6px 8px 7px;
            background: radial-gradient(circle at top, #141b31, #050812);
            box-shadow: inset 0 0 8px rgba(0,0,0,0.85);
        }

        .robot-row {
            display: flex;
            justify-content: space-between;
            align-items: baseline;
            margin-bottom: 3px;
            font-size: 12px;
        }

        .robot-row-label {
            color: #8a98b6;
            text-transform: uppercase;
            letter-spacing: 0.14em;
            font-size: 10px;
        }

        .robot-row-value {
            font-family: ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, "Liberation Mono", "Courier New", monospace;
            color: #c0f4ff;
        }

        .robot-btns {
            display: flex;
            gap: 5px;
            margin-top: 5px;
        }

        .robot-btns .btn {
            font-size: 11px;
            padding: 5px 4px;
            letter-spacing: 0.06em;
        }

        .timer {
            font-family: ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, "Liberation Mono", "Courier New", monospace;
            color: #b9f7ff;
            font-size: 12px;
            padding: 4px 8px;
            border-radius: 7px;
            background: radial-gradient(circle at left, #151f38, #050812);
            border: 1px solid rgba(100,139,205,0.75);
            box-shadow:
                0 0 18px rgba(0,0,0,0.75),
                0 0 0 1px rgba(11,28,52,0.9);
            display: inline-block;
        }

        .console {
            height: 220px;
            background: radial-gradient(circle at top, #0d1628, #03050c);
            border: 1px solid rgba(82,120,190,0.9);
            border-radius: 9px;
            padding: 7px 8px;
            overflow: auto;
            font-family: ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, "Liberation Mono", "Courier New", monospace;
            font-size: 12px;
            color: #b2f3ff;
            box-shadow:
                inset 0 0 14px rgba(0,0,0,0.95),
                0 0 22px rgba(13,180,230,0.18);
        }

        .console .line {
            margin-bottom: 2px;
        }

        .tag {
            display: inline-block;
            padding: 0 7px;
            border-radius: 999px;
            margin-right: 6px;
            border: 1px solid rgba(80,150,215,0.8);
            color: #baf9ff;
            background: linear-gradient(135deg, #162331, #050812);
            font-size: 11px;
        }

        .console-time {
            color: #7386ad;
            margin-right: 6px;
        }

        @media (max-width: 1060px) {
            .hub-grid {
                grid-template-columns: 1.1fr 1.4fr;
                grid-template-rows: auto auto;
            }

            .right-column-stack {
                grid-row: 2;
                grid-column: 1 / span 2;
            }
        }

        @media (max-width: 840px) {
            .hub-grid {
                grid-template-columns: 1fr;
            }

            .right-column-stack {
                grid-row: auto;
                grid-column: auto;
            }
        }

        @media (max-width: 540px) {
            .joystick-wrapper {
                flex-direction: column;
            }

            .sensor-panel-layout {
                grid-template-columns: 1fr;
            }

            .robot-status-panel {
                grid-template-columns: 1fr;
            }
        }
    </style>
</head>
<body>
<div class="hub-shell">
    <h1>Control Hub</h1>
    <div class="hub-grid">
        <div class="panel">
            <div class="panel-inner">
                <div class="panel-title-row">
                    <div class="panel-title">Joystick &amp; Sensors</div>
                    <div class="panel-subtitle">Manual override / live telemetry</div>
                </div>

                <div class="left-column-stack">
                    <div class="joystick-section">
                        <div class="joystick-wrapper">
                            <div class="joystick-area-shell">
                                <div id="joystick-area" class="joystick-area">
                                    <div class="joystick-grid-h"></div>
                                    <div class="joystick-grid-v"></div>
                                    <div class="joystick-base-ring"></div>
                                    <div class="joystick-knob" id="joystick-knob"></div>
                                    <div class="joystick-lights"></div>
                                </div>
                            </div>

                            <div class="joy-readouts-block">
                                <div class="joy-readouts">
                                    <div class="joy-chip">
                                        <span class="joy-chip-label">X</span>
                                        <span class="joy-chip-value" id="joy-x">0.0</span>
                                    </div>
                                    <div class="joy-chip">
                                        <span class="joy-chip-label">Y</span>
                                        <span class="joy-chip-value" id="joy-y">0.0</span>
                                    </div>
                                </div>
                                <div class="joy-trash">
                                    <span>Drag inside the pad to send normalized joystick commands.</span>
                                </div>
                            </div>
                        </div>
                    </div>

                    <div>
                        <div class="sensor-panel-layout">
                            <div class="sensor-block">
                                <div class="sensor-header">
                                    <div class="sensor-header-label">IMU1 attitude</div>
                                    <div class="sensor-chip">
                                        <span id="imu-time">Time</span>
                                        <span class="calib-countdown" id="calib-status">Auto</span>
                                    </div>
                                </div>

                                <div class="kv">
                                    <span class="kv-label">FB Tilt</span>
                                    <strong id="imu1-fb"></strong>
                                </div>
                                <div class="kv">
                                    <span class="kv-label">SS Tilt</span>
                                    <strong id="imu1-ss"></strong>
                                </div>
                                <div class="kv">
                                    <span class="kv-label">Yaw</span>
                                    <strong id="imu1-yaw"></strong>
                                </div>
                                <div class="kv">
                                    <span class="kv-label">Pitch Rate</span>
                                    <strong id="imu1-pitch"></strong>
                                </div>
                                <div class="kv">
                                    <span class="kv-label">Roll Rate</span>
                                    <strong id="imu1-roll"></strong>
                                </div>
                                <div class="kv">
                                    <span class="kv-label">Rot Vel</span>
                                    <strong id="imu1-rot"></strong>
                                </div>
                            </div>

                            <div class="sensor-block">
                                <div class="sensor-header">
                                    <div class="sensor-header-label">IMU2 + encoders</div>
                                    <div class="sensor-chip">Derived state</div>
                                </div>

                                <div class="kv">
                                    <span class="kv-label">IMU2 FB Tilt</span>
                                    <strong id="imu2-fb"></strong>
                                </div>
                                <div class="kv">
                                    <span class="kv-label">IMU2 SS Tilt</span>
                                    <strong id="imu2-ss"></strong>
                                </div>
                                <div class="kv">
                                    <span class="kv-label">IMU2 Yaw</span>
                                    <strong id="imu2-yaw"></strong>
                                </div>
                                <div class="kv">
                                    <span class="kv-label">IMU2 Pitch Rate</span>
                                    <strong id="imu2-pitch"></strong>
                                </div>
                                <div class="kv">
                                    <span class="kv-label">IMU2 Roll Rate</span>
                                    <strong id="imu2-roll"></strong>
                                </div>
                                <div class="kv">
                                    <span class="kv-label">IMU2 Rot Vel</span>
                                    <strong id="imu2-rot"></strong>
                                </div>
                                <div class="kv">
                                    <span class="kv-label">IMU1 Linear Vel</span>
                                    <strong id="imu1linear-lv"></strong>
                                </div>
                                <div class="kv">
                                    <span class="kv-label">IMU1 X vel</span>
                                    <strong id="imu1linear-xv"></strong>
                                </div>
                                <div class="kv">
                                    <span class="kv-label">IMU1 Y vel</span>
                                    <strong id="imu1linear-yv"></strong>
                                </div>
                                <div class="kv">
                                    <span class="kv-label">Robot Yaw Rate</span>
                                    <strong id="robot-yaw-rate"></strong>
                                </div>
                                <div class="kv">
                                    <span class="kv-label">Pendulum Ang Vel</span>
                                    <strong id="pend-angvel"></strong>
                                </div>
                                <div class="kv">
                                    <span class="kv-label">Pendulum Angle</span>
                                    <strong id="pend-angle"></strong>
                                </div>
                                <div class="kv">
                                    <span class="kv-label">Pend Angle (deg)</span>
                                    <strong id="pend-angle-deg"></strong>
                                </div>
                                <div class="kv">
                                    <span class="kv-label">Enc L Speed</span>
                                    <strong id="enc-l-val"></strong>
                                </div>
                                <div class="kv">
                                    <span class="kv-label">Enc L Dir</span>
                                    <strong id="enc-l-status"></strong>
                                </div>
                                <div class="kv">
                                    <span class="kv-label">Enc R Speed</span>
                                    <strong id="enc-r-val"></strong>
                                </div>
                                <div class="kv">
                                    <span class="kv-label">Enc R Dir</span>
                                    <strong id="enc-r-status"></strong>
                                </div>
                                <div class="kv">
                                    <span class="kv-label">Ultrasonic Right</span>
                                    <strong id="ultra-right"></strong>
                                </div>
                                <div class="kv">
                                    <span class="kv-label">Ultrasonic Left</span>
                                    <strong id="ultra-left"></strong>
                                </div>
                            </div>
                        </div>

                        <div class="sensor-status-row">
                            <div class="sensor-status-label">Sensor Stack</div>
                            <div class="sensor-status-pill">
                                <span>Sensors:</span>
                                <strong id="sensor-switch" class="sensor-switch OFF">OFF</strong>
                            </div>
                        </div>
                    </div>
                </div>
            </div>
        </div>

        <div class="panel">
            <div class="panel-inner">
                <div class="panel-title-row">
                    <div class="panel-title">Action Panel</div>
                    <div class="panel-subtitle">Pipeline &amp; behaviors</div>
                </div>

                <div class="action-panel-grid">
                    <button class="btn danger" onclick="getTopRow()">Remove Row</button>
                    <button class="btn success" onclick="toggleSensor(true)">Sensors ON</button>
                    <button class="btn danger" onclick="toggleSensor(false)">Sensors OFF</button>

                    <button class="btn" onclick="startMotorTest()">Start Motor</button>
                    <button class="btn" onclick="stopMotorTest()">Stop Motor</button>

                    <button class="btn success" onclick="startML()">ML Start</button>
                    <button class="btn danger" onclick="stopML()">ML Stop</button>

                    <button class="btn success" onclick="startAutonav()">AutoNav Start</button>
                    <button class="btn danger" onclick="stopAutonav()">AutoNav Stop</button>
                    <span></span>
                </div>
            </div>
        </div>

        <div class="panel right-column-stack">
            <div class="panel-inner">
                <div class="panel-title-row">
                    <div class="panel-title">Robot Status</div>
                    <div class="panel-subtitle">Drive / power</div>
                </div>

                <div class="robot-status-panel">
                    <div class="robot-main">
                        <div class="robot-row">
                            <span class="robot-row-label">Speed</span>
                            <span class="robot-row-value" id="robot-speed">0</span>
                        </div>
                        <div class="robot-btns">
                            <button class="btn" onclick="setSpeed('slow')">Slow</button>
                            <button class="btn" onclick="setSpeed('med')">Medium</button>
                            <button class="btn" onclick="setSpeed('fast')">Fast</button>
                            <button class="btn" onclick="brakeRobot()">Brake</button>
                        </div>
                    </div>

                    <div class="robot-secondary">
                        <div class="robot-row">
                            <span class="robot-row-label">Braking</span>
                            <span class="robot-row-value" id="robot-brake">OFF</span>
                        </div>
                        <div class="robot-row">
                            <span class="robot-row-label">Battery</span>
                            <span class="robot-row-value" id="robot-battery">0</span>
                        </div>
                        <div class="robot-row">
                            <span class="robot-row-label">Status</span>
                            <span class="robot-row-value" id="robot-switch">OFF</span>
                        </div>
                    </div>
                </div>
            </div>

            <div class="panel-inner">
                <div class="timer" id="timer">Task time: 0.00s</div>
            </div>

            <div class="panel-inner">
                <div class="panel-title-row">
                    <div class="panel-title">Event Console</div>
                    <div class="panel-subtitle">Recent events</div>
                </div>
                <div class="console" id="console"></div>
            </div>
        </div>
    </div>
</div>

<script>
    function getval(el, val) {
        document.getElementById(el).textContent = val;
    }

    function updateSensorFeed() {
        fetch('/sensor_feed')
            .then(r => r.json())
            .then(data => {
                getval('imu-time', data.IMU1["Time"]);
                getval('imu1-fb', data.IMU1["Forward/backwards Tilt"]);
                getval('imu1-ss', data.IMU1["Side-to-Side Tilt"]);
                getval('imu1-yaw', data.IMU1["Yaw"]);
                getval('imu1-pitch', data.IMU1["Pitch Rate"]);
                getval('imu1-roll', data.IMU1["Roll Rate"]);
                getval('imu1-rot', data.IMU1["Rotational Velocity"]);

                getval('imu2-fb', data.IMU2["Forward/backwards Tilt"]);
                getval('imu2-ss', data.IMU2["Side-to-Side Tilt"]);
                getval('imu2-yaw', data.IMU2["Yaw"]);
                getval('imu2-pitch', data.IMU2["Pitch Rate"]);
                getval('imu2-roll', data.IMU2["Roll Rate"]);
                getval('imu2-rot', data.IMU2["Rotational Velocity"]);

                getval('imu1linear-lv', data.IMU1Linear["Linear Velocity"]);
                getval('imu1linear-xv', data.IMU1Linear["X velocity"]);
                getval('imu1linear-yv', data.IMU1Linear["Y velocity"]);

                getval('robot-yaw-rate', data.Extra["Robot Yaw Rate"]);
                getval('pend-angvel', data.Extra["Pendulum Angular Velocity"]);
                getval('pend-angle', data.Extra["Pendulum Angle"]);
                getval('pend-angle-deg', data.Extra["Pendulum Angle Deg"]);

                getval('enc-l-val', data.EncoderL["Speed"]);
                getval('enc-l-status', data.EncoderL["Direction"]);
                getval('enc-r-val', data.EncoderR["Speed"]);
                getval('enc-r-status', data.EncoderR["Direction"]);

                getval('ultra-right', data.Extra["Ultrasonic Right"]);
                getval('ultra-left', data.Extra["Ultrasonic Left"]);
            })
            .catch(err => {
                console.error('Sensor feed error', err);
            });
    }

    setInterval(updateSensorFeed, 250);
    updateSensorFeed();

    let calibTimer = null;

    function toggleSensor(on) {
        let logMsg = on ? "Sensors turned ON" : "Sensors turned OFF";
        fetch(on ? '/sensor_on' : '/sensor_off', { method: 'POST' })
            .then(r => r.json())
            .then(data => {
                let s = document.getElementById('sensor-switch');
                if (on && data.status === "ON") {
                    s.textContent = "ON";
                    s.className = "sensor-switch CALIB";
                    calibrateSensorCountdown(10, function() {
                        s.textContent = "ON";
                        s.className = "sensor-switch ON";
                    });
                    logLine("SENSOR", logMsg);
                } else if (!on && data.status === "OFF") {
                    s.textContent = "OFF";
                    s.className = "sensor-switch OFF";
                    document.getElementById('calib-status').textContent = "Auto";
                    if (calibTimer) clearInterval(calibTimer);
                    logLine("SENSOR", logMsg);
                } else {
                    logLine("SENSOR", "Error toggling sensors");
                }
            });
    }

    function calibrateSensorCountdown(seconds, doneCb) {
        let counter = seconds;
        document.getElementById('calib-status').textContent = "Calibrating " + counter;

        let s = document.getElementById('sensor-switch');
        s.textContent = "ON";
        s.className = "sensor-switch CALIB";

        if (calibTimer) clearInterval(calibTimer);
        calibTimer = setInterval(function() {
            counter--;
            document.getElementById('calib-status').textContent =
                counter > 0 ? "Calibrating " + counter : "Done";
            if (counter <= 0) {
                clearInterval(calibTimer);
                if (doneCb) doneCb();
            }
        }, 1000);
    }

    function setSpeed(mode) {
        let val = mode === "slow" ? 25 : (mode === "med" ? 42 : 88);
        document.getElementById('robot-speed').textContent = val;
        logLine("SPEED", "Speed set to " + mode.toUpperCase() + " (" + val + ")");
    }

    function brakeRobot() {
        document.getElementById('robot-brake').textContent = "ON";
        logLine("BRAKE", "Robot Braking activated");
        setTimeout(function() {
            document.getElementById('robot-brake').textContent = "OFF";
        }, 900);
    }

    function getTopRow() {
        logLine("ACTION", "Remove Row button pressed");
        fetch('/get_top_row', { method: 'POST' })
            .then(r => r.json())
            .then(data => { });
    }

    function startMotorTest() {
        logLine("MOTOR", "Starting Motor Test (clearing numbers.txt)");
        fetch('/start_motor_test', { method: 'POST' })
            .then(r => r.json())
            .then(data => {
                if (data.status === "started") {
                    logLine("MOTOR", "Motor Test started successfully");
                } else {
                    logLine("MOTOR", "Motor Test failed to start");
                }
            })
            .catch(err => {
                logLine("ERR", "Motor Test start error: " + err);
            });
    }

    function stopMotorTest() {
        logLine("MOTOR", "Stopping Motor Test");
        fetch('/stop_motor_test', { method: 'POST' })
            .then(r => r.json())
            .then(data => {
                logLine("MOTOR", "Motor Test stopped");
            })
            .catch(err => {
                logLine("ERR", "Motor Test stop error: " + err);
            });
    }

    function startML() {
        logLine("ML", "Starting ML");
        fetch('/start_ml', { method: 'POST' })
            .then(r => r.json())
            .then(data => {
                if (data.status === "started") {
                    logLine("ML", "ML started");
                } else {
                    logLine("ML", "ML failed to start");
                }
            })
            .catch(err => {
                logLine("ERR", "ML start error: " + err);
            });
    }

    function stopML() {
        logLine("ML", "Stopping ML");
        fetch('/stop_ml', { method: 'POST' })
            .then(r => r.json())
            .then(data => {
                logLine("ML", "ML stopped");
            })
            .catch(err => {
                logLine("ERR", "ML stop error: " + err);
            });
    }

    function startAutonav() {
        logLine("AUTONAV", "Starting AutoNav");
        fetch('/start_autonav', { method: 'POST' })
            .then(r => r.json())
            .then(data => {
                if (data.status === "started") {
                    logLine("AUTONAV", "AutoNav started");
                } else {
                    logLine("AUTONAV", "AutoNav failed to start");
                }
            })
            .catch(err => {
                logLine("ERR", "AutoNav start error: " + err);
            });
    }

    function stopAutonav() {
        logLine("AUTONAV", "Stopping AutoNav");
        fetch('/stop_autonav', { method: 'POST' })
            .then(r => r.json())
            .then(data => {
                logLine("AUTONAV", "AutoNav stopped");
            })
            .catch(err => {
                logLine("ERR", "AutoNav stop error: " + err);
            });
    }

    function logLine(tag, text) {
        const c = document.getElementById('console');
        if (!c) return;
        const div = document.createElement('div');
        div.className = 'line';
        const timeSpan = document.createElement('span');
        timeSpan.className = 'console-time';
        timeSpan.textContent = new Date().toLocaleTimeString();
        const t = document.createElement('span');
        t.className = 'tag';
        t.textContent = tag;
        const span = document.createElement('span');
        span.textContent = text;
        div.appendChild(timeSpan);
        div.appendChild(t);
        div.appendChild(span);
        c.appendChild(div);
        c.scrollTop = c.scrollHeight;
    }

    const area = document.getElementById('joystick-area');
    const knob = document.getElementById('joystick-knob');
    let joyCenter = { x: area.clientWidth / 2, y: area.clientHeight / 2 };
    let joyRadius = 70;
    let joyActive = false;
    let lastJoystickLog = 0;
    const joystickThrottleMs = 250;

    function setKnob(x, y) {
        knob.style.left = x + 'px';
        knob.style.top = y + 'px';
    }

    setKnob(joyCenter.x - knob.clientWidth / 2, joyCenter.y - knob.clientHeight / 2);

    function sendJoystick(x, y) {
        const now = Date.now();
        const shouldLog = now - lastJoystickLog > joystickThrottleMs;
        if (shouldLog) lastJoystickLog = now;

        fetch('/direction_ajax', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ x: x, y: y })
        })
        .then(r => r.json())
        .then(data => {
            if (data && data.message && shouldLog) {
                logLine("JOY", data.message);
            }
        })
        .catch(err => {
            if (shouldLog) logLine("ERR", "Joystick send failed: " + err);
        });
    }

    function updateJoystick(clientX, clientY) {
        const rect = area.getBoundingClientRect();
        const cx = clientX - rect.left;
        const cy = clientY - rect.top;
        let dx = cx - joyCenter.x;
        let dy = cy - joyCenter.y;

        const dist = Math.sqrt(dx * dx + dy * dy);
        if (dist > joyRadius) {
            const scale = joyRadius / dist;
            dx *= scale;
            dy *= scale;
        }

        const knobX = joyCenter.x + dx - knob.clientWidth / 2;
        const knobY = joyCenter.y + dy - knob.clientHeight / 2;
        setKnob(knobX, knobY);

        let normX = dx / joyRadius;
        let normY = -dy / joyRadius;

        if (normY < -1) normY = -1;
        if (normY > 1) normY = 1;
        if (normX < -1) normX = -1;
        if (normX > 1.2) normX = 1.2;

        document.getElementById('joy-x').textContent = normX.toFixed(1);
        document.getElementById('joy-y').textContent = normY.toFixed(1);

        sendJoystick(normX, normY);
    }

    function resetJoystick() {
        setKnob(joyCenter.x - knob.clientWidth / 2, joyCenter.y - knob.clientHeight / 2);
        document.getElementById('joy-x').textContent = "0.0";
        document.getElementById('joy-y').textContent = "0.0";
        const now = Date.now();
        if (now - lastJoystickLog > joystickThrottleMs) {
            lastJoystickLog = now;
            sendJoystick(0, 0);
        }
    }

    area.addEventListener('mousedown', function(e) {
        joyActive = true;
        updateJoystick(e.clientX, e.clientY);
    });

    window.addEventListener('mousemove', function(e) {
        if (joyActive) updateJoystick(e.clientX, e.clientY);
    });

    window.addEventListener('mouseup', function() {
        if (joyActive) {
            joyActive = false;
            resetJoystick();
        }
    });

    area.addEventListener('touchstart', function(e) {
        e.preventDefault();
        joyActive = true;
        const t = e.touches[0];
        updateJoystick(t.clientX, t.clientY);
    }, { passive: false });

    area.addEventListener('touchmove', function(e) {
        e.preventDefault();
        if (!joyActive) return;
        const t = e.touches[0];
        updateJoystick(t.clientX, t.clientY);
    }, { passive: false });

    area.addEventListener('touchend', function(e) {
        e.preventDefault();
        if (joyActive) {
            joyActive = false;
            resetJoystick();
        }
    }, { passive: false });
</script>
</body>
</html>
"""


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000, debug=True)
