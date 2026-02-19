from flask import Flask, jsonify, request
import subprocess
import os
import signal
from datetime import datetime

app = Flask(__name__)

# ================================
# PROGRAM DEFINITIONS (VARIABLES)
# ================================
IMU_SCRIPT = "sensor.py"
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
            preexec_fn=os.setsid
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
    """
    Parse a line like:

    18:41:46, IMU1, Forward/backwards, 0.0000, Side-to-Side, 0.0000, Yaw, 0.0000, ...
    ..., Robot Yaw Rate, 0.0000, Pendulum Angular Velocity, 0.0000, Pendulum Angle, 0.0000,
    Pendulum Angle (deg), 0.00, EncoderL, 0.0000, Direction, stopped, EncoderR, 0.0000, Direction, stopped,
    Ultrasonic Right, 36.7 cm, Ultrasonic Left, 10 cm

    and return a structured dict with all fields, numeric ones formatted to 2 significant figures.
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

        # Time
        if idx < len(tokens) and ":" in tokens[idx]:
            result["IMU1"]["Time"] = tokens[idx]
            idx += 1

        # IMU1
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

        # IMU2
        if idx < len(tokens) and tokens[idx] == "IMU2":
            idx += 1
            imu2_map = [
                ("Tilt", "Forward/backwards Tilt"),
                ("Side-to-Side tilt", "Side-to-Side Tilt"),
                ("Yaw", "Yaw"),
                ("Pitch Rate", "Pitch Rate"),
                ("Roll Rate", "Roll Rate"),
                ("Rotational Velocity", "Rotational Velocity"),
            ]
            for label, key in imu2_map:
                if idx + 1 < len(tokens) and tokens[idx] == label:
                    result["IMU2"][key] = num2sig(tokens[idx + 1])
                    idx += 2

        # IMU1 Linear Velocity
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

        # Robot Yaw Rate
        if idx < len(tokens) and tokens[idx] == "Robot Yaw Rate":
            idx += 1
            if idx < len(tokens):
                result["Robot"]["Yaw Rate"] = num2sig(tokens[idx])
                idx += 1

        # Pendulum values
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

        # EncoderL
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

        # EncoderR
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

        # Ultrasonic Right, val cm, Ultrasonic Left, val cm
        if idx < len(tokens) and tokens[idx] == "Ultrasonic Right":
            idx += 1
            if idx < len(tokens):
                val = tokens[idx]
                val_num = val.replace("cm", "").strip()
                result["Ultrasonic"]["Right"] = num2sig(val_num)
                idx += 1

        if idx < len(tokens) and tokens[idx] == "Ultrasonic Left":
            idx += 1
            if idx < len(tokens):
                val = tokens[idx]
                val_num = val.replace("cm", "").strip()
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
<html>
<head>
    <meta charset="utf-8">
    <title>Control Hub</title>
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
            margin: 0;
            padding: 0;
            background: var(--bg);
            color: var(--text);
            font-family: system-ui, sans-serif;
            height: 100vh;
            display: flex;
            flex-direction: column;
        }

        .top-bar {
            background: #111922;
            padding: 8px 12px;
            box-shadow: 0 2px 10px #000a;
            display: flex;
            flex-direction: column;
            gap: 4px;
        }

        .top-row, .bottom-row {
            display: flex;
            align-items: center;
            justify-content: space-between;
            gap: 8px;
        }

        .top-row-left, .top-row-right,
        .bottom-row-left, .bottom-row-right {
            display: flex;
            align-items: center;
            gap: 8px;
        }

        .top-title {
            font-weight: 600;
            letter-spacing: 0.03em;
            color: var(--accent);
            text-shadow: 0 0 8px #29f0ff55;
        }

        .top-btn {
            background: #101820;
            border-radius: 7px;
            border: 1px solid #18303c;
            padding: 4px 8px;
            font-size: 12px;
            color: var(--text);
            cursor: pointer;
            text-transform: uppercase;
            font-weight: 600;
        }

        .top-btn:hover {
            border-color: var(--accent);
            background: #11262c;
        }

        .top-btn.danger {
            color: var(--bad);
            border-color: #a8444e;
        }

        .top-btn.success {
            color: var(--ok);
            border-color: #229953;
        }

        .top-btn.neutral {
            color: var(--warn);
            border-color: #bfa23a;
        }

        .top-status {
            font-size: 12px;
            font-family: ui-monospace;
        }

        .top-status span strong.ON {
            color: var(--ok);
        }

        .top-status span strong.OFF {
            color: var(--bad);
        }

        .top-status span strong.CALIB {
            color: var(--warn);
        }

        .main-layout {
            flex: 1;
            padding: 10px;
            display: flex;
            gap: var(--grid-gap);
            box-sizing: border-box;
        }

        .panel {
            background: var(--panel);
            border-radius: 9px;
            padding: 11px;
            box-shadow: 0 2px 12px #0008;
            display: flex;
            flex-direction: column;
            min-width: 0;
        }

        .col-left, .col-center, .col-right {
            flex: 1;
            min-width: 0;
            display: flex;
            flex-direction: column;
            gap: var(--grid-gap);
        }

        .panel-title {
            font-size: 0.9rem;
            margin-bottom: 6px;
            color: var(--accent);
        }

        .joystick-wrapper {
            display: flex;
            flex-direction: column;
            align-items: center;
            gap: 6px;
        }

        .joystick-area {
            position: relative;
            width: 180px;
            height: 180px;
            border-radius: 50%;
            margin-top: 4px;
            background: radial-gradient(circle at 30% 30%, #1e2b36, #05090e);
            border: 2px solid #18303c;
            box-shadow: 0 0 10px #000a;
            touch-action: none;
        }

        .joystick-base {
            position: absolute;
            left: 50%;
            top: 50%;
            width: 140px;
            height: 140px;
            margin-left: -70px;
            margin-top: -70px;
            border-radius: 50%;
            border: 1px dashed #2a4f60;
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
            background: radial-gradient(circle at 30% 30%, #3cf0ff, #0e4f5b);
            box-shadow: 0 0 14px #29f0ffaa;
        }

        .joy-readouts {
            display: flex;
            gap: 8px;
            font-family: ui-monospace;
            font-size: 12px;
        }

        .joy-readouts span strong {
            color: var(--ok);
        }

        .console {
            flex: 1 1 auto;
            background: #061016;
            border: 1px solid #17303b;
            border-radius: 7px;
            padding: 8px 7px;
            overflow: auto;
            font-family: ui-monospace;
            font-size: 13px;
            color: #b2f3ff;
        }

        .console .line {
            margin-bottom: 2px;
        }

        .tag {
            display: inline-block;
            padding: 0 7px;
            border-radius: 3px;
            margin-right: 5px;
            border: 1px solid #234a54;
            color: #baf9ff;
            background: #141d20;
            font-size: 12px;
        }

        .kv {
            border: 1px dashed #18303c;
            border-radius: 7px;
            padding: 2px 7px;
            font-family: ui-monospace;
            font-size: 12px;
            color: #aee9ef;
            margin-bottom: 2px;
            min-width: 0;
            line-height: 1.3;
            display: flex;
            justify-content: space-between;
        }

        .kv strong {
            color: var(--ok);
            font-weight: 600;
            font-size: 13px;
        }

        .sensor-section-title {
            margin-top: 4px;
            margin-bottom: 2px;
            text-decoration: underline;
        }

        .sensor-avg {
            font-size: 11px;
            color: #c2f2ff;
            margin-left: 6px;
        }

        .sensor-status-line {
            margin-top: 4px;
            font-size: 12px;
        }

        .calib-countdown {
            font-weight: 700;
            font-size: 13px;
            margin-left: 0.75em;
        }

        @media (max-width: 960px) {
            .main-layout {
                flex-direction: column;
            }
        }
    </style>
</head>
<body>
<div class="top-bar">
    <div class="top-row">
        <div class="top-row-left">
            <div class="top-title">Control Hub</div>
        </div>
        <div class="top-row-right">
            <button class="top-btn success" onclick="startMotorTest()">Motors ON</button>
            <button class="top-btn success" onclick="toggleSensor(true)">Sensors ON</button>
            <button class="top-btn neutral" onclick="startML()">ML ON</button>
            <button class="top-btn neutral" onclick="startAutonav()">Auto Nav ON</button>
        </div>
    </div>
    <div class="bottom-row">
        <div class="bottom-row-left">
            <span class="top-status">
                <span>Sens <strong id="sensor-switch" class="OFF">OFF</strong></span>
                &nbsp;&nbsp;
                <span>Calib <span class="calib-countdown" id="calib-status">Auto</span></span>
            </span>
        </div>
        <div class="bottom-row-right">
            <button class="top-btn danger" onclick="stopMotorTest()">Motors OFF</button>
            <button class="top-btn danger" onclick="toggleSensor(false)">Sensors OFF</button>
            <button class="top-btn danger" onclick="stopML()">ML OFF</button>
            <button class="top-btn danger" onclick="stopAutonav()">Auto Nav OFF</button>
        </div>
    </div>
</div>

<div class="main-layout">
    <div class="col-left">
        <div class="panel">
            <div class="panel-title">Joystick Input</div>
            <div class="joystick-wrapper">
                <div id="joystick-area" class="joystick-area">
                    <div id="joystick-base" class="joystick-base"></div>
                    <div id="joystick-knob" class="joystick-knob"></div>
                </div>
                <div class="joy-readouts">
                    <span>X: <strong id="joy-x">0.00</strong></span>
                    <span>Y: <strong id="joy-y">0.00</strong></span>
                </div>
            </div>
        </div>
    </div>

    <div class="col-center">
        <div class="panel console-panel">
            <div class="panel-title">Event Console</div>
            <div class="console" id="console"></div>
        </div>
    </div>

    <div class="col-right">
        <div class="panel">
            <div class="panel-title">Sensor Feed with 3s Averages</div>
            <div id="sensor-feed-panel">
                <div class="kv">
                    <span><strong id="imu-time">Time</strong></span>
                    <span class="sensor-avg" id="imu-time-avg"></span>
                </div>

                <div class="sensor-section-title">IMU1</div>
                <div class="kv">
                    <span>FB Tilt</span>
                    <span><strong id="imu1-fb"></strong><span class="sensor-avg" id="imu1-fb-avg"></span></span>
                </div>
                <div class="kv">
                    <span>SS Tilt</span>
                    <span><strong id="imu1-ss"></strong><span class="sensor-avg" id="imu1-ss-avg"></span></span>
                </div>
                <div class="kv">
                    <span>Yaw</span>
                    <span><strong id="imu1-yaw"></strong><span class="sensor-avg" id="imu1-yaw-avg"></span></span>
                </div>
                <div class="kv">
                    <span>Pitch Rate</span>
                    <span><strong id="imu1-pitch"></strong><span class="sensor-avg" id="imu1-pitch-avg"></span></span>
                </div>
                <div class="kv">
                    <span>Roll Rate</span>
                    <span><strong id="imu1-roll"></strong><span class="sensor-avg" id="imu1-roll-avg"></span></span>
                </div>
                <div class="kv">
                    <span>Rot Vel</span>
                    <span><strong id="imu1-rot"></strong><span class="sensor-avg" id="imu1-rot-avg"></span></span>
                </div>

                <div class="sensor-section-title">IMU2</div>
                <div class="kv">
                    <span>FB Tilt</span>
                    <span><strong id="imu2-fb"></strong><span class="sensor-avg" id="imu2-fb-avg"></span></span>
                </div>
                <div class="kv">
                    <span>SS Tilt</span>
                    <span><strong id="imu2-ss"></strong><span class="sensor-avg" id="imu2-ss-avg"></span></span>
                </div>
                <div class="kv">
                    <span>Yaw</span>
                    <span><strong id="imu2-yaw"></strong><span class="sensor-avg" id="imu2-yaw-avg"></span></span>
                </div>
                <div class="kv">
                    <span>Pitch Rate</span>
                    <span><strong id="imu2-pitch"></strong><span class="sensor-avg" id="imu2-pitch-avg"></span></span>
                </div>
                <div class="kv">
                    <span>Roll Rate</span>
                    <span><strong id="imu2-roll"></strong><span class="sensor-avg" id="imu2-roll-avg"></span></span>
                </div>
                <div class="kv">
                    <span>Rot Vel</span>
                    <span><strong id="imu2-rot"></strong><span class="sensor-avg" id="imu2-rot-avg"></span></span>
                </div>

                <div class="sensor-section-title">IMU1 Linear</div>
                <div class="kv">
                    <span>Linear Vel</span>
                    <span><strong id="imu1linear-lv"></strong><span class="sensor-avg" id="imu1linear-lv-avg"></span></span>
                </div>
                <div class="kv">
                    <span>X velocity</span>
                    <span><strong id="imu1linear-xv"></strong><span class="sensor-avg" id="imu1linear-xv-avg"></span></span>
                </div>
                <div class="kv">
                    <span>Y velocity</span>
                    <span><strong id="imu1linear-yv"></strong><span class="sensor-avg" id="imu1linear-yv-avg"></span></span>
                </div>

                <div class="sensor-section-title">EncoderL</div>
                <div class="kv">
                    <span>Speed</span>
                    <span><strong id="enc-l-val"></strong><span class="sensor-avg" id="enc-l-val-avg"></span></span>
                </div>
                <div class="kv">
                    <span>Direction</span>
                    <span><strong id="enc-l-status"></strong><span class="sensor-avg" id="enc-l-status-avg"></span></span>
                </div>

                <div class="sensor-section-title">EncoderR</div>
                <div class="kv">
                    <span>Speed</span>
                    <span><strong id="enc-r-val"></strong><span class="sensor-avg" id="enc-r-val-avg"></span></span>
                </div>
                <div class="kv">
                    <span>Direction</span>
                    <span><strong id="enc-r-status"></strong><span class="sensor-avg" id="enc-r-status-avg"></span></span>
                </div>

                <div class="sensor-section-title">Robot / Pendulum</div>
                <div class="kv">
                    <span>Robot Yaw Rate</span>
                    <span><strong id="robot-yaw-rate"></strong><span class="sensor-avg" id="robot-yaw-rate-avg"></span></span>
                </div>
                <div class="kv">
                    <span>Pendulum Ang Vel</span>
                    <span><strong id="pend-ang-vel"></strong><span class="sensor-avg" id="pend-ang-vel-avg"></span></span>
                </div>
                <div class="kv">
                    <span>Pendulum Angle</span>
                    <span><strong id="pend-angle"></strong><span class="sensor-avg" id="pend-angle-avg"></span></span>
                </div>
                <div class="kv">
                    <span>Pend Angle (deg)</span>
                    <span><strong id="pend-angle-deg"></strong><span class="sensor-avg" id="pend-angle-deg-avg"></span></span>
                </div>

                <div class="sensor-section-title">Ultrasonic</div>
                <div class="kv">
                    <span>Right (cm)</span>
                    <span><strong id="ultra-right"></strong><span class="sensor-avg" id="ultra-right-avg"></span></span>
                </div>
                <div class="kv">
                    <span>Left (cm)</span>
                    <span><strong id="ultra-left"></strong><span class="sensor-avg" id="ultra-left-avg"></span></span>
                </div>
            </div>

            <div class="sensor-status-line">
                Sensor state shown in top bar
            </div>
        </div>
    </div>
</div>

<script>
    function getval(el, val) {
        document.getElementById(el).textContent = val;
    }

    const SENSOR_HISTORY_WINDOW = 3.0;
    const SENSOR_POLL_MS = 250;
    const SENSOR_MAX_SAMPLES = Math.ceil(SENSOR_HISTORY_WINDOW * 1000 / SENSOR_POLL_MS);

    const sensorNumericKeys = {
        "imu1-fb": ["IMU1", "Forward/backwards Tilt"],
        "imu1-ss": ["IMU1", "Side-to-Side Tilt"],
        "imu1-yaw": ["IMU1", "Yaw"],
        "imu1-pitch": ["IMU1", "Pitch Rate"],
        "imu1-roll": ["IMU1", "Roll Rate"],
        "imu1-rot": ["IMU1", "Rotational Velocity"],

        "imu2-fb": ["IMU2", "Forward/backwards Tilt"],
        "imu2-ss": ["IMU2", "Side-to-Side Tilt"],
        "imu2-yaw": ["IMU2", "Yaw"],
        "imu2-pitch": ["IMU2", "Pitch Rate"],
        "imu2-roll": ["IMU2", "Roll Rate"],
        "imu2-rot": ["IMU2", "Rotational Velocity"],

        "imu1linear-lv": ["IMU1Linear", "Linear Velocity"],
        "imu1linear-xv": ["IMU1Linear", "X velocity"],
        "imu1linear-yv": ["IMU1Linear", "Y velocity"],

        "enc-l-val": ["EncoderL", "Speed"],
        "enc-r-val": ["EncoderR", "Speed"],

        "robot-yaw-rate": ["Robot", "Yaw Rate"],
        "pend-ang-vel": ["Pendulum", "Angular Velocity"],
        "pend-angle": ["Pendulum", "Angle"],
        "pend-angle-deg": ["Pendulum", "AngleDeg"],
        "ultra-right": ["Ultrasonic", "Right"],
        "ultra-left": ["Ultrasonic", "Left"],
    };

    const sensorHistory = {};
    Object.keys(sensorNumericKeys).forEach(k => sensorHistory[k] = []);

    function addSample(key, value) {
        const arr = sensorHistory[key];
        const num = parseFloat(value);
        if (!isNaN(num)) {
            arr.push(num);
            if (arr.length > SENSOR_MAX_SAMPLES) arr.shift();
        }
    }

    function computeAvg(arr) {
        if (!arr || arr.length === 0) return "";
        let sum = 0;
        for (let v of arr) sum += v;
        const avg = sum / arr.length;
        return avg.toFixed(2);
    }

    function updateAverages() {
        Object.keys(sensorHistory).forEach(key => {
            const avgEl = document.getElementById(key + "-avg");
            if (!avgEl) return;
            const avg = computeAvg(sensorHistory[key]);
            avgEl.textContent = avg ? ("avg " + avg) : "";
        });
    }

    function updateSensorFeed() {
        fetch('/sensor_feed')
            .then(r => r.json())
            .then(data => {
                getval('imu-time', data.IMU1["Time"]);

                const vimu1fb = data.IMU1["Forward/backwards Tilt"];
                const vimu1ss = data.IMU1["Side-to-Side Tilt"];
                const vimu1yaw = data.IMU1["Yaw"];
                const vimu1pitch = data.IMU1["Pitch Rate"];
                const vimu1roll = data.IMU1["Roll Rate"];
                const vimu1rot = data.IMU1["Rotational Velocity"];

                const vimu2fb = data.IMU2["Forward/backwards Tilt"];
                const vimu2ss = data.IMU2["Side-to-Side Tilt"];
                const vimu2yaw = data.IMU2["Yaw"];
                const vimu2pitch = data.IMU2["Pitch Rate"];
                const vimu2roll = data.IMU2["Roll Rate"];
                const vimu2rot = data.IMU2["Rotational Velocity"];

                const vlv = data.IMU1Linear["Linear Velocity"];
                const vxv = data.IMU1Linear["X velocity"];
                const vyv = data.IMU1Linear["Y velocity"];

                const venclspd = data.EncoderL["Speed"];
                const vencldir = data.EncoderL["Direction"];
                const vencrspd = data.EncoderR["Speed"];
                const vencrdir = data.EncoderR["Direction"];

                const vrobotYaw = data.Robot["Yaw Rate"];
                const vpendAngVel = data.Pendulum["Angular Velocity"];
                const vpendAngle = data.Pendulum["Angle"];
                const vpendAngleDeg = data.Pendulum["AngleDeg"];
                const vultRight = data.Ultrasonic["Right"];
                const vultLeft = data.Ultrasonic["Left"];

                getval('imu1-fb', vimu1fb);
                getval('imu1-ss', vimu1ss);
                getval('imu1-yaw', vimu1yaw);
                getval('imu1-pitch', vimu1pitch);
                getval('imu1-roll', vimu1roll);
                getval('imu1-rot', vimu1rot);

                getval('imu2-fb', vimu2fb);
                getval('imu2-ss', vimu2ss);
                getval('imu2-yaw', vimu2yaw);
                getval('imu2-pitch', vimu2pitch);
                getval('imu2-roll', vimu2roll);
                getval('imu2-rot', vimu2rot);

                getval('imu1linear-lv', vlv);
                getval('imu1linear-xv', vxv);
                getval('imu1linear-yv', vyv);

                getval('enc-l-val', venclspd);
                getval('enc-l-status', vencldir);
                getval('enc-r-val', vencrspd);
                getval('enc-r-status', vencrdir);

                getval('robot-yaw-rate', vrobotYaw);
                getval('pend-ang-vel', vpendAngVel);
                getval('pend-angle', vpendAngle);
                getval('pend-angle-deg', vpendAngleDeg);
                getval('ultra-right', vultRight);
                getval('ultra-left', vultLeft);

                addSample('imu1-fb', vimu1fb);
                addSample('imu1-ss', vimu1ss);
                addSample('imu1-yaw', vimu1yaw);
                addSample('imu1-pitch', vimu1pitch);
                addSample('imu1-roll', vimu1roll);
                addSample('imu1-rot', vimu1rot);

                addSample('imu2-fb', vimu2fb);
                addSample('imu2-ss', vimu2ss);
                addSample('imu2-yaw', vimu2yaw);
                addSample('imu2-pitch', vimu2pitch);
                addSample('imu2-roll', vimu2roll);
                addSample('imu2-rot', vimu2rot);

                addSample('imu1linear-lv', vlv);
                addSample('imu1linear-xv', vxv);
                addSample('imu1linear-yv', vyv);

                addSample('enc-l-val', venclspd);
                addSample('enc-r-val', vencrspd);

                addSample('robot-yaw-rate', vrobotYaw);
                addSample('pend-ang-vel', vpendAngVel);
                addSample('pend-angle', vpendAngle);
                addSample('pend-angle-deg', vpendAngleDeg);
                addSample('ultra-right', vultRight);
                addSample('ultra-left', vultLeft);

                updateAverages();
            })
            .catch(err => {
                console.error('Sensor feed error', err);
            });
    }

    setInterval(updateSensorFeed, SENSOR_POLL_MS);
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
                    s.className = "ON";
                    startCalibrationCountdown(10, function () {
                        s.textContent = "ON";
                        s.className = "ON";
                    });
                    logLine("SENSOR", logMsg);
                } else if (!on && data.status === "OFF") {
                    s.textContent = "OFF";
                    s.className = "OFF";
                    document.getElementById('calib-status').textContent = "Auto";
                    if (calibTimer) clearInterval(calibTimer);
                    logLine("SENSOR", logMsg);
                } else {
                    logLine("SENSOR", "Error toggling sensors");
                }
            });
    }

    function startCalibrationCountdown(seconds, donecb) {
        let counter = seconds;
        document.getElementById('calib-status').textContent = "Calibrating " + counter;
        let s = document.getElementById('sensor-switch');
        s.textContent = "ON";
        s.className = "CALIB";

        if (calibTimer) clearInterval(calibTimer);
        calibTimer = setInterval(function () {
            counter--;
            document.getElementById('calib-status').textContent =
                counter > 0 ? "Calibrating " + counter : "Done";
            if (counter <= 0) {
                clearInterval(calibTimer);
                if (donecb) donecb();
            }
        }, 1000);
    }

    function startMotorTest() {
        logLine("MOTOR", "Starting Motor Test clearing numbers.txt");
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
                logLine("ERR", "Motor Test start error " + err);
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
                logLine("ERR", "Motor Test stop error " + err);
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
                logLine("ERR", "ML start error " + err);
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
                logLine("ERR", "ML stop error " + err);
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
                logLine("ERR", "AutoNav start error " + err);
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
                logLine("ERR", "AutoNav stop error " + err);
            });
    }

    function logLine(tag, text) {
        const c = document.getElementById('console');
        if (!c) return;
        const div = document.createElement('div');
        div.className = 'line';
        const t = document.createElement('span');
        t.className = 'tag';
        t.textContent = tag;
        const span = document.createElement('span');
        span.textContent = text;
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
            if (shouldLog) logLine("ERR", "Joystick send failed " + err);
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

        document.getElementById('joy-x').textContent = normX.toFixed(2);
        document.getElementById('joy-y').textContent = normY.toFixed(2);

        sendJoystick(normX, normY);
    }

    function resetJoystick() {
        setKnob(joyCenter.x - knob.clientWidth / 2, joyCenter.y - knob.clientHeight / 2);
        document.getElementById('joy-x').textContent = "0.00";
        document.getElementById('joy-y').textContent = "0.00";
        const now = Date.now();
        if (now - lastJoystickLog > joystickThrottleMs) {
            lastJoystickLog = now;
            sendJoystick(0, 0);
        }
    }

    area.addEventListener('mousedown', function (e) {
        joyActive = true;
        updateJoystick(e.clientX, e.clientY);
    });

    window.addEventListener('mousemove', function (e) {
        if (joyActive) updateJoystick(e.clientX, e.clientY);
    });

    window.addEventListener('mouseup', function () {
        if (joyActive) {
            joyActive = false;
            resetJoystick();
        }
    });

    area.addEventListener('touchstart', function (e) {
        e.preventDefault();
        joyActive = true;
        const t = e.touches[0];
        updateJoystick(t.clientX, t.clientY);
    }, { passive: false });

    area.addEventListener('touchmove', function (e) {
        e.preventDefault();
        if (!joyActive) return;
        const t = e.touches[0];
        updateJoystick(t.clientX, t.clientY);
    }, { passive: false });

    area.addEventListener('touchend', function (e) {
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
