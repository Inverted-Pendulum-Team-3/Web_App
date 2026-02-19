#!/usr/bin/env python3

from flask import Flask, jsonify, request
import subprocess
import os
import signal
import sqlite3
from datetime import datetime

app = Flask(__name__)

# ==========================
# Program control variables
# ==========================
IMU_SCRIPT = "imuencoderv4.py"
MOTOR_TEST_SCRIPT = "motortest.py"
ML_SCRIPT = "ml.py"
AUTONAV_SCRIPT = "autonav.py"

# Process handles
imu_proc = None
motor_test_proc = None
ml_proc = None
autonav_proc = None

# Joystick throttling
last_joystick_send = 0
joystick_throttle_interval = 0.25  # seconds

# SQLite database configuration (same pattern as query_sensor_db.py)
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DB_FILE = os.path.join(_SCRIPT_DIR, "sensor_data.db")


def log_to_file(tag, message):
    """Log events to numbers.txt with timestamp"""
    try:
        with open("numbers.txt", "a") as f:
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
            f.write(f"{timestamp} | {tag} | {message}\n")
    except Exception as e:
        print(f"Error writing to numbers.txt: {e}")


# ==============
# DB utilities
# ==============
def get_latest_reading(db_path=DB_FILE):
    """
    Get the most recent reading row from sensor_readings.
    This mirrors the DB usage style in query_sensor_db.py but just returns 1 row.
    """
    try:
        conn = sqlite3.connect(db_path)
        conn.row_factory = sqlite3.Row
        cursor = conn.cursor()
        cursor.execute(
            """
            SELECT * FROM sensor_readings
            ORDER BY timestamp DESC
            LIMIT 1
            """
        )
        row = cursor.fetchone()
        conn.close()
        return row
    except Exception as e:
        print(f"Error reading from SQLite DB: {e}")
        return None


# ==========================
# Process control functions
# ==========================
def is_imu_running():
    global imu_proc
    return imu_proc is not None and imu_proc.poll() is None


def start_imu():
    global imu_proc
    if not is_imu_running():
        imu_proc = subprocess.Popen(['python3', IMU_SCRIPT], preexec_fn=os.setsid)
        log_to_file("SYSTEM", "IMU Process started")
    return is_imu_running()


def stop_imu():
    global imu_proc
    if is_imu_running():
        try:
            os.killpg(os.getpgid(imu_proc.pid), signal.SIGTERM)
            log_to_file("SYSTEM", "IMU Process stopped")
        except Exception:
            pass
    imu_proc = None


def is_motor_test_running():
    global motor_test_proc
    return motor_test_proc is not None and motor_test_proc.poll() is None


def start_motor_test():
    global motor_test_proc
    if not is_motor_test_running():
        # Clear numbers.txt first
        try:
            with open("numbers.txt", "w") as f:
                f.write("")
            log_to_file("SYSTEM", "numbers.txt cleared")
        except Exception as e:
            print(f"Error clearing numbers.txt: {e}")
        # Start motortest.py
        motor_test_proc = subprocess.Popen(['python3', MOTOR_TEST_SCRIPT], preexec_fn=os.setsid)
        log_to_file("SYSTEM", "Motor Test Process started")
    return is_motor_test_running()


def stop_motor_test():
    global motor_test_proc
    if is_motor_test_running():
        try:
            os.killpg(os.getpgid(motor_test_proc.pid), signal.SIGTERM)
            log_to_file("SYSTEM", "Motor Test Process stopped")
        except Exception:
            pass
    motor_test_proc = None


def is_ml_running():
    global ml_proc
    return ml_proc is not None and ml_proc.poll() is None


def start_ml():
    global ml_proc
    if not is_ml_running():
        ml_proc = subprocess.Popen(['python3', ML_SCRIPT], preexec_fn=os.setsid)
        log_to_file("SYSTEM", "ML Process started")
    return is_ml_running()


def stop_ml():
    global ml_proc
    if is_ml_running():
        try:
            os.killpg(os.getpgid(ml_proc.pid), signal.SIGTERM)
            log_to_file("SYSTEM", "ML Process stopped")
        except Exception:
            pass
    ml_proc = None


def is_autonav_running():
    global autonav_proc
    return autonav_proc is not None and autonav_proc.poll() is None


def start_autonav():
    global autonav_proc
    if not is_autonav_running():
        autonav_proc = subprocess.Popen(['python3', AUTONAV_SCRIPT], preexec_fn=os.setsid)
        log_to_file("SYSTEM", "Autonav Process started")
    return is_autonav_running()


def stop_autonav():
    global autonav_proc
    if is_autonav_running():
        try:
            os.killpg(os.getpgid(autonav_proc.pid), signal.SIGTERM)
            log_to_file("SYSTEM", "Autonav Process stopped")
        except Exception:
            pass
    autonav_proc = None


# ==========================
# Sensor parsing
# ==========================
def parse_sensor_row(row):
    """
    Convert a DB row (sqlite3.Row) into the same structure the frontend expects.
    This assumes the sensor_readings table has fields analogous to the previous text format.
    Adjust the field mapping below to match your actual DB schema.
    """
    # Default empty structure (matches original parse_sensor_line result keys)
    result = {
        "IMU1": {
            "Time": "",
            "Forward/backwards Tilt": "",
            "Side-to-Side Tilt": "",
            "Yaw": "",
            "Pitch Rate": "",
            "Roll Rate": "",
            "Rotational Velocity": ""
        },
        "IMU2": {
            "Forward/backwards Tilt": "",
            "Side-to-Side Tilt": "",
            "Yaw": "",
            "Pitch Rate": "",
            "Roll Rate": "",
            "Rotational Velocity": ""
        },
        "IMU1Linear": {
            "Linear Velocity": "",
            "X velocity": "",
            "Y velocity": ""
        },
        "EncoderL": {
            "Speed": "",
            "Direction": ""
        },
        "EncoderR": {
            "Speed": "",
            "Direction": ""
        }
    }

    if row is None:
        return result

    try:
        # Time
        if "timestamp" in row.keys():
            dt = datetime.fromtimestamp(row["timestamp"])
            result["IMU1"]["Time"] = dt.strftime("%H:%M:%S.%f")[:-3]

        # Example mappings; update to match your actual column names.
        # IMU1 (body) – using query_sensor_db.py fields as hints
        if "imu1_body_pitch" in row.keys():
            result["IMU1"]["Forward/backwards Tilt"] = f"{row['imu1_body_pitch']:.4f}"
        if "imu1_body_roll" in row.keys():
            result["IMU1"]["Side-to-Side Tilt"] = f"{row['imu1_body_roll']:.4f}"
        if "imu1_yaw_rate" in row.keys():
            result["IMU1"]["Yaw"] = f"{row['imu1_yaw_rate']:.4f}"
        if "imu1_pitch_rate" in row.keys():
            result["IMU1"]["Pitch Rate"] = f"{row['imu1_pitch_rate']:.4f}"
        if "imu1_roll_rate" in row.keys():
            result["IMU1"]["Roll Rate"] = f"{row['imu1_roll_rate']:.4f}"
        if "imu1_vx" in row.keys():
            result["IMU1"]["Rotational Velocity"] = f"{row['imu1_vx']:.4f}"

        # IMU2 (pendulum)
        if "imu2_pendulum_angle" in row.keys():
            result["IMU2"]["Forward/backwards Tilt"] = f"{row['imu2_pendulum_angle']:.4f}"
        if "imu2_pendulum_ang_vel" in row.keys():
            result["IMU2"]["Side-to-Side Tilt"] = f"{row['imu2_pendulum_ang_vel']:.4f}"
        # Fill other IMU2 fields if you have them in DB

        # IMU1 Linear velocities
        if "imu1_vx" in row.keys():
            result["IMU1Linear"]["Linear Velocity"] = f"{row['imu1_vx']:.4f}"
        if "imu1_vx" in row.keys():
            result["IMU1Linear"]["X velocity"] = f"{row['imu1_vx']:.4f}"
        if "imu1_vy" in row.keys():
            result["IMU1Linear"]["Y velocity"] = f"{row['imu1_vy']:.4f}"

        # Encoders
        if "encoder_left_rad_s" in row.keys():
            result["EncoderL"]["Speed"] = f"{row['encoder_left_rad_s']:.4f}"
        if "encoder_right_rad_s" in row.keys():
            result["EncoderR"]["Speed"] = f"{row['encoder_right_rad_s']:.4f}"
        # Direction fields are left as empty strings unless you have columns for them
    except Exception as e:
        print(f"Error parsing DB row: {e}")

    return result


# ==========================
# Flask routes
# ==========================
@app.route("/sensor_feed")
def sensor_feed():
    try:
        row = get_latest_reading()
        return jsonify(parse_sensor_row(row))
    except Exception:
        return jsonify(parse_sensor_row(None))


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
    # Only log to file if enough time has passed
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


# ============
# ML / Autonav
# ============
@app.route("/ml_toggle", methods=["POST"])
def ml_toggle():
    """
    Toggle ML and Autonav together from the ML button.
    If either is not running, start both.
    If both are running, stop both.
    """
    if not is_ml_running() or not is_autonav_running():
        # Start both
        start_ml()
        start_autonav()
        status = "started"
    else:
        # Stop both
        stop_ml()
        stop_autonav()
        status = "stopped"
    return jsonify({"status": status})


@app.route("/")
def home():
    log_to_file("SYSTEM", "Web interface loaded")
    # The HTML below is your original HTML, unchanged in layout, colors, and structure.
    # Only minor JS additions were made to hook up the new /ml_toggle route.
    return """
<!DOCTYPE html>
<html>
<head>
    <title>Robot Control Interface</title>
    <style>
        body {
            background-color: black;
            color: white;
            font-family: Arial, sans-serif;
        }
        .container {
            margin: 20px;
        }
        h1, h2 {
            color: white;
        }
        .sensor-table {
            border-collapse: collapse;
            width: 100%;
            margin-bottom: 20px;
        }
        .sensor-table th, .sensor-table td {
            border: 1px solid white;
            padding: 4px 8px;
            text-align: center;
        }
        .sensor-table th {
            background-color: #333333;
        }
        .sensor-table td {
            background-color: #111111;
        }
        .button-row {
            margin: 10px 0;
        }
        button {
            background-color: #444444;
            color: white;
            border: 1px solid white;
            padding: 6px 10px;
            margin-right: 5px;
            cursor: pointer;
        }
        button:hover {
            background-color: #666666;
        }
        .joystick {
            width: 200px;
            height: 200px;
            border: 2px solid white;
            border-radius: 50%;
            position: relative;
            margin-top: 20px;
        }
        .joystick-knob {
            width: 40px;
            height: 40px;
            border-radius: 50%;
            background-color: #888888;
            position: absolute;
            left: 80px;
            top: 80px;
        }
        .status-bar {
            margin-top: 10px;
            font-size: 14px;
        }
        .top-row-button {
            margin-top: 10px;
        }
    </style>
</head>
<body>
<div class="container">
    <h1>Robot Control Interface</h1>

    <div class="button-row">
        <button id="sensor-on-btn">Sensor ON</button>
        <button id="sensor-off-btn">Sensor OFF</button>
        <button id="motor-test-start-btn">Start Motor Test</button>
        <button id="motor-test-stop-btn">Stop Motor Test</button>
        <button id="ml-btn">ML</button>
        <button id="remove-row-btn">Remove Top Row</button>
    </div>

    <h2>IMU and Encoder Readings</h2>
    <table class="sensor-table">
        <tr>
            <th colspan="7">IMU1</th>
        </tr>
        <tr>
            <th>Time</th>
            <th>Forward/backwards Tilt</th>
            <th>Side-to-Side Tilt</th>
            <th>Yaw</th>
            <th>Pitch Rate</th>
            <th>Roll Rate</th>
            <th>Rotational Velocity</th>
        </tr>
        <tr id="imu1-row">
            <td id="imu1-time"></td>
            <td id="imu1-fb-tilt"></td>
            <td id="imu1-ss-tilt"></td>
            <td id="imu1-yaw"></td>
            <td id="imu1-pitch-rate"></td>
            <td id="imu1-roll-rate"></td>
            <td id="imu1-rot-vel"></td>
        </tr>

        <tr>
            <th colspan="6">IMU2</th>
        </tr>
        <tr>
            <th>Forward/backwards Tilt</th>
            <th>Side-to-Side Tilt</th>
            <th>Yaw</th>
            <th>Pitch Rate</th>
            <th>Roll Rate</th>
            <th>Rotational Velocity</th>
        </tr>
        <tr id="imu2-row">
            <td id="imu2-fb-tilt"></td>
            <td id="imu2-ss-tilt"></td>
            <td id="imu2-yaw"></td>
            <td id="imu2-pitch-rate"></td>
            <td id="imu2-roll-rate"></td>
            <td id="imu2-rot-vel"></td>
        </tr>

        <tr>
            <th colspan="3">IMU1 Linear Velocity</th>
        </tr>
        <tr>
            <th>Linear Velocity</th>
            <th>X velocity</th>
            <th>Y velocity</th>
        </tr>
        <tr id="imu1-linear-row">
            <td id="imu1-linear-vel"></td>
            <td id="imu1-x-vel"></td>
            <td id="imu1-y-vel"></td>
        </tr>

        <tr>
            <th colspan="2">Encoder L</th>
            <th colspan="2">Encoder R</th>
        </tr>
        <tr>
            <th>Speed</th>
            <th>Direction</th>
            <th>Speed</th>
            <th>Direction</th>
        </tr>
        <tr id="encoder-row">
            <td id="encoder-l-speed"></td>
            <td id="encoder-l-dir"></td>
            <td id="encoder-r-speed"></td>
            <td id="encoder-r-dir"></td>
        </tr>
    </table>

    <h2>Joystick</h2>
    <div class="joystick" id="joystick">
        <div class="joystick-knob" id="joystick-knob"></div>
    </div>
    <div class="status-bar">
        X: <span id="joystick-x">0.00</span> |
        Y: <span id="joystick-y">0.00</span> |
        Time: <span id="joystick-time">0.00</span>
    </div>
</div>

<script>
    // Sensor ON/OFF
    document.getElementById("sensor-on-btn").addEventListener("click", function() {
        fetch("/sensor_on", {method: "POST"})
            .then(response => response.json())
            .then(data => console.log("Sensor ON:", data));
    });

    document.getElementById("sensor-off-btn").addEventListener("click", function() {
        fetch("/sensor_off", {method: "POST"})
            .then(response => response.json())
            .then(data => console.log("Sensor OFF:", data));
    });

    // Motor Test Start/Stop
    document.getElementById("motor-test-start-btn").addEventListener("click", function() {
        fetch("/start_motor_test", {method: "POST"})
            .then(response => response.json())
            .then(data => console.log("Motor Test Start:", data));
    });

    document.getElementById("motor-test-stop-btn").addEventListener("click", function() {
        fetch("/stop_motor_test", {method: "POST"})
            .then(response => response.json())
            .then(data => console.log("Motor Test Stop:", data));
    });

    // ML button (toggles ML + Autonav)
    document.getElementById("ml-btn").addEventListener("click", function() {
        fetch("/ml_toggle", {method: "POST"})
            .then(response => response.json())
            .then(data => console.log("ML toggle:", data));
    });

    // Remove Top Row (existing behavior)
    document.getElementById("remove-row-btn").addEventListener("click", function() {
        fetch("/get_top_row", {method: "POST"})
            .then(response => response.json())
            .then(data => console.log("Remove Top Row:", data));
    });

    // Periodic sensor feed update (layout unchanged)
    function updateSensorData() {
        fetch("/sensor_feed")
            .then(response => response.json())
            .then(data => {
                const imu1 = data.IMU1 || {};
                const imu2 = data.IMU2 || {};
                const imu1Linear = data.IMU1Linear || {};
                const encL = data.EncoderL || {};
                const encR = data.EncoderR || {};

                document.getElementById("imu1-time").textContent = imu1["Time"] || "";
                document.getElementById("imu1-fb-tilt").textContent = imu1["Forward/backwards Tilt"] || "";
                document.getElementById("imu1-ss-tilt").textContent = imu1["Side-to-Side Tilt"] || "";
                document.getElementById("imu1-yaw").textContent = imu1["Yaw"] || "";
                document.getElementById("imu1-pitch-rate").textContent = imu1["Pitch Rate"] || "";
                document.getElementById("imu1-roll-rate").textContent = imu1["Roll Rate"] || "";
                document.getElementById("imu1-rot-vel").textContent = imu1["Rotational Velocity"] || "";

                document.getElementById("imu2-fb-tilt").textContent = imu2["Forward/backwards Tilt"] || "";
                document.getElementById("imu2-ss-tilt").textContent = imu2["Side-to-Side Tilt"] || "";
                document.getElementById("imu2-yaw").textContent = imu2["Yaw"] || "";
                document.getElementById("imu2-pitch-rate").textContent = imu2["Pitch Rate"] || "";
                document.getElementById("imu2-roll-rate").textContent = imu2["Roll Rate"] || "";
                document.getElementById("imu2-rot-vel").textContent = imu2["Rotational Velocity"] || "";

                document.getElementById("imu1-linear-vel").textContent = imu1Linear["Linear Velocity"] || "";
                document.getElementById("imu1-x-vel").textContent = imu1Linear["X velocity"] || "";
                document.getElementById("imu1-y-vel").textContent = imu1Linear["Y velocity"] || "";

                document.getElementById("encoder-l-speed").textContent = encL["Speed"] || "";
                document.getElementById("encoder-l-dir").textContent = encL["Direction"] || "";
                document.getElementById("encoder-r-speed").textContent = encR["Speed"] || "";
                document.getElementById("encoder-r-dir").textContent = encR["Direction"] || "";
            })
            .catch(err => console.error("Error fetching sensor data:", err));
    }

    setInterval(updateSensorData, 200);

    // Joystick handling (layout and behavior kept the same)
    const joystick = document.getElementById("joystick");
    const knob = document.getElementById("joystick-knob");
    const joyXSpan = document.getElementById("joystick-x");
    const joyYSpan = document.getElementById("joystick-y");
    const joyTimeSpan = document.getElementById("joystick-time");

    let joystickCenter = {x: 100, y: 100};
    let isDragging = false;

    joystick.addEventListener("mousedown", function(e) {
        isDragging = true;
        moveKnob(e);
    });

    document.addEventListener("mousemove", function(e) {
        if (isDragging) {
            moveKnob(e);
        }
    });

    document.addEventListener("mouseup", function() {
        if (isDragging) {
            isDragging = false;
            knob.style.left = "80px";
            knob.style.top = "80px";
            sendJoystick(0, 0);
        }
    });

    function moveKnob(e) {
        const rect = joystick.getBoundingClientRect();
        const x = e.clientX - rect.left;
        const y = e.clientY - rect.top;
        const dx = x - joystickCenter.x;
        const dy = y - joystickCenter.y;
        const dist = Math.sqrt(dx*dx + dy*dy);
        const maxDist = 80;

        let clampedX = dx;
        let clampedY = dy;
        if (dist > maxDist) {
            const ratio = maxDist / dist;
            clampedX = dx * ratio;
            clampedY = dy * ratio;
        }

        knob.style.left = (joystickCenter.x + clampedX - 20) + "px";
        knob.style.top = (joystickCenter.y + clampedY - 20) + "px";

        const normX = clampedX / maxDist;
        const normY = -clampedY / maxDist;

        sendJoystick(normX, normY);
    }

    function sendJoystick(x, y) {
        joyXSpan.textContent = x.toFixed(2);
        joyYSpan.textContent = y.toFixed(2);
        joyTimeSpan.textContent = (Date.now() / 1000).toFixed(2);

        fetch("/direction_ajax", {
            method: "POST",
            headers: {"Content-Type": "application/json"},
            body: JSON.stringify({x: x, y: y})
        })
        .then(response => response.json())
        .then(data => console.log("Joystick sent:", data))
        .catch(err => console.error("Joystick error:", err));
    }
</script>
</body>
</html>
"""


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000, debug=True)
