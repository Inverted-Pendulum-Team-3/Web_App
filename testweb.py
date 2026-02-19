from flask import Flask, jsonify, request
import subprocess
import os
import signal
from datetime import datetime
import sqlite3
from datetime import datetime as dt
import time

app = Flask(__name__)

# ==============================
# Configurable program commands
# ==============================

IMU_SCRIPT = "imuencoderv4.py"
MOTOR_TEST_SCRIPT = "motortest.py"
ML_SCRIPT = "ml.py"
AUTONAV_SCRIPT = "autonav.py"

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DB_FILE = os.path.join(_SCRIPT_DIR, "sensor_data.db")

# ==============================
# Process handles
# ==============================

imu_proc = None
motor_test_proc = None
ml_proc = None
autonav_proc = None

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


def is_proc_running(proc):
    return proc is not None and proc.poll() is None


def start_generic(script_path):
    return subprocess.Popen(['python3', script_path], preexec_fn=os.setsid)


def stop_generic(proc):
    if is_proc_running(proc):
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        except Exception:
            pass
    return None


def is_imu_running():
    global imu_proc
    return is_proc_running(imu_proc)


def start_imu():
    global imu_proc
    if not is_imu_running():
        imu_proc = start_generic(IMU_SCRIPT)
        log_to_file("SYSTEM", "IMU Process started")
    return is_imu_running()


def stop_imu():
    global imu_proc
    if is_imu_running():
        imu_proc = stop_generic(imu_proc)
        log_to_file("SYSTEM", "IMU Process stopped")


def is_motor_test_running():
    global motor_test_proc
    return is_proc_running(motor_test_proc)


def start_motor_test():
    global motor_test_proc
    if not is_motor_test_running():
        try:
            with open("numbers.txt", "w") as f:
                f.write("")
            log_to_file("SYSTEM", "numbers.txt cleared")
        except Exception as e:
            print(f"Error clearing numbers.txt: {e}")
        motor_test_proc = start_generic(MOTOR_TEST_SCRIPT)
        log_to_file("SYSTEM", "Motor Test Process started")
    return is_motor_test_running()


def stop_motor_test():
    global motor_test_proc
    if is_motor_test_running():
        motor_test_proc = stop_generic(motor_test_proc)
        log_to_file("SYSTEM", "Motor Test Process stopped")


def are_ml_autonav_running():
    global ml_proc, autonav_proc
    return is_proc_running(ml_proc) or is_proc_running(autonav_proc)


def start_ml_autonav():
    global ml_proc, autonav_proc
    if not is_proc_running(ml_proc):
        ml_proc = start_generic(ML_SCRIPT)
        log_to_file("SYSTEM", "ML Process started")
    if not is_proc_running(autonav_proc):
        autonav_proc = start_generic(AUTONAV_SCRIPT)
        log_to_file("SYSTEM", "Autonav Process started")
    return are_ml_autonav_running()


def stop_ml_autonav():
    global ml_proc, autonav_proc
    if is_proc_running(ml_proc):
        ml_proc = stop_generic(ml_proc)
        log_to_file("SYSTEM", "ML Process stopped")
    if is_proc_running(autonav_proc):
        autonav_proc = stop_generic(autonav_proc)
        log_to_file("SYSTEM", "Autonav Process stopped")


# ==============================
# DB helpers
# ==============================

def get_latest_sensor_row(db_path=DB_FILE):
    if not os.path.exists(db_path):
        return None

    conn = sqlite3.connect(db_path)
    conn.row_factory = sqlite3.Row
    try:
        cursor = conn.cursor()
        cursor.execute(
            """
            SELECT * FROM sensor_readings
            ORDER BY timestamp DESC
            LIMIT 1
            """
        )
        row = cursor.fetchone()
    except Exception as e:
        print(f"DB error: {e}")
        row = None
    finally:
        conn.close()
    return row


def map_db_row_to_sensor_dict(row):
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
        ts = row["timestamp"]
        result["IMU1"]["Time"] = dt.fromtimestamp(ts).strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]

        if "imu1_body_pitch" in row.keys():
            result["IMU1"]["Forward/backwards Tilt"] = f"{row['imu1_body_pitch']:.4f}"
        if "imu1_yaw_rate" in row.keys():
            result["IMU1"]["Yaw"] = f"{row['imu1_yaw_rate']:.4f}"
            result["IMU1"]["Rotational Velocity"] = f"{row['imu1_yaw_rate']:.4f}"
        if "imu1_vx" in row.keys():
            result["IMU1Linear"]["Linear Velocity"] = f"{row['imu1_vx']:.4f}"
            result["IMU1Linear"]["X velocity"] = f"{row['imu1_vx']:.4f}"

        if "imu2_pendulum_angle" in row.keys():
            result["IMU2"]["Forward/backwards Tilt"] = f"{row['imu2_pendulum_angle']:.4f}"
        if "imu2_pendulum_ang_vel" in row.keys():
            result["IMU2"]["Pitch Rate"] = f"{row['imu2_pendulum_ang_vel']:.4f}"

        if "encoder_left_rad_s" in row.keys():
            result["EncoderL"]["Speed"] = f"{row['encoder_left_rad_s']:.4f}"
        if "encoder_right_rad_s" in row.keys():
            result["EncoderR"]["Speed"] = f"{row['encoder_right_rad_s']:.4f}"

        result["EncoderL"]["Direction"] = ""
        result["EncoderR"]["Direction"] = ""
    except Exception as e:
        print(f"Error mapping DB row to sensor dict: {e}")

    return result


def parse_sensor_line(line):
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
    try:
        tokens = [tok.strip() for tok in line.split(",")]
        idx = 0
        if idx < len(tokens) and ":" in tokens[idx]:
            result["IMU1"]["Time"] = tokens[idx]
            idx += 1
        if idx < len(tokens) and tokens[idx] == "IMU1":
            idx += 1
        imu_fields = [
            "Forward/backwards Tilt", "Side-to-Side Tilt", "Yaw",
            "Pitch Rate", "Roll Rate", "Rotational Velocity"
        ]
        for field in imu_fields:
            if idx + 1 < len(tokens) and tokens[idx] == field:
                result["IMU1"][field] = tokens[idx + 1]
                idx += 2
        if idx < len(tokens) and tokens[idx] == "IMU2":
            idx += 1
        for field in imu_fields:
            if idx + 1 < len(tokens) and tokens[idx] == field:
                result["IMU2"][field] = tokens[idx + 1]
                idx += 2
        if idx < len(tokens) and tokens[idx] == "IMU1 Linear Velocity":
            idx += 1
        if idx < len(tokens):
            result["IMU1Linear"]["Linear Velocity"] = tokens[idx]
            idx += 1
        if idx < len(tokens) and tokens[idx] == "IMU1's X-Velocity":
            idx += 1
        if idx < len(tokens):
            result["IMU1Linear"]["X velocity"] = tokens[idx]
            idx += 1
        if idx < len(tokens) and tokens[idx] == "IMU1's Y-Velocity":
            idx += 1
        if idx < len(tokens):
            result["IMU1Linear"]["Y velocity"] = tokens[idx]
            idx += 1
        if idx < len(tokens) and tokens[idx] == "EncoderL":
            idx += 1
        if idx < len(tokens):
            result["EncoderL"]["Speed"] = tokens[idx]
            idx += 1
        if idx < len(tokens):
            result["EncoderL"]["Direction"] = tokens[idx]
            idx += 1
        if idx < len(tokens) and tokens[idx] == "EncoderR":
            idx += 1
        if idx < len(tokens):
            result["EncoderR"]["Speed"] = tokens[idx]
            idx += 1
        if idx < len(tokens):
            result["EncoderR"]["Direction"] = tokens[idx]
            idx += 1
    except Exception:
        pass
    return result


@app.route("/sensor_feed")
def sensor_feed():
    row = get_latest_sensor_row()
    if row is not None:
        return jsonify(map_db_row_to_sensor_dict(row))
    else:
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


@app.route("/ml_on", methods=["POST"])
def ml_on():
    ok = start_ml_autonav()
    return jsonify({"status": "ON" if ok else "ERROR"})


@app.route("/ml_off", methods=["POST"])
def ml_off():
    stop_ml_autonav()
    return jsonify({"status": "OFF"})


@app.route("/")
def home():
    log_to_file("SYSTEM", "Web interface loaded")
    return """
<!DOCTYPE html>
<html>
<head>
    <title>Robot Sensor Dashboard</title>
    <style>
        body { font-family: Arial, sans-serif; background-color: #202020; color: white; }
        .container { display: flex; }
        .left-panel { width: 65%; padding: 10px; }
        .right-panel { width: 35%; padding: 10px; }
        table { border-collapse: collapse; width: 100%; margin-bottom: 10px; }
        th, td { border: 1px solid #555; padding: 4px; text-align: center; }
        th { background-color: #303030; }
        .joystick-container { position: relative; width: 200px; height: 200px; margin: 20px auto; border: 2px solid #555; border-radius: 50%; background-color: #101010; }
        .joystick-knob { position: absolute; width: 40px; height: 40px; border-radius: 50%; background-color: #888; top: 80px; left: 80px; cursor: pointer; }
        .button-row { margin: 10px 0; }
        button { margin-right: 5px; padding: 8px 12px; background-color: #444; color: white; border: 1px solid #666; cursor: pointer; }
        button:hover { background-color: #666; }
        .top-row-btn { margin-top: 5px; }
        .status { margin-top: 10px; font-size: 0.9em; color: #ccc; }
    </style>
</head>
<body>
    <h1>Robot Sensor Dashboard</h1>
    <div class="container">
        <div class="left-panel">
            <h2>Sensor Readings</h2>
            <table id="sensor-table">
                <tr>
                    <th>Sensor</th>
                    <th>Parameter</th>
                    <th>Value</th>
                </tr>
                <tr><td>IMU1</td><td>Time</td><td id="imu1_time"></td></tr>
                <tr><td>IMU1</td><td>Forward/backwards Tilt</td><td id="imu1_fwd"></td></tr>
                <tr><td>IMU1</td><td>Side-to-Side Tilt</td><td id="imu1_side"></td></tr>
                <tr><td>IMU1</td><td>Yaw</td><td id="imu1_yaw"></td></tr>
                <tr><td>IMU1</td><td>Pitch Rate</td><td id="imu1_pitch_rate"></td></tr>
                <tr><td>IMU1</td><td>Roll Rate</td><td id="imu1_roll_rate"></td></tr>
                <tr><td>IMU1</td><td>Rotational Velocity</td><td id="imu1_rot_vel"></td></tr>

                <tr><td>IMU2</td><td>Forward/backwards Tilt</td><td id="imu2_fwd"></td></tr>
                <tr><td>IMU2</td><td>Side-to-Side Tilt</td><td id="imu2_side"></td></tr>
                <tr><td>IMU2</td><td>Yaw</td><td id="imu2_yaw"></td></tr>
                <tr><td>IMU2</td><td>Pitch Rate</td><td id="imu2_pitch_rate"></td></tr>
                <tr><td>IMU2</td><td>Roll Rate</td><td id="imu2_roll_rate"></td></tr>
                <tr><td>IMU2</td><td>Rotational Velocity</td><td id="imu2_rot_vel"></td></tr>

                <tr><td>IMU1Linear</td><td>Linear Velocity</td><td id="imu1_lin_vel"></td></tr>
                <tr><td>IMU1Linear</td><td>X velocity</td><td id="imu1_x_vel"></td></tr>
                <tr><td>IMU1Linear</td><td>Y velocity</td><td id="imu1_y_vel"></td></tr>

                <tr><td>EncoderL</td><td>Speed</td><td id="enc_l_speed"></td></tr>
                <tr><td>EncoderL</td><td>Direction</td><td id="enc_l_dir"></td></tr>
                <tr><td>EncoderR</td><td>Speed</td><td id="enc_r_speed"></td></tr>
                <tr><td>EncoderR</td><td>Direction</td><td id="enc_r_dir"></td></tr>
            </table>

            <div class="button-row">
                <button id="sensor-on-btn">Sensor ON</button>
                <button id="sensor-off-btn">Sensor OFF</button>
                <button id="motor-start-btn">Start Motor Test</button>
                <button id="motor-stop-btn">Stop Motor Test</button>
                <button id="ml-on-btn">ML ON</button>
                <button id="ml-off-btn">ML OFF</button>
            </div>
            <button class="top-row-btn" id="remove-top-row-btn">Remove Top Row</button>
            <div class="status" id="status-msg"></div>
        </div>

        <div class="right-panel">
            <h2>Joystick Control</h2>
            <div class="joystick-container" id="joystick">
                <div class="joystick-knob" id="joystick-knob"></div>
            </div>
            <div class="status">
                X: <span id="joy-x">0.00</span>,
                Y: <span id="joy-y">0.00</span>
            </div>
        </div>
    </div>

    <script>
        function updateSensorTable(data) {
            document.getElementById("imu1_time").textContent = data.IMU1.Time;
            document.getElementById("imu1_fwd").textContent = data.IMU1["Forward/backwards Tilt"];
            document.getElementById("imu1_side").textContent = data.IMU1["Side-to-Side Tilt"];
            document.getElementById("imu1_yaw").textContent = data.IMU1["Yaw"];
            document.getElementById("imu1_pitch_rate").textContent = data.IMU1["Pitch Rate"];
            document.getElementById("imu1_roll_rate").textContent = data.IMU1["Roll Rate"];
            document.getElementById("imu1_rot_vel").textContent = data.IMU1["Rotational Velocity"];

            document.getElementById("imu2_fwd").textContent = data.IMU2["Forward/backwards Tilt"];
            document.getElementById("imu2_side").textContent = data.IMU2["Side-to-Side Tilt"];
            document.getElementById("imu2_yaw").textContent = data.IMU2["Yaw"];
            document.getElementById("imu2_pitch_rate").textContent = data.IMU2["Pitch Rate"];
            document.getElementById("imu2_roll_rate").textContent = data.IMU2["Roll Rate"];
            document.getElementById("imu2_rot_vel").textContent = data.IMU2["Rotational Velocity"];

            document.getElementById("imu1_lin_vel").textContent = data.IMU1Linear["Linear Velocity"];
            document.getElementById("imu1_x_vel").textContent = data.IMU1Linear["X velocity"];
            document.getElementById("imu1_y_vel").textContent = data.IMU1Linear["Y velocity"];

            document.getElementById("enc_l_speed").textContent = data.EncoderL["Speed"];
            document.getElementById("enc_l_dir").textContent = data.EncoderL["Direction"];
            document.getElementById("enc_r_speed").textContent = data.EncoderR["Speed"];
            document.getElementById("enc_r_dir").textContent = data.EncoderR["Direction"];
        }

        function fetchSensorData() {
            fetch("/sensor_feed")
                .then(response => response.json())
                .then(data => {
                    updateSensorTable(data);
                })
                .catch(err => console.error("Error fetching sensor feed:", err));
        }

        setInterval(fetchSensorData, 200);

        document.getElementById("sensor-on-btn").addEventListener("click", function() {
            fetch("/sensor_on", {method: "POST"})
                .then(r => r.json())
                .then(data => {
                    document.getElementById("status-msg").textContent = "Sensor: " + data.status;
                });
        });

        document.getElementById("sensor-off-btn").addEventListener("click", function() {
            fetch("/sensor_off", {method: "POST"})
                .then(r => r.json())
                .then(data => {
                    document.getElementById("status-msg").textContent = "Sensor: " + data.status;
                });
        });

        document.getElementById("motor-start-btn").addEventListener("click", function() {
            fetch("/start_motor_test", {method: "POST"})
                .then(r => r.json())
                .then(data => {
                    document.getElementById("status-msg").textContent = "Motor test: " + data.status;
                });
        });

        document.getElementById("motor-stop-btn").addEventListener("click", function() {
            fetch("/stop_motor_test", {method: "POST"})
                .then(r => r.json())
                .then(data => {
                    document.getElementById("status-msg").textContent = "Motor test: " + data.status;
                });
        });

        document.getElementById("ml-on-btn").addEventListener("click", function() {
            fetch("/ml_on", {method: "POST"})
                .then(r => r.json())
                .then(data => {
                    document.getElementById("status-msg").textContent = "ML: " + data.status;
                });
        });

        document.getElementById("ml-off-btn").addEventListener("click", function() {
            fetch("/ml_off", {method: "POST"})
                .then(r => r.json())
                .then(data => {
                    document.getElementById("status-msg").textContent = "ML: " + data.status;
                });
        });

        document.getElementById("remove-top-row-btn").addEventListener("click", function() {
            fetch("/get_top_row", {method: "POST"})
                .then(r => r.json())
                .then(data => {
                    document.getElementById("status-msg").textContent = data.message;
                });
        });

        const joystick = document.getElementById("joystick");
        const knob = document.getElementById("joystick-knob");
        const joyXSpan = document.getElementById("joy-x");
        const joyYSpan = document.getElementById("joy-y");
        const rect = joystick.getBoundingClientRect();
        const centerX = rect.width / 2;
        const centerY = rect.height / 2;
        const maxRadius = rect.width / 2 - knob.offsetWidth / 2;
        let dragging = false;

        function sendJoystick(x, y) {
            fetch("/direction_ajax", {
                method: "POST",
                headers: {"Content-Type": "application/json"},
                body: JSON.stringify({x: x, y: y})
            })
            .then(r => r.json())
            .then(data => {
                document.getElementById("status-msg").textContent = data.message;
            })
            .catch(err => console.error("Joystick AJAX error:", err));
        }

        knob.addEventListener("mousedown", function(e) {
            dragging = true;
        });

        document.addEventListener("mouseup", function(e) {
            if (dragging) {
                dragging = false;
                knob.style.left = (centerX - knob.offsetWidth / 2) + "px";
                knob.style.top = (centerY - knob.offsetHeight / 2) + "px";
                joyXSpan.textContent = "0.00";
                joyYSpan.textContent = "0.00";
                sendJoystick(0, 0);
            }
        });

        document.addEventListener("mousemove", function(e) {
            if (!dragging) return;
            const rect = joystick.getBoundingClientRect();
            const x = e.clientX - rect.left;
            const y = e.clientY - rect.top;
            let dx = x - centerX;
            let dy = y - centerY;
            const dist = Math.sqrt(dx*dx + dy*dy);
            if (dist > maxRadius) {
                dx = dx * maxRadius / dist;
                dy = dy * maxRadius / dist;
            }
            knob.style.left = (centerX + dx - knob.offsetWidth / 2) + "px";
            knob.style.top = (centerY + dy - knob.offsetHeight / 2) + "px";
            const normX = dx / maxRadius;
            const normY = -dy / maxRadius;
            joyXSpan.textContent = normX.toFixed(2);
            joyYSpan.textContent = normY.toFixed(2);
            sendJoystick(normX, normY);
        });
    </script>
</body>
</html>
    """


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000, debug=False)
