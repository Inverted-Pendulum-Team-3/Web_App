from flask import Flask, jsonify, request
import subprocess
import os
import signal
import sqlite3
from datetime import datetime
import time

app = Flask(__name__)

# =========================
# CONFIGURABLE PROGRAM NAMES
# =========================
IMU_SCRIPT = "imuencoderv4.py"
MOTOR_TEST_SCRIPT = "motortest.py"
ML_SCRIPT = "ml.py"
AUTONAV_SCRIPT = "autonav.py"

# DB configuration (same folder as this script)
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DB_FILE = os.path.join(_SCRIPT_DIR, "sensor_data.db")

# =========================
# PROCESS HANDLES
# =========================
imu_proc = None
motor_test_proc = None
ml_proc = None
autonav_proc = None

# Joystick throttling
last_joystick_send = 0
joystick_throttle_interval = 0.25  # seconds


# =========================
# LOGGING
# =========================
def log_to_file(tag, message):
    """Log events to numbers.txt with timestamp"""
    try:
        with open("numbers.txt", "a") as f:
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
            f.write(f"{timestamp} | {tag} | {message}\n")
    except Exception as e:
        print(f"Error writing to numbers.txt: {e}")


# =========================
# SQLITE HELPERS
# =========================
def get_latest_reading(db_path=DB_FILE):
    """
    Get the latest (most recent) sensor_readings row from the SQLite DB.
    Assumes a table 'sensor_readings' with a 'timestamp' column.
    """
    if not os.path.exists(db_path):
        return None

    conn = sqlite3.connect(db_path)
    conn.row_factory = sqlite3.Row
    cur = conn.cursor()

    cur.execute(
        """
        SELECT *
        FROM sensor_readings
        ORDER BY timestamp DESC
        LIMIT 1
        """
    )
    row = cur.fetchone()
    conn.close()
    return row


def reading_to_sensor_dict(row):
    """
    Convert a DB row (from sensor_readings) into the structure expected
    by the front end (matching the old parse_sensor_line output).
    This mapping uses the column names from query_sensor_db.py. Adjust if needed.
    """
    if row is None:
        # Return empty structure
        return {
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
            "EncoderL": {"Speed": "", "Direction": ""},
            "EncoderR": {"Speed": "", "Direction": ""},
        }

    # Build from DB fields (names taken from query_sensor_db.py)
    try:
        ts = row["timestamp"]
        dt_str = datetime.fromtimestamp(ts).strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
    except Exception:
        dt_str = ""

    result = {
        "IMU1": {
            "Time": dt_str,
            # You can decide which angle maps to which label; this is one example.
            "Forward/backwards Tilt": f"{row.get('imu1_body_pitch', 0.0):.4f}",
            "Side-to-Side Tilt": "",  # not present in DB, leave blank or compute
            "Yaw": "",  # no explicit yaw angle column in provided snippet
            "Pitch Rate": "",
            "Roll Rate": "",
            "Rotational Velocity": f"{row.get('imu1_yaw_rate', 0.0):.4f}",
        },
        "IMU2": {
            "Forward/backwards Tilt": f"{row.get('imu2_pendulum_angle', 0.0):.4f}",
            "Side-to-Side Tilt": "",
            "Yaw": "",
            "Pitch Rate": "",
            "Roll Rate": "",
            "Rotational Velocity": f"{row.get('imu2_pendulum_ang_vel', 0.0):.4f}",
        },
        "IMU1Linear": {
            "Linear Velocity": f"{row.get('imu1_vx', 0.0):.4f}",
            "X velocity": f"{row.get('imu1_vx', 0.0):.4f}",
            "Y velocity": "0.0000",  # no Y in snippet; adjust if you have it
        },
        "EncoderL": {
            "Speed": f"{row.get('encoder_left_rad_s', 0.0):.4f}",
            "Direction": "",  # not in DB; compute if you have sign columns
        },
        "EncoderR": {
            "Speed": f"{row.get('encoder_right_rad_s', 0.0):.4f}",
            "Direction": "",
        },
    }

    return result


# =========================
# PROCESS CONTROL HELPERS
# =========================
def is_proc_running(proc):
    return proc is not None and proc.poll() is None


def start_process(script_name):
    return subprocess.Popen(["python3", script_name], preexec_fn=os.setsid)


def stop_process(proc_ref_name):
    """
    proc_ref_name: string name of global variable, e.g., 'imu_proc'
    """
    global imu_proc, motor_test_proc, ml_proc, autonav_proc
    proc = globals().get(proc_ref_name)
    if is_proc_running(proc):
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        except Exception:
            pass
    globals()[proc_ref_name] = None


# IMU
def is_imu_running():
    global imu_proc
    return is_proc_running(imu_proc)


def start_imu():
    global imu_proc
    if not is_imu_running():
        imu_proc = start_process(IMU_SCRIPT)
        log_to_file("SYSTEM", "IMU Process started")
    return is_imu_running()


def stop_imu():
    stop_process("imu_proc")
    log_to_file("SYSTEM", "IMU Process stopped")


# Motor test
def is_motor_test_running():
    global motor_test_proc
    return is_proc_running(motor_test_proc)


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
        motor_test_proc = start_process(MOTOR_TEST_SCRIPT)
        log_to_file("SYSTEM", "Motor Test Process started")
    return is_motor_test_running()


def stop_motor_test():
    stop_process("motor_test_proc")
    log_to_file("SYSTEM", "Motor Test Process stopped")


# ML
def is_ml_running():
    global ml_proc
    return is_proc_running(ml_proc)


def start_ml():
    global ml_proc
    if not is_ml_running():
        ml_proc = start_process(ML_SCRIPT)
        log_to_file("SYSTEM", "ML Process started")
    return is_ml_running()


def stop_ml():
    stop_process("ml_proc")
    log_to_file("SYSTEM", "ML Process stopped")


# AutoNav
def is_autonav_running():
    global autonav_proc
    return is_proc_running(autonav_proc)


def start_autonav():
    global autonav_proc
    if not is_autonav_running():
        autonav_proc = start_process(AUTONAV_SCRIPT)
        log_to_file("SYSTEM", "AutoNav Process started")
    return is_autonav_running()


def stop_autonav():
    stop_process("autonav_proc")
    log_to_file("SYSTEM", "AutoNav Process stopped")


# =========================
# ROUTES
# =========================
@app.route("/sensor_feed")
def sensor_feed():
    """
    Instead of reading sensor_data.txt, pull the latest row from sensor_data.db
    and convert it to the JSON structure expected by the UI.
    """
    try:
        row = get_latest_reading()
        data = reading_to_sensor_dict(row)
        return jsonify(data)
    except Exception as e:
        print(f"Error reading from DB: {e}")
        return jsonify(reading_to_sensor_dict(None))


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
    # Left as placeholder; adjust to pull from DB if desired
    return jsonify({"value": 42})


@app.route("/direction_ajax", methods=["POST"])
def direction_ajax():
    global last_joystick_send
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


# ===== ML BUTTON ENDPOINTS =====
@app.route("/ml_on", methods=["POST"])
def ml_on():
    ok = start_ml()
    status = "started" if ok else "failed"
    return jsonify({"status": status})


@app.route("/ml_off", methods=["POST"])
def ml_off():
    stop_ml()
    return jsonify({"status": "stopped"})


# ===== AUTONAV BUTTON ENDPOINTS =====
@app.route("/autonav_on", methods=["POST"])
def autonav_on():
    ok = start_autonav()
    status = "started" if ok else "failed"
    return jsonify({"status": status})


@app.route("/autonav_off", methods=["POST"])
def autonav_off():
    stop_autonav()
    return jsonify({"status": "stopped"})


@app.route("/")
def home():
    log_to_file("SYSTEM", "Web interface loaded")
    # You can put your HTML here; kept as in original file
    return """
<!DOCTYPE html>
<html>
<head>
    <title>Robot Web Interface</title>
</head>
<body>
    <h1>Robot Web Interface</h1>
    <!-- Your existing HTML/JS goes here -->
</body>
</html>
"""


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000, debug=True)
