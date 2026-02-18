from flask import Flask, jsonify, request
import subprocess
import os
import signal
from datetime import datetime

app = Flask(__name__)
imu_proc = None
motor_test_proc = None

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


def is_imu_running():
    global imu_proc
    return imu_proc is not None and imu_proc.poll() is None


def start_imu():
    global imu_proc
    if not is_imu_running():
        imu_proc = subprocess.Popen(['python3', 'imuencoderv4.py'], preexec_fn=os.setsid)
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
        motor_test_proc = subprocess.Popen(['python3', 'motortest.py'], preexec_fn=os.setsid)
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


@app.route("/")
def home():
    log_to_file("SYSTEM", "Web interface loaded")
    return """
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
        display:flex;
        align-items:center;
        justify-content:space-between;
        gap:8px;
      }
      .top-row-left, .top-row-right,
      .bottom-row-left, .bottom-row-right {
        display:flex;
        align-items:center;
        gap:8px;
      }
      .top-title {
        font-weight:600;
        letter-spacing:0.03em;
        color: var(--accent);
        text-shadow:0 0 8px #29f0ff55;
      }
      .top-btn {
        background:#101820;
        border-radius:7px;
        border:1px solid #18303c;
        padding:4px 8px;
        font-size:12px;
        color:var(--text);
        cursor:pointer;
        text-transform:uppercase;
        font-weight:600;
      }
      .top-btn:hover {
        border-color: var(--accent);
        background:#11262c;
      }
      .top-btn.danger { color:var(--bad); border-color:#a8444e; }
      .top-btn.success { color:var(--ok); border-color:#229953; }
      .top-btn.neutral { color:var(--warn); border-color:#bfa23a; }
      .top-status {
        font-size:12px;
        font-family:ui-monospace;
      }
      .top-status span strong.ON { color:var(--ok); }
      .top-status span strong.OFF { color:var(--bad); }
      .top-status span strong.CALIB { color:var(--warn); }

      .main-layout {
        flex:1;
        padding: 10px;
        display:flex;
        gap: var(--grid-gap);
        box-sizing:border-box;
      }
      .panel {
        background:var(--panel);
        border-radius:9px;
        padding:11px;
        box-shadow:0 2px 12px #0008;
        display:flex;
        flex-direction:column;
        min-width:0;
      }
      .col-left, .col-center, .col-right {
        flex:1;
        min-width:0;
        display:flex;
        flex-direction:column;
        gap: var(--grid-gap);
      }
      .panel-title {
        font-size:0.9rem;
        margin-bottom:6px;
        color:var(--accent);
      }
      .joystick-wrapper {
        display:flex;
        flex-direction:column;
        align-items:center;
        gap:6px;
      }
      #joystick-area {
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
      #joystick-base {
        position:absolute;
        left:50%; top:50%;
        width:140px; height:140px;
        margin-left:-70px; margin-top:-70px;
        border-radius:50%;
        border:1px dashed #2a4f60;
      }
      #joystick-knob {
        position:absolute;
        left:50%; top:50%;
        width:60px; height:60px;
        margin-left:-30px; margin-top:-30px;
        border-radius:50%;
        background: radial-gradient(circle at 30% 30%, #3cf0ff, #0e4f5b);
        box-shadow:0 0 14px #29f0ffaa;
      }
      .joy-readouts {
        display:flex;
        gap:8px;
        font-family:ui-monospace;
        font-size:12px;
      }
      .joy-readouts span strong { color:var(--ok); }

      /* console column constrained to same height as joystick/sensor columns */
      .col-center {
        max-height: 100%;
      }
      .panel.console-panel {
        flex: 1 1 auto;
        max-height: 100%;
      }
      .console {
        flex:1 1 auto;
        background:#061016;
        border:1px solid #17303b;
        border-radius:7px;
        padding:8px 7px;
        overflow:auto;
        font-family:ui-monospace;
        font-size:13px;
        color:#b2f3ff;
        max-height: 100%;
      }
      .console .line {
        margin-bottom:2px;
      }
      .tag {
        display:inline-block;
        padding:0 7px;
        border-radius:3px;
        margin-right:5px;
        border:1px solid #234a54;
        color:#baf9ff;
        background:#141d20;
        font-size:12px;
      }

      .kv {
        border:1px dashed #18303c;
        border-radius:7px;
        padding:2px 7px;
        font-family:ui-monospace;
        font-size:12px;
        color:#aee9ef;
        margin-bottom:2px;
        min-width:0;
        line-height:1.3;
        display:flex;
        justify-content:space-between;
      }
      .kv strong { color:var(--ok); font-weight:600; font-size:13px; }
      .sensor-section-title {
        margin-top:4px;
        margin-bottom:2px;
        text-decoration:underline;
      }
      .sensor-avg {
        font-size:11px;
        color:#c2f2ff;
        margin-left:6px;
      }

      .sensor-status-line {
        margin-top:4px;
        font-size:12px;
      }
      .calib-countdown {
        font-weight:700;
        font-size:13px;
        margin-left:0.75em;
      }

      @media(max-width: 960px) {
        .main-layout {
          flex-direction:column;
        }
      }
    </style>

    <div class="top-bar">
      <div class="top-row">
        <div class="top-row-left">
          <div class="top-title">Control Hub</div>
        </div>
        <div class="top-row-right">
          <!-- ON row -->
          <button class="top-btn success" onclick="startMotorTest()">Motors ON</button>
          <button class="top-btn success" onclick="toggleSensor(true)">Sensors ON</button>
          <button class="top-btn neutral" onclick="toggleML(true)">ML ON</button>
          <button class="top-btn neutral" onclick="toggleAutonomous(true)">Auto Nav ON</button>
        </div>
      </div>
      <div class="bottom-row">
        <div class="bottom-row-left">
          <span class="top-status">
            <span>Sens: <strong id="sensor-switch" class="OFF">OFF</strong></span>
            &nbsp;|&nbsp;
            <span>Calib: <span class="calib-countdown" id="calib-status">Auto</span></span>
          </span>
        </div>
        <div class="bottom-row-right">
          <!-- OFF row -->
          <button class="top-btn danger" onclick="stopMotorTest()">Motors OFF</button>
          <button class="top-btn danger" onclick="toggleSensor(false)">Sensors OFF</button>
          <button class="top-btn danger" onclick="toggleML(false)">ML OFF</button>
          <button class="top-btn danger" onclick="toggleAutonomous(false)">Auto Nav OFF</button>
        </div>
      </div>
    </div>

    <div class="main-layout">
      <div class="col-left">
        <div class="panel">
          <div class="panel-title">Joystick Input</div>
          <div class="joystick-wrapper">
            <div id="joystick-area">
              <div id="joystick-base"></div>
              <div id="joystick-knob"></div>
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
          <div class="panel-title">Sensor Feed (with 3s Averages)</div>
          <div id="sensor-feed-panel">
            <div class="kv">
              <span><strong id="imu-time">Time</strong></span>
              <span class="sensor-avg" id="imu-time-avg"></span>
            </div>
            <div class="sensor-section-title">IMU1</div>
            <div class="kv">FB Tilt:
              <strong id="imu1-fb"></strong>
              <span class="sensor-avg" id="imu1-fb-avg"></span>
            </div>
            <div class="kv">SS Tilt:
              <strong id="imu1-ss"></strong>
              <span class="sensor-avg" id="imu1-ss-avg"></span>
            </div>
            <div class="kv">Yaw:
              <strong id="imu1-yaw"></strong>
              <span class="sensor-avg" id="imu1-yaw-avg"></span>
            </div>
            <div class="kv">Pitch Rate:
              <strong id="imu1-pitch"></strong>
              <span class="sensor-avg" id="imu1-pitch-avg"></span>
            </div>
            <div class="kv">Roll Rate:
              <strong id="imu1-roll"></strong>
              <span class="sensor-avg" id="imu1-roll-avg"></span>
            </div>
            <div class="kv">Rot Vel:
              <strong id="imu1-rot"></strong>
              <span class="sensor-avg" id="imu1-rot-avg"></span>
            </div>

            <div class="sensor-section-title">IMU2</div>
            <div class="kv">FB Tilt:
              <strong id="imu2-fb"></strong>
              <span class="sensor-avg" id="imu2-fb-avg"></span>
            </div>
            <div class="kv">SS Tilt:
              <strong id="imu2-ss"></strong>
              <span class="sensor-avg" id="imu2-ss-avg"></span>
            </div>
            <div class="kv">Yaw:
              <strong id="imu2-yaw"></strong>
              <span class="sensor-avg" id="imu2-yaw-avg"></span>
            </div>
            <div class="kv">Pitch Rate:
              <strong id="imu2-pitch"></strong>
              <span class="sensor-avg" id="imu2-pitch-avg"></span>
            </div>
            <div class="kv">Roll Rate:
              <strong id="imu2-roll"></strong>
              <span class="sensor-avg" id="imu2-roll-avg"></span>
            </div>
            <div class="kv">Rot Vel:
              <strong id="imu2-rot"></strong>
              <span class="sensor-avg" id="imu2-rot-avg"></span>
            </div>

            <div class="sensor-section-title">IMU1 Linear</div>
            <div class="kv">Linear Vel:
              <strong id="imu1linear-lv"></strong>
              <span class="sensor-avg" id="imu1linear-lv-avg"></span>
            </div>
            <div class="kv">X velocity:
              <strong id="imu1linear-xv"></strong>
              <span class="sensor-avg" id="imu1linear-xv-avg"></span>
            </div>
            <div class="kv">Y velocity:
              <strong id="imu1linear-yv"></strong>
              <span class="sensor-avg" id="imu1linear-yv-avg"></span>
            </div>

            <div class="sensor-section-title">EncoderL</div>
            <div class="kv">Speed:
              <strong id="enc-l-val"></strong>
              <span class="sensor-avg" id="enc-l-val-avg"></span>
            </div>
            <div class="kv">Direction:
              <strong id="enc-l-status"></strong>
              <span class="sensor-avg" id="enc-l-status-avg"></span>
            </div>

            <div class="sensor-section-title">EncoderR</div>
            <div class="kv">Speed:
              <strong id="enc-r-val"></strong>
              <span class="sensor-avg" id="enc-r-val-avg"></span>
            </div>
            <div class="kv">Direction:
              <strong id="enc-r-status"></strong>
              <span class="sensor-avg" id="enc-r-status-avg"></span>
            </div>
          </div>
          <div class="sensor-status-line">
            (Sensor state shown in top bar)
          </div>
        </div>
      </div>
    </div>

    <script>
      function getval(el, val) { document.getElementById(el).textContent = val || ""; }

      // Rolling window storage for last ~3 seconds (3s / 0.25s = 12 samples)
      const SENSOR_HISTORY_WINDOW = 3.0; // seconds
      const SENSOR_POLL_MS = 250;
      const SENSOR_MAX_SAMPLES = Math.ceil((SENSOR_HISTORY_WINDOW * 1000) / SENSOR_POLL_MS);

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
        "enc-r-val": ["EncoderR", "Speed"]
      };

      const sensorHistory = {};
      Object.keys(sensorNumericKeys).forEach(k => sensorHistory[k] = []);

      function addSample(key, value) {
        const arr = sensorHistory[key];
        const num = parseFloat(value);
        if (!isNaN(num)) {
          arr.push(num);
          if (arr.length > SENSOR_MAX_SAMPLES) {
            arr.shift();
          }
        }
      }

      function computeAvg(arr) {
        if (!arr || arr.length === 0) return "";
        let sum = 0;
        for (let v of arr) sum += v;
        const avg = sum / arr.length;
        return avg.toFixed(3);
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
        fetch('/sensor_feed').then(r => r.json()).then(data => {
          // Raw values
          getval('imu-time', data.IMU1.Time);

          const v_imu1_fb = data.IMU1['Forward/backwards Tilt'];
          const v_imu1_ss = data.IMU1['Side-to-Side Tilt'];
          const v_imu1_yaw = data.IMU1['Yaw'];
          const v_imu1_pitch = data.IMU1['Pitch Rate'];
          const v_imu1_roll = data.IMU1['Roll Rate'];
          const v_imu1_rot = data.IMU1['Rotational Velocity'];

          const v_imu2_fb = data.IMU2['Forward/backwards Tilt'];
          const v_imu2_ss = data.IMU2['Side-to-Side Tilt'];
          const v_imu2_yaw = data.IMU2['Yaw'];
          const v_imu2_pitch = data.IMU2['Pitch Rate'];
          const v_imu2_roll = data.IMU2['Roll Rate'];
          const v_imu2_rot = data.IMU2['Rotational Velocity'];

          const v_lv  = data.IMU1Linear['Linear Velocity'];
          const v_xv  = data.IMU1Linear['X velocity'];
          const v_yv  = data.IMU1Linear['Y velocity'];

          const v_enc_l_spd = data.EncoderL.Speed;
          const v_enc_l_dir = data.EncoderL.Direction;
          const v_enc_r_spd = data.EncoderR.Speed;
          const v_enc_r_dir = data.EncoderR.Direction;

          getval('imu1-fb', v_imu1_fb);
          getval('imu1-ss', v_imu1_ss);
          getval('imu1-yaw', v_imu1_yaw);
          getval('imu1-pitch', v_imu1_pitch);
          getval('imu1-roll', v_imu1_roll);
          getval('imu1-rot', v_imu1_rot);

          getval('imu2-fb', v_imu2_fb);
          getval('imu2-ss', v_imu2_ss);
          getval('imu2-yaw', v_imu2_yaw);
          getval('imu2-pitch', v_imu2_pitch);
          getval('imu2-roll', v_imu2_roll);
          getval('imu2-rot', v_imu2_rot);

          getval('imu1linear-lv', v_lv);
          getval('imu1linear-xv', v_xv);
          getval('imu1linear-yv', v_yv);

          getval('enc-l-val', v_enc_l_spd);
          getval('enc-l-status', v_enc_l_dir);
          getval('enc-r-val', v_enc_r_spd);
          getval('enc-r-status', v_enc_r_dir);

          addSample('imu1-fb', v_imu1_fb);
          addSample('imu1-ss', v_imu1_ss);
          addSample('imu1-yaw', v_imu1_yaw);
          addSample('imu1-pitch', v_imu1_pitch);
          addSample('imu1-roll', v_imu1_roll);
          addSample('imu1-rot', v_imu1_rot);

          addSample('imu2-fb', v_imu2_fb);
          addSample('imu2-ss', v_imu2_ss);
          addSample('imu2-yaw', v_imu2_yaw);
          addSample('imu2-pitch', v_imu2_pitch);
          addSample('imu2-roll', v_imu2_roll);
          addSample('imu2-rot', v_imu2_rot);

          addSample('imu1linear-lv', v_lv);
          addSample('imu1linear-xv', v_xv);
          addSample('imu1linear-yv', v_yv);

          addSample('enc-l-val', v_enc_l_spd);
          addSample('enc-r-val', v_enc_r_spd);

          updateAverages();
        });
      }

      setInterval(updateSensorFeed, SENSOR_POLL_MS);
      updateSensorFeed();

      let calibTimer = null;
      function toggleSensor(on) {
        let logMsg = on ? 'Sensors turned ON' : 'Sensors turned OFF';
        fetch(on ? '/sensor_on' : '/sensor_off', { method: 'POST' })
          .then(r => r.json())
          .then(data => {
            let s = document.getElementById('sensor-switch');
            if (on && data.status === 'ON') {
              s.textContent = 'ON';
              s.className = 'CALIB';
              startCalibrationCountdown(10, function() {
                s.textContent = 'ON';
                s.className = 'ON';
              });
              logLine('SENSOR', logMsg);
            } else if (!on && data.status === 'OFF') {
              s.textContent = 'OFF';
              s.className = 'OFF';
              document.getElementById('calib-status').textContent = 'Auto';
              if (calibTimer) clearInterval(calibTimer);
              logLine('SENSOR', logMsg);
            } else {
              logLine('SENSOR', 'Error toggling sensors');
            }
          });
      }

      function startCalibrationCountdown(seconds, donecb) {
        let counter = seconds;
        document.getElementById('calib-status').textContent = 'Calibrating: ' + counter;
        let s = document.getElementById('sensor-switch');
        s.textContent = 'ON';
        s.className = 'CALIB';
        if (calibTimer) clearInterval(calibTimer);
        calibTimer = setInterval(function() {
          counter--;
          document.getElementById('calib-status').textContent =
            counter > 0 ? 'Calibrating: ' + counter : 'Done';
          if (counter <= 0) {
            clearInterval(calibTimer);
            if (donecb) donecb();
          }
        }, 1000);
      }

      function startMotorTest() {
        logLine('MOTOR', 'Starting Motor Test (clearing numbers.txt)');
        fetch('/start_motor_test', { method: 'POST' })
          .then(r => r.json())
          .then(data => {
            if (data.status === 'started') {
              logLine('MOTOR', 'Motor Test started successfully');
            } else {
              logLine('MOTOR', 'Motor Test failed to start');
            }
          })
          .catch(err => {
            logLine('ERR', 'Motor Test start error: ' + err);
          });
      }

      function stopMotorTest() {
        logLine('MOTOR', 'Stopping Motor Test');
        fetch('/stop_motor_test', { method: 'POST' })
          .then(r => r.json())
          .then(data => {
            logLine('MOTOR', 'Motor Test stopped');
          })
          .catch(err => {
            logLine('ERR', 'Motor Test stop error: ' + err);
          });
      }

      function toggleML(on) {
        // Placeholder for future ML handling
        const state = on ? 'ON' : 'OFF';
        logLine('ML', 'ML ' + state + ' (future use)');
      }

      function toggleAutonomous(on) {
        // Placeholder for future autonomous nav handling
        const state = on ? 'ON' : 'OFF';
        logLine('AUTO', 'Autonomous Nav ' + state + ' (future use)');
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

      // Joystick logic with 0.25s throttling for console logging
      const area = document.getElementById('joystick-area');
      const knob = document.getElementById('joystick-knob');
      let joyCenter = { x: area.clientWidth / 2, y: area.clientHeight / 2 };
      let joyRadius = 70;
      let joyActive = false;
      let lastJoystickLog = 0;
      const joystickThrottleMs = 250; // 0.25 seconds in milliseconds

      function setKnob(x, y) {
        knob.style.left = x + 'px';
        knob.style.top = y + 'px';
      }

      setKnob(joyCenter.x - knob.clientWidth / 2, joyCenter.y - knob.clientHeight / 2);

      function sendJoystick(x, y) {
        const now = Date.now();
        const shouldLog = now - lastJoystickLog >= joystickThrottleMs;

        if (shouldLog) {
          lastJoystickLog = now;
        }

        fetch('/direction_ajax', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ x: x, y: y })
        })
        .then(r => r.json())
        .then(data => {
          if (data && data.message && shouldLog) {
            logLine('JOY', data.message);
          }
        })
        .catch(err => {
          if (shouldLog) {
            logLine('ERR', 'Joystick send failed: ' + err);
          }
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
        if (normX > 0) {
          normX = normX * 1.2;
        }
        if (normX < -1) normX = -1;
        if (normX > 1.2) normX = 1.2;

        document.getElementById('joy-x').textContent = normX.toFixed(2);
        document.getElementById('joy-y').textContent = normY.toFixed(2);
        sendJoystick(normX, normY);
      }

      function resetJoystick() {
        setKnob(joyCenter.x - knob.clientWidth / 2, joyCenter.y - knob.clientHeight / 2);
        document.getElementById('joy-x').textContent = '0.00';
        document.getElementById('joy-y').textContent = '0.00';
        const now = Date.now();
        if (now - lastJoystickLog >= joystickThrottleMs) {
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
    """


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000, debug=True)
