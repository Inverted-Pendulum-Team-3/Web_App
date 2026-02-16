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
                result["IMU1"][field] = tokens[idx+1]
            idx += 2

        if idx < len(tokens) and tokens[idx] == "IMU2":
            idx += 1
        for field in imu_fields:
            if idx + 1 < len(tokens) and tokens[idx] == field:
                result["IMU2"][field] = tokens[idx+1]
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
    sensor_value = 42
    log_to_file("SYSTEM", "Web interface loaded")
    return f"""
    <style>
      :root {{
        --panel: #101820;
        --accent: #29f0ff;
        --text: #d7f8ff; --warn: #ffdd57; --ok: #8dff8a; --bad: #ff5560;
        --bg: #0b0f14; --grid-gap: 10px;
      }}
      body {{ margin: 0; padding: 14px; background: var(--bg); color: var(--text); font-family: system-ui,sans-serif; }}
      h1 {{ font-size:1.1rem; margin-bottom:8px; color:var(--accent); text-shadow:0 0 8px #29f0ff55; }}
      .hub-grid {{ display: grid; grid-template-columns: 1.1fr 1.5fr 1.2fr; gap: var(--grid-gap); }}
      .panel {{ background:var(--panel); border-radius:9px; padding:11px; box-shadow:0 2px 12px #0008; margin-bottom:var(--grid-gap); }}
      .left-stack {{ display: grid; gap:var(--grid-gap); }}
      .joystick-wrapper {{ display:flex; flex-direction:column; align-items:center; gap:6px; }}
      #joystick-area {{
        position: relative;
        width: 180px;
        height: 180px;
        border-radius: 50%;
        margin-top: 4px;
        background: radial-gradient(circle at 30% 30%, #1e2b36, #05090e);
        border: 2px solid #18303c;
        box-shadow: 0 0 10px #000a;
        touch-action: none;
      }}
      #joystick-base {{
        position:absolute;
        left:50%; top:50%;
        width:140px; height:140px;
        margin-left:-70px; margin-top:-70px;
        border-radius:50%;
        border:1px dashed #2a4f60;
      }}
      #joystick-knob {{
        position:absolute;
        left:50%; top:50%;
        width:60px; height:60px;
        margin-left:-30px; margin-top:-30px;
        border-radius:50%;
        background: radial-gradient(circle at 30% 30%, #3cf0ff, #0e4f5b);
        box-shadow:0 0 14px #29f0ffaa;
      }}
      .joy-readouts {{
        display:flex;
        gap:8px;
        font-family:ui-monospace;
        font-size:12px;
      }}
      .joy-readouts span strong {{ color:var(--ok); }}
      .kv {{ border:1px dashed #18303c; border-radius:7px; padding:2px 7px; font-family:ui-monospace; font-size:12px; color:#aee9ef; margin-bottom:2px; min-width:0; line-height:1.3; }}
      .kv strong {{ color:var(--ok); font-weight:600; font-size:13px; }}
      .sensor-panel {{ margin-bottom:7px; }}
      .calib-countdown {{ font-weight:700; font-size:14px; margin-left:0.75em; }}
      .sensor-status strong#sensor-switch.ON {{ color:var(--ok); }}
      .sensor-status strong#sensor-switch.OFF {{ color:var(--bad); }}
      .sensor-status strong#sensor-switch.CALIB {{ color:var(--warn); }}
      .right-stack {{ display:grid;grid-template-rows:auto 1fr auto; gap:var(--grid-gap); }}
      .grid-3x3 {{ display:grid; gap:8px; grid-template-columns:repeat(3,1fr); }}
      .btn {{ background:#111; border:1px solid #18303c; border-radius:8px; color:var(--text); cursor:pointer; font-weight:600;
        text-transform:uppercase; font-size:13.5px; outline:none; box-shadow:0 2px 6px #0006; padding:6px 0; }}
      .btn:hover {{ border-color: var(--accent); background:#11262c; }}
      .btn:active {{ transform:scale(0.97); }}
      .btn.success {{ color:var(--ok); border-color:#229953; }}
      .btn.danger {{ color:var(--bad); border-color:#a8444e; }}
      .btn-placeholder {{ color:#7aaac0; border-style:dashed; font-size:12.5px; }}
      .robot-status-panel .kv, .sensor-panel .kv {{ margin-bottom:4px; }}
      .timer {{ font-family:ui-monospace; color:#b9f7ff; font-size:13px; padding:3px 7px; border-radius:7px; background:#0b1419; border:1px solid #18303c; }}
      .console {{ height:200px; background:#061016; border:1px solid #17303b; border-radius:7px; padding:8px 7px; overflow:auto;
        font-family:ui-monospace; font-size:13px; color:#b2f3ff; }}
      .console .line {{ margin-bottom:2px; }}
      .tag {{ display:inline-block; padding:0 7px; border-radius:3px; margin-right:5px; border:1px solid #234a54; color:#baf9ff; background:#141d20; font-size:12px; }}
      @media(max-width:910px){{ .hub-grid {{ grid-template-columns:1fr; }} }}
    </style>
    <h1>Control Hub</h1>
    <div class="hub-grid">
      <div class="panel left-stack">
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
      <div class="panel sensor-panel">
        <div class="panel-title">Sensor Feed</div>
        <div id="sensor-feed-panel">
          <div class="kv" style="display:flex; align-items:center;">
            <strong id="imu-time">Time</strong>
            <span class="calib-countdown" id="calib-status">Auto</span>
          </div>
          <div style="margin-bottom:6px;"><u>IMU1</u>
            <div class="kv">FB Tilt: <strong id="imu1-fb"></strong></div>
            <div class="kv">SS Tilt: <strong id="imu1-ss"></strong></div>
            <div class="kv">Yaw: <strong id="imu1-yaw"></strong></div>
            <div class="kv">Pitch Rate: <strong id="imu1-pitch"></strong></div>
            <div class="kv">Roll Rate: <strong id="imu1-roll"></strong></div>
            <div class="kv">Rot Vel: <strong id="imu1-rot"></strong></div>
          </div>
          <div style="margin-bottom:6px;"><u>IMU2</u>
            <div class="kv">FB Tilt: <strong id="imu2-fb"></strong></div>
            <div class="kv">SS Tilt: <strong id="imu2-ss"></strong></div>
            <div class="kv">Yaw: <strong id="imu2-yaw"></strong></div>
            <div class="kv">Pitch Rate: <strong id="imu2-pitch"></strong></div>
            <div class="kv">Roll Rate: <strong id="imu2-roll"></strong></div>
            <div class="kv">Rot Vel: <strong id="imu2-rot"></strong></div>
          </div>
          <div style="margin-bottom:6px;">
            <u>IMU1 Linear</u>
            <div class="kv">Linear Vel: <strong id="imu1linear-lv"></strong></div>
            <div class="kv">X velocity: <strong id="imu1linear-xv"></strong></div>
            <div class="kv">Y velocity: <strong id="imu1linear-yv"></strong></div>
          </div>
          <div style="margin-bottom:6px;">
            <u>EncoderL</u>
            <div class="kv">Speed: <strong id="enc-l-val"></strong></div>
            <div class="kv">Direction: <strong id="enc-l-status"></strong></div>
          </div>
          <div style="margin-bottom:6px;">
            <u>EncoderR</u>
            <div class="kv">Speed: <strong id="enc-r-val"></strong></div>
            <div class="kv">Direction: <strong id="enc-r-status"></strong></div>
          </div>
        </div>
        <div class="sensor-status">
          <span>Sensors: <strong id="sensor-switch" class="OFF">OFF</strong></span>
        </div>
      </div>
      <div class="right-stack">
        <div class="panel">
          <div class="panel-title">Action Panel</div>
          <div class="grid-3x3">
            <button class="btn danger" onclick="getTopRow()">Remove Row</button>
            <button class="btn success" onclick="toggleSensor(true)">Sensors ON</button>
            <button class="btn danger" onclick="toggleSensor(false)">Sensors OFF</button>
            <button class="btn" onclick="startMotorTest()">Start Motor</button>
            <button class="btn" onclick="stopMotorTest()">Stop Motor</button>
            <button class="btn btn-placeholder" disabled>Empty</button>
            <button class="btn btn-placeholder" disabled>Empty</button>
            <button class="btn btn-placeholder" disabled>Empty</button>
            <button class="btn" onclick="toggleRobot()">ON/OFF</button>
          </div>
        </div>
        <div class="panel robot-status-panel">
          <div class="panel-title">Robot Status</div>
          <div class="kv">Speed: <strong id="robot-speed">{sensor_value}</strong></div>
          <div class="robot-btns">
            <button class="btn" onclick="setSpeed('slow')">Slow</button>
            <button class="btn" onclick="setSpeed('med')">Medium</button>
            <button class="btn" onclick="setSpeed('fast')">Fast</button>
            <button class="btn" onclick="brakeRobot()">Brake</button>
          </div>
          <div class="kv">Braking: <strong id="robot-brake">OFF</strong></div>
          <div class="kv">Battery: <strong id="robot-battery">{sensor_value}</strong>%</div>
          <div class="kv">Status: <strong id="robot-switch">OFF</strong></div>
        </div>
        <div class="panel timer" id="timer">Task time: 0.00s</div>
        <div class="panel">
          <div class="panel-title">Event Console</div>
          <div class="console" id="console"></div>
        </div>
      </div>
    </div>
    <script>
      function getval(el, val) {{ document.getElementById(el).textContent = val || ""; }}
      function updateSensorFeed() {{
        fetch('/sensor_feed').then(r => r.json()).then(data => {{
          getval('imu-time', data.IMU1.Time);
          getval('imu1-fb', data.IMU1['Forward/backwards Tilt']);
          getval('imu1-ss', data.IMU1['Side-to-Side Tilt']);
          getval('imu1-yaw', data.IMU1['Yaw']);
          getval('imu1-pitch', data.IMU1['Pitch Rate']);
          getval('imu1-roll', data.IMU1['Roll Rate']);
          getval('imu1-rot', data.IMU1['Rotational Velocity']);

          getval('imu2-fb', data.IMU2['Forward/backwards Tilt']);
          getval('imu2-ss', data.IMU2['Side-to-Side Tilt']);
          getval('imu2-yaw', data.IMU2['Yaw']);
          getval('imu2-pitch', data.IMU2['Pitch Rate']);
          getval('imu2-roll', data.IMU2['Roll Rate']);
          getval('imu2-rot', data.IMU2['Rotational Velocity']);

          getval('imu1linear-lv', data.IMU1Linear['Linear Velocity']);
          getval('imu1linear-xv', data.IMU1Linear['X velocity']);
          getval('imu1linear-yv', data.IMU1Linear['Y velocity']);

          getval('enc-l-val', data.EncoderL.Speed);
          getval('enc-l-status', data.EncoderL.Direction);
          getval('enc-r-val', data.EncoderR.Speed);
          getval('enc-r-status', data.EncoderR.Direction);
        }});
      }}
      setInterval(updateSensorFeed, 250);
      updateSensorFeed();

      let calibTimer = null;
      function toggleSensor(on) {{
        let logMsg = on ? 'Sensors turned ON' : 'Sensors turned OFF';
        fetch(on ? '/sensor_on' : '/sensor_off', {{ method: 'POST' }})
          .then(r => r.json())
          .then(data => {{
            let s = document.getElementById('sensor-switch');
            if (on && data.status === 'ON') {{
              s.textContent = 'ON';
              s.className = 'CALIB';
              calibrateSensorCountdown(10, function() {{
                s.textContent = 'ON';
                s.className = 'ON';
              }});
              logLine('SENSOR', logMsg);
            }} else if (!on && data.status === 'OFF') {{
              s.textContent = 'OFF';
              s.className = 'OFF';
              document.getElementById('calib-status').textContent = 'Auto';
              if (calibTimer) clearInterval(calibTimer);
              logLine('SENSOR', logMsg);
            }} else {{
              logLine('SENSOR', 'Error toggling sensors');
            }}
          }});
      }}

      function calibrateSensorCountdown(seconds, donecb) {{
        let counter = seconds;
        document.getElementById('calib-status').textContent = 'Calibrating: ' + counter;
        let s = document.getElementById('sensor-switch');
        s.textContent = 'ON';
        s.className = 'CALIB';
        if (calibTimer) clearInterval(calibTimer);
        calibTimer = setInterval(function() {{
          counter--;
          document.getElementById('calib-status').textContent =
            counter > 0 ? 'Calibrating: ' + counter : 'Done';
          if (counter <= 0) {{
            clearInterval(calibTimer);
            if (donecb) donecb();
          }}
        }}, 1000);
      }}

      function setSpeed(mode) {{
        let val = mode == 'slow' ? 25 : mode == 'med' ? 42 : 88;
        document.getElementById('robot-speed').textContent = val;
        logLine('SPEED', 'Speed set to ' + mode.toUpperCase() + ' (' + val + ')');
      }}
      function brakeRobot() {{
        document.getElementById('robot-brake').textContent = 'ON';
        logLine('BRAKE', 'Robot Braking activated');
        setTimeout(function() {{
          document.getElementById('robot-brake').textContent = 'OFF';
        }}, 900);
      }}
      function toggleRobot() {{
        const el = document.getElementById('robot-switch');
        const newState = el.textContent == 'OFF' ? 'ON' : 'OFF';
        el.textContent = newState;
        logLine('ROBOT', 'Robot switched ' + newState);
      }}
      document.getElementById('robot-battery').textContent = {sensor_value};

      function getTopRow() {{
        logLine('ACTION', 'Remove Row button pressed');
        fetch('/get_top_row', {{ method: 'POST' }}).then(r => r.json());
      }}

      function startMotorTest() {{
        logLine('MOTOR', 'Starting Motor Test (clearing numbers.txt)');
        fetch('/start_motor_test', {{ method: 'POST' }})
          .then(r => r.json())
          .then(data => {{
            if (data.status === 'started') {{
              logLine('MOTOR', 'Motor Test started successfully');
            }} else {{
              logLine('MOTOR', 'Motor Test failed to start');
            }}
          }})
          .catch(err => {{
            logLine('ERR', 'Motor Test start error: ' + err);
          }});
      }}

      function stopMotorTest() {{
        logLine('MOTOR', 'Stopping Motor Test');
        fetch('/stop_motor_test', {{ method: 'POST' }})
          .then(r => r.json())
          .then(data => {{
            logLine('MOTOR', 'Motor Test stopped');
          }})
          .catch(err => {{
            logLine('ERR', 'Motor Test stop error: ' + err);
          }});
      }}

      function logLine(tag, text) {{
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
      }}

      // Joystick logic with 0.25s throttling for console logging
      const area = document.getElementById('joystick-area');
      const knob = document.getElementById('joystick-knob');
      let joyCenter = {{ x: area.clientWidth / 2, y: area.clientHeight / 2 }};
      let joyRadius = 70;
      let joyActive = false;
      let lastJoystickLog = 0;
      const joystickThrottleMs = 250; // 0.25 seconds in milliseconds

      function setKnob(x, y) {{
        knob.style.left = x + 'px';
        knob.style.top = y + 'px';
      }}

      setKnob(joyCenter.x - knob.clientWidth / 2, joyCenter.y - knob.clientHeight / 2);

      function sendJoystick(x, y) {{
        const now = Date.now();
        const shouldLog = now - lastJoystickLog >= joystickThrottleMs;
        
        if (shouldLog) {{
          lastJoystickLog = now;
        }}

        fetch('/direction_ajax', {{
          method: 'POST',
          headers: {{ 'Content-Type': 'application/json' }},
          body: JSON.stringify({{ x: x, y: y }})
        }})
        .then(r => r.json())
        .then(data => {{
          if (data && data.message && shouldLog) {{
            logLine('JOY', data.message);
          }}
        }})
        .catch(err => {{
          if (shouldLog) {{
            logLine('ERR', 'Joystick send failed: ' + err);
          }}
        }});
      }}

      function updateJoystick(clientX, clientY) {{
        const rect = area.getBoundingClientRect();
        const cx = clientX - rect.left;
        const cy = clientY - rect.top;
        let dx = cx - joyCenter.x;
        let dy = cy - joyCenter.y;
        const dist = Math.sqrt(dx * dx + dy * dy);
        if (dist > joyRadius) {{
          const scale = joyRadius / dist;
          dx *= scale;
          dy *= scale;
        }};
        const knobX = joyCenter.x + dx - knob.clientWidth / 2;
        const knobY = joyCenter.y + dy - knob.clientHeight / 2;
        setKnob(knobX, knobY);

        let normX = dx / joyRadius;
        let normY = -dy / joyRadius;

        if (normY < -1) normY = -1;
        if (normY > 1) normY = 1;
        if (normX > 0) {{
          normX = normX * 1.2;
        }}
        if (normX < -1) normX = -1;
        if (normX > 1.2) normX = 1.2;

        document.getElementById('joy-x').textContent = normX.toFixed(2);
        document.getElementById('joy-y').textContent = normY.toFixed(2);
        sendJoystick(normX, normY);
      }}

      function resetJoystick() {{
        setKnob(joyCenter.x - knob.clientWidth / 2, joyCenter.y - knob.clientHeight / 2);
        document.getElementById('joy-x').textContent = '0.00';
        document.getElementById('joy-y').textContent = '0.00';
        const now = Date.now();
        if (now - lastJoystickLog >= joystickThrottleMs) {{
          lastJoystickLog = now;
          sendJoystick(0, 0);
        }}
      }}

      area.addEventListener('mousedown', function(e) {{
        joyActive = true;
        updateJoystick(e.clientX, e.clientY);
      }});
      window.addEventListener('mousemove', function(e) {{
        if (joyActive) updateJoystick(e.clientX, e.clientY);
      }});
      window.addEventListener('mouseup', function() {{
        if (joyActive) {{
          joyActive = false;
          resetJoystick();
        }}
      }});

      area.addEventListener('touchstart', function(e) {{
        e.preventDefault();
        joyActive = true;
        const t = e.touches[0];
        updateJoystick(t.clientX, t.clientY);
      }}, {{ passive: false }});
      area.addEventListener('touchmove', function(e) {{
        e.preventDefault();
        if (!joyActive) return;
        const t = e.touches[0];
        updateJoystick(t.clientX, t.clientY);
      }}, {{ passive: false }});
      area.addEventListener('touchend', function(e) {{
        e.preventDefault();
        if (joyActive) {{
          joyActive = false;
          resetJoystick();
        }}
      }}, {{ passive: false }});
    </script>
    """

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000, debug=True)
