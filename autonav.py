#!/usr/bin/env python3
"""
autonav.py — Autonomous wall-avoidance navigation for the self-balancing robot.

Runs ON TOP of the PID balance controller (deployPID_3_28.py must already be
running and keeping the robot upright). This script ONLY writes forward/turn
velocity targets to a shared command file that deployPID reads, so balance is
never compromised.

Architecture:
  - deployPID_3_28.py  → balance loop at 100 Hz (unchanged)
  - autonav.py         → navigation loop at ~20 Hz, writes autonav_cmd.json
  - deployPID reads autonav_cmd.json every 10 loops and uses CMD_FORWARD/CMD_TURN

Ultrasonic sensors (via ultrasonic_cache.bin written by ultrasonic_bg.py):
  Right sensor: forward-right facing
  Left  sensor: forward-left facing

Wall avoidance logic:
  1. Drive forward at cruise speed
  2. If either sensor detects obstacle within WARN_CM  → slow down
  3. If either sensor detects obstacle within STOP_CM  → stop forward, turn away
  4. If both sensors blocked                           → back up briefly then turn

Run:
  Terminal 1: python3 sensors_3_28.py
  Terminal 2: python3 ultrasonic_bg.py
  Terminal 3: python3 deployPID_3_28.py
  Terminal 4: python3 autonav.py
"""

import os
import sys
import time
import json
import struct
import math
from datetime import datetime

# ---------------------------------------------------------------------------
# File paths
# ---------------------------------------------------------------------------
_SCRIPT_DIR         = os.path.dirname(os.path.abspath(__file__))
ULTRASONIC_CACHE    = os.path.join(_SCRIPT_DIR, "ultrasonic_cache.bin")
AUTONAV_CMD_FILE    = os.path.join(_SCRIPT_DIR, "autonav_cmd.json")

# ---------------------------------------------------------------------------
# Navigation parameters — tune after achieving stable balance
# ---------------------------------------------------------------------------
LOOP_HZ             = 20        # autonav update rate
US_MAX_AGE_S        = 0.3       # ignore ultrasonic reading older than this

CRUISE_FWD          = 0.4       # forward command during free driving  (0–1)
WARN_CM             = 60.0      # slow down when obstacle within this range (cm)
STOP_CM             = 30.0      # stop & turn when obstacle within this range (cm)
SLOW_FWD            = 0.15      # reduced forward speed in warn zone

TURN_CMD            = 0.5       # turn command magnitude during avoidance (0–1)
TURN_DURATION_S     = 0.8       # how long to hold a turn before re-evaluating
BACKUP_FWD          = -0.2      # backward command when both sensors blocked
BACKUP_DURATION_S   = 0.5       # how long to back up before turning

CMD_STALE_S         = 0.5       # deployPID stops using autonav_cmd if older

# ---------------------------------------------------------------------------
# State machine
# ---------------------------------------------------------------------------
STATE_DRIVE   = "DRIVE"
STATE_WARN    = "WARN"
STATE_TURN    = "TURN"
STATE_BACKUP  = "BACKUP"
STATE_STOPPED = "STOPPED"       # obstacle on both sides with no clear path


def read_ultrasonic():
    """Read ultrasonic_cache.bin. Returns (right_cm, left_cm) or (None, None)."""
    try:
        with open(ULTRASONIC_CACHE, "rb") as f:
            raw = f.read()
        if len(raw) < 16:
            return None, None
        ts,          = struct.unpack("<d",  raw[0:8])
        right, left  = struct.unpack("<ff", raw[8:16])
        if time.monotonic() - ts > US_MAX_AGE_S:
            return None, None
        right = float(right) if right >= 0 else None
        left  = float(left)  if left  >= 0 else None
        return right, left
    except Exception:
        return None, None


def write_cmd(fwd: float, turn: float) -> None:
    """
    Write forward/turn targets to autonav_cmd.json.
    deployPID_3_28.py reads this and uses it as CMD_FORWARD / CMD_TURN.
    """
    try:
        data = {
            "fwd":       round(float(fwd),  3),
            "turn":      round(float(turn), 3),
            "timestamp": time.monotonic(),
        }
        tmp = AUTONAV_CMD_FILE + ".tmp"
        with open(tmp, "w") as f:
            json.dump(data, f)
        os.replace(tmp, AUTONAV_CMD_FILE)
    except Exception as e:
        print(f"[autonav] Failed to write cmd: {e}")


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def main():
    target_dt   = 1.0 / LOOP_HZ
    state       = STATE_DRIVE
    state_until = 0.0          # monotonic time when current timed state ends
    turn_dir    = 1.0          # +1 = turn right, -1 = turn left
    loop_count  = 0

    print("=" * 55)
    print("  autonav.py — Wall Avoidance")
    print(f"  Cruise: {CRUISE_FWD}  Warn: {WARN_CM} cm  Stop: {STOP_CM} cm")
    print("  Ctrl+C to stop")
    print("=" * 55)
    print()

    try:
        while True:
            t_start = time.monotonic()
            now     = t_start
            loop_count += 1

            right_cm, left_cm = read_ultrasonic()

            # Treat missing readings as "clear" (sensor offline) so robot
            # doesn't get stuck if ultrasonic_bg.py isn't running.
            right = right_cm if right_cm is not None else 999.0
            left  = left_cm  if left_cm  is not None else 999.0

            right_blocked = right < STOP_CM
            left_blocked  = left  < STOP_CM
            right_warn    = right < WARN_CM
            left_warn     = left  < WARN_CM
            both_blocked  = right_blocked and left_blocked

            # ----------------------------------------------------------------
            # State transitions
            # ----------------------------------------------------------------
            if state in (STATE_DRIVE, STATE_WARN):
                if both_blocked:
                    state       = STATE_BACKUP
                    state_until = now + BACKUP_DURATION_S
                    # After backup, turn toward the less-blocked side
                    turn_dir    = 1.0 if right > left else -1.0
                    print(f"[{loop_count:5d}] BACKUP  R={right:.0f} L={left:.0f} cm")

                elif right_blocked or left_blocked:
                    # Turn away from the blocked side
                    turn_dir    = -1.0 if right_blocked else 1.0
                    state       = STATE_TURN
                    state_until = now + TURN_DURATION_S
                    print(f"[{loop_count:5d}] TURN({'R' if turn_dir > 0 else 'L'})  "
                          f"R={right:.0f} L={left:.0f} cm")

                elif right_warn or left_warn:
                    if state != STATE_WARN:
                        print(f"[{loop_count:5d}] WARN    R={right:.0f} L={left:.0f} cm")
                    state = STATE_WARN

                else:
                    if state != STATE_DRIVE:
                        print(f"[{loop_count:5d}] DRIVE   R={right:.0f} L={left:.0f} cm")
                    state = STATE_DRIVE

            elif state == STATE_TURN:
                if now >= state_until:
                    state = STATE_DRIVE
                    print(f"[{loop_count:5d}] DRIVE   (turn complete)")

            elif state == STATE_BACKUP:
                if now >= state_until:
                    state       = STATE_TURN
                    state_until = now + TURN_DURATION_S
                    print(f"[{loop_count:5d}] TURN({'R' if turn_dir > 0 else 'L'})  "
                          f"(after backup)")

            # ----------------------------------------------------------------
            # Output commands based on current state
            # ----------------------------------------------------------------
            if state == STATE_DRIVE:
                fwd, turn = CRUISE_FWD, 0.0

            elif state == STATE_WARN:
                # Slow down; bias turn away from closer side
                bias = clamp((left - right) / WARN_CM, -0.3, 0.3)
                fwd, turn = SLOW_FWD, bias

            elif state == STATE_TURN:
                fwd, turn = 0.0, TURN_CMD * turn_dir

            elif state == STATE_BACKUP:
                fwd, turn = BACKUP_FWD, 0.0

            else:
                fwd, turn = 0.0, 0.0

            write_cmd(fwd, turn)

            sys.stdout.write(
                f"\r[{loop_count:5d}] {state:<8s}  "
                f"R={right_cm if right_cm is not None else '---':>5}  "
                f"L={left_cm  if left_cm  is not None else '---':>5}  "
                f"fwd={fwd:+.2f}  turn={turn:+.2f}   "
            )
            sys.stdout.flush()

            elapsed = time.monotonic() - t_start
            sleep_t = target_dt - elapsed
            if sleep_t > 0:
                time.sleep(sleep_t)

    except KeyboardInterrupt:
        print(f"\nStopped after {loop_count} loops.")

    finally:
        # Zero commands on exit so PID controller returns to stationary balance
        write_cmd(0.0, 0.0)
        print("autonav stopped — commands zeroed.")


if __name__ == "__main__":
    main()
