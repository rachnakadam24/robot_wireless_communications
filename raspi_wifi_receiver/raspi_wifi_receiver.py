#!/usr/bin/env python3
"""
ESP32 -> Raspberry Pi Servo client
- Auto-reconnect on any error or missed heartbeats
- Non-blocking socket with select()
- TCP keepalive enabled
- Watchdog: auto-stop wheels if no valid command within N seconds
- Robust JSON parsing (extracts { ... } even from noisy lines)
- ID filtering for control: applies only if obj["id"] matches this robot's ID,
  or list contains it, or obj["broadcast"] is true.
- Heartbeat is NOT robot-specific: any {"type":"heartbeat"} resets the HB timer.
- Debug prints for RAW/JSON/HB/ACT/SKIP/IGN/ERR/WARN
"""

import os, sys, time, json, socket, select, signal, re, argparse
from adafruit_servokit import ServoKit

# ========= Defaults =========
DEFAULT_HOST = "192.168.4.1"
DEFAULT_PORT = 9000
DEFAULT_ID   = int(os.environ.get("ROBOT_ID", "1"))  # override with --id or env ROBOT_ID

# ========= Heartbeat policy (robot-agnostic) =========
HB_INTERVAL = 2.0     # seconds; ESP32 should emit every 5s
HB_MISS     = 3       # reconnect if this many are missed
HB_GRACE    = 2.0     # extra grace seconds to avoid false positives
HB_TIMEOUT  = HB_INTERVAL * HB_MISS + HB_GRACE  # 17s by default

# ========= Servo HAT settings =========
KIT = ServoKit(channels=16, address=0x40)

# Channel mapping (change if needed)
CH_GRIPPER = 0           # standard servo
CH_RIGHT   = 1           # continuous
CH_LEFT    = 2           # continuous

# Pulse ranges (tune for your hardware)
KIT.servo[CH_GRIPPER].set_pulse_width_range(500, 2500)
KIT.continuous_servo[CH_LEFT].set_pulse_width_range(1000, 2000)
KIT.continuous_servo[CH_RIGHT].set_pulse_width_range(1000, 2000)

GRIPPER_OPEN_ANGLE  = 0
GRIPPER_CLOSE_ANGLE = 160

WATCHDOG_SECONDS = 2.5   # stop wheels if no valid command within this time

# ---------- utils ----------
def deg_to_throttle(v):
    """Map 0..180 to -1..+1 with 90=stop."""
    val = float(v)
    thr = (val - 90.0) / 90.0
    return max(-1.0, min(1.0, thr))

def set_drive(left=None, right=None):
    if left is not None:
        KIT.continuous_servo[CH_LEFT].throttle = float(left)
    if right is not None:
        KIT.continuous_servo[CH_RIGHT].throttle = float(right)

def set_gripper(cmd):
    if isinstance(cmd, str):
        c = cmd.strip().lower()
        if c == "open":
            KIT.servo[CH_GRIPPER].angle = GRIPPER_OPEN_ANGLE
            return "open", GRIPPER_OPEN_ANGLE
        if c == "close":
            KIT.servo[CH_GRIPPER].angle = GRIPPER_CLOSE_ANGLE
            return "close", GRIPPER_CLOSE_ANGLE
        # else try numeric-in-string
        cmd = float(cmd)
    angle = max(0, min(180, float(cmd)))
    KIT.servo[CH_GRIPPER].angle = angle
    return "angle", angle

def stop_motors():
    KIT.continuous_servo[CH_LEFT].throttle = 0.0
    KIT.continuous_servo[CH_RIGHT].throttle = 0.0

def cleanup_and_exit(signum=None, frame=None):
    try:
        stop_motors()
    finally:
        sys.exit(0)

signal.signal(signal.SIGINT, cleanup_and_exit)
signal.signal(signal.SIGTERM, cleanup_and_exit)

# Extract a JSON object from a noisy line
brace_re = re.compile(r'\{.*\}')

def extract_json(line: str):
    if line.startswith("{") and line.endswith("}"):
        return line
    m = brace_re.search(line)
    return m.group(0) if m else None

# ---------- networking ----------
def set_tcp_keepalive(sock):
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
    try:
        # Linux-specific (ignore errors elsewhere)
        sock.setsockopt(socket.IPPROTO_TCP, 0x10, 60)  # TCP_KEEPIDLE
        sock.setsockopt(socket.IPPROTO_TCP, 0x12, 10)  # TCP_KEEPINTVL
        sock.setsockopt(socket.IPPROTO_TCP, 0x11, 3)   # TCP_KEEPCNT
    except Exception:
        pass

def connect(host, port, timeout=5.0):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    set_tcp_keepalive(s)
    s.settimeout(timeout)      # connect timeout only
    s.connect((host, port))
    s.settimeout(0.0)          # non-blocking for select/recv
    print(f"[INFO] Connected to {host}:{port}")
    return s

def should_apply(obj, my_id):
    """
    Apply control command if:
      - obj["broadcast"] is True
      - or obj["id"] == my_id (int/str)
      - or obj["id"] is a list containing my_id (int/str)
    Heartbeats are handled separately and are NOT filtered by id.
    """
    if not isinstance(obj, dict):
        return False, "obj-not-dict"

    if obj.get("broadcast") is True:
        return True, "broadcast"

    if "id" not in obj:
        return False, "missing-id"

    rid = obj["id"]

    # int or float
    if isinstance(rid, (int, float)):
        return (int(rid) == int(my_id)), f"id={rid} vs my_id={my_id}"

    # str ID (allow "2")
    if isinstance(rid, str):
        try:
            return (int(rid.strip()) == int(my_id)), f"id='{rid}' vs my_id={my_id}"
        except Exception:
            return (rid.strip() == str(my_id)), f"id='{rid}' vs my_id={my_id}"

    # list IDs
    if isinstance(rid, list):
        try:
            ints = set(int(x) for x in rid)
            return (int(my_id) in ints), f"id-list={ints} my_id={my_id}"
        except Exception:
            strs = set(str(x).strip() for x in rid)
            return (str(my_id) in strs), f"id-list-str {strs} my_id={my_id}"

    return False, "id-type-unsupported"

# ---------- main loop with auto-reconnect & heartbeat ----------
def main():
    parser = argparse.ArgumentParser(description="ESP32→Pi Servo client with ID filtering, heartbeat, and auto-reconnect")
    parser.add_argument("host", nargs="?", default=DEFAULT_HOST)
    parser.add_argument("port", nargs="?", type=int, default=DEFAULT_PORT)
    parser.add_argument("--id", type=int, default=DEFAULT_ID, help="This robot's ID (default from $ROBOT_ID or 1)")
    args = parser.parse_args()

    my_id = int(args.id)
    host, port = args.host, int(args.port)
    print(f"[INFO] ROBOT_ID={my_id}")
    print(f"[INFO] HB policy: interval={HB_INTERVAL}s miss={HB_MISS} grace={HB_GRACE}s timeout={HB_TIMEOUT}s")

    buf = b""
    last_cmd_ts = time.time()
    last_hb_ts  = time.time()  # heartbeat timer (robot-agnostic)
    s = None

    while True:
        try:
            if s is None:
                s = connect(host, port, timeout=5.0)
                buf = b""
                now = time.time()
                last_cmd_ts = now
                last_hb_ts  = now
                stop_motors()  # start safe

            # poll readability
            r, _, _ = select.select([s], [], [], 0.1)

            now = time.time()

            # Heartbeat timeout → force reconnect
            if now - last_hb_ts > HB_TIMEOUT:
                raise TimeoutError(f"Missed ~{HB_TIMEOUT:.0f}s of heartbeats (interval {HB_INTERVAL}s * {HB_MISS} + grace).")

            # Watchdog: stop wheels if stale commands
            if now - last_cmd_ts > WATCHDOG_SECONDS:
                stop_motors()
                last_cmd_ts = now  # avoid spamming stop

            if not r:
                continue

            # recv non-blocking
            try:
                chunk = s.recv(4096)
            except BlockingIOError:
                chunk = b""
            if not chunk:
                raise ConnectionError("Server closed connection")

            buf += chunk

            # process complete lines
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                text = line.decode("utf-8", "replace").strip("\r")
                if text == "":
                    print("[DBG] Empty line")
                    continue

                print(f"[RAW] {text}")

                # Extract and parse JSON
                candidate = extract_json(text)
                if not candidate:
                    print("[IGN] No JSON object found")
                    continue
                try:
                    obj = json.loads(candidate)
                except json.JSONDecodeError as e:
                    print(f"[IGN] JSON decode error: {e}")
                    continue

                # --- Heartbeat handling (robot-agnostic) ---
                if isinstance(obj, dict) and obj.get("type") == "heartbeat":
                    last_hb_ts = time.time()
                    print(f"[HB] {obj}")
                    continue

                print(f"[JSON] {obj}")

                # --- ID filtering for control commands ---
                apply_it, reason = should_apply(obj, my_id)
                if not apply_it:
                    print(f"[SKIP] Not for me ({reason})")
                    continue

                # Apply command
                try:
                    left_thr = right_thr = None
                    if "left" in obj:
                        left_thr = deg_to_throttle(obj["left"])
                    if "right" in obj:
                        right_thr = deg_to_throttle(obj["right"])
                    if left_thr is not None or right_thr is not None:
                        set_drive(left_thr, right_thr)
                        print(f"[ACT] drive: left={left_thr:.2f} right={right_thr:.2f}")

                    if "gripper" in obj:
                        mode, val = set_gripper(obj["gripper"])
                        print(f"[ACT] gripper: {mode} -> {val}")

                    last_cmd_ts = time.time()

                except Exception as cmd_err:
                    print(f"[ERR] Command handling failed: {cmd_err}")

        except Exception as e:
            print(f"[WARN] {e} Reconnecting in 2s...")
            try:
                if s: s.close()
            except Exception:
                pass
            s = None
            stop_motors()
            time.sleep(2)

if __name__ == "__main__":
    main()
