#!/usr/bin/env python3
"""
BLE Central for ESP32 NUS ("ESP-UART") + Servo Control (simple)

- No system calls. No auto "bring-up". If the adapter isn't ready, we fail fast with a clear message.
- Packet: [0] id_lo [1] id_hi [2] left 0..180|0xFF [3] right 0..180|0xFF [4] gripper 0..180|0xFF
- 0xFF means 'unset' (ignore that field)
- left/right mapped via deg_to_throttle (90=stop), gripper = angle (180=close, 0=open)
- Servo channels/pulse ranges match your Wi-Fi reference.  :contentReference[oaicite:1]{index=1}
"""

import os, sys, time, signal, argparse, asyncio, contextlib
from bleak import BleakScanner, BleakClient, BleakError
from adafruit_servokit import ServoKit

# ---------- NUS UUIDs ----------
UUID_NUS_SERVICE = "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
UUID_NUS_RX      = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"   # optional writes
UUID_NUS_TX      = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"   # notifications

# ---------- Defaults ----------
DEFAULT_NAME = "ESP-UART"
DEFAULT_ADAPTER = "hci0"
DEFAULT_ID = int(os.environ.get("ROBOT_ID", "1"))
WATCHDOG_SECONDS = 2.5

# ---------- Servo setup (mirrors your reference) ----------
KIT = ServoKit(channels=16, address=0x40)
CH_GRIPPER = 0           # standard servo
CH_RIGHT   = 1           # continuous
CH_LEFT    = 2           # continuous
KIT.servo[CH_GRIPPER].set_pulse_width_range(500, 2500)
KIT.continuous_servo[CH_LEFT].set_pulse_width_range(1000, 2000)
KIT.continuous_servo[CH_RIGHT].set_pulse_width_range(1000, 2000)

def deg_to_throttle(v):
    """0..180 -> -1..+1 (90=0)."""
    thr = (float(v) - 90.0) / 90.0
    return max(-1.0, min(1.0, thr))

def set_drive(left=None, right=None):
    if left  is not None: KIT.continuous_servo[CH_LEFT].throttle  = float(left)
    if right is not None: KIT.continuous_servo[CH_RIGHT].throttle = float(right)

def set_gripper(angle):
    angle = max(0, min(180, float(angle)))
    KIT.servo[CH_GRIPPER].angle = angle
    return angle

def stop_motors():
    KIT.continuous_servo[CH_LEFT].throttle  = 0.0
    KIT.continuous_servo[CH_RIGHT].throttle = 0.0

def cleanup_and_exit(*_):
    try: stop_motors()
    finally: sys.exit(0)

signal.signal(signal.SIGINT,  cleanup_and_exit)
signal.signal(signal.SIGTERM, cleanup_and_exit)

# ---------- Packet decode ----------
def decode_packet(data: bytes):
    if len(data) != 5:
        return None
    rid   = data[0] | (data[1] << 8)
    left  = data[2]   # 0..180 or 0xFF
    right = data[3]   # 0..180 or 0xFF
    grip  = data[4]   # 0..180 or 0xFF (180=close, 0=open)
    return rid, left, right, grip

def fmt_opt(v): return "unset" if v == 0xFF else str(v)
def fmt_grip(v):
    if v == 0xFF: return "unset"
    if v >= 180:  return "close (180)"
    if v == 0:    return "open (0)"
    return str(v)

# ---------- Controller (ID filter + watchdog) ----------
class Controller:
    def __init__(self, my_id: int, filter_by_id: bool):
        self.my_id = int(my_id)
        self.filter_by_id = bool(filter_by_id)
        self.last_cmd_ts = time.time()

    def maybe_apply(self, rid, left, right, grip):
        if self.filter_by_id and rid != self.my_id:
            print(f"[SKIP] id={rid} != my_id={self.my_id}")
            return
        applied = False
        if left != 0xFF:
            lt = deg_to_throttle(left)
            KIT.continuous_servo[CH_LEFT].throttle = lt
            print(f"[ACT] left  {left:3d} -> {lt:+.2f}")
            applied = True
        if right != 0xFF:
            rt = deg_to_throttle(right)
            KIT.continuous_servo[CH_RIGHT].throttle = rt
            print(f"[ACT] right {right:3d} -> {rt:+.2f}")
            applied = True
        if grip != 0xFF:
            ang = set_gripper(grip)
            label = "close" if ang >= 180 else ("open" if ang == 0 else str(int(ang)))
            print(f"[ACT] gripper -> {label} ({int(ang)})")
            applied = True
        if applied:
            self.last_cmd_ts = time.time()

    def watchdog_tick(self):
        if time.time() - self.last_cmd_ts > WATCHDOG_SECONDS:
            stop_motors()
            self.last_cmd_ts = time.time()
            print("[WD] stale -> stop motors")

# ---------- BLE helpers ----------
async def find_by_name(name: str, adapters=None, timeout: float = 6.0) -> tuple[str, str]:
    """
    Try the provided adapter list in order; return (addr, adapter) on success.
    If adapters is None, probe a small, safe default list.
    """
    probe = adapters or ["hci0", "hci1"]
    last_err = None
    for ad in probe:
        print(f"[Pi] Scanning for '{name}' on {ad} ...")
        try:
            devices = await BleakScanner.discover(timeout=timeout, adapter=ad)
            for d in devices:
                if (d.name or "") == name:
                    print(f"[Pi] Found '{name}' at {d.address} on {ad}")
                    return d.address, ad
        except BleakError as e:
            last_err = e
            print(f"[WARN] {e} (adapter={ad})")
    raise RuntimeError(f"No Bluetooth adapter worked (tried {probe}). Last error: {last_err}")

def make_notify(controller: Controller):
    def _cb(_, data: bytearray):
        pkt = decode_packet(bytes(data))
        if pkt is None:
            # Not a 5B packet; show as text for debugging.
            try:
                sys.stdout.write(bytes(data).decode("utf-8", errors="replace"))
                sys.stdout.flush()
            except Exception:
                print(f"[DBG] non-packet len={len(data)}")
            return
        rid, left, right, grip = pkt
        print(f"[Pi] RX -> id={rid:>3} left={fmt_opt(left):>5} right={fmt_opt(right):>5} grip={fmt_grip(grip):>10}")
        controller.maybe_apply(rid, left, right, grip)
    return _cb

async def stdin_writer(client: BleakClient):
    # Optional debug: type lines to send to ESP32 RX as text
    loop = asyncio.get_event_loop()
    while True:
        line = await loop.run_in_executor(None, sys.stdin.readline)
        if not line:
            await asyncio.sleep(0.05); continue
        try:
            await client.write_gatt_char(UUID_NUS_RX, line.encode("utf-8"), response=False)
        except Exception as e:
            print(f"[Pi] Write failed: {e}")

async def run(addr: str, adapter: str, my_id: int, no_filter: bool):
    ctrl = Controller(my_id, filter_by_id=not no_filter)
    while True:
        try:
            print(f"[Pi] Connecting to {addr} on {adapter} ...")
            async with BleakClient(addr, timeout=10.0, adapter=adapter) as client:
                print("[Pi] Connected")
                await client.start_notify(UUID_NUS_TX, make_notify(ctrl))
                print("[Pi] Subscribed. Filter:",
                      "OFF (all ids)" if no_filter else f"ON (my_id={my_id})")
                writer = asyncio.create_task(stdin_writer(client))
                try:
                    while True:
                        await asyncio.sleep(0.2)
                        ctrl.watchdog_tick()
                except asyncio.CancelledError:
                    pass
                finally:
                    writer.cancel()
                    with contextlib.suppress(Exception):
                        await writer
        except (BleakError, RuntimeError) as e:
            # No auto “fix”; just stop safely and retry.
            print(f"[WARN] {e}  Retrying in 2s...")
            stop_motors()
            await asyncio.sleep(2.0)
        except Exception as e:
            print(f"[WARN] {e}  Retrying in 2s...")
            stop_motors()
            await asyncio.sleep(2.0)

# ---------- CLI ----------
async def main():
    ap = argparse.ArgumentParser(description="BLE angles+gripper servo client (simple)")
    ap.add_argument("--name", default=DEFAULT_NAME, help="Target BLE name to scan for")
    ap.add_argument("--addr", default=None, help="Target MAC (skip scan)")
    ap.add_argument("--adapter", default=None, help="BLE adapter (e.g., hci0). If omitted, we'll probe hci0,hci1.")
    ap.add_argument("--id", default=os.environ.get("ROBOT_ID", DEFAULT_ID), type=int, help="This robot's ID")
    ap.add_argument("--no-filter", action="store_true", help="Apply commands regardless of packet id")
    args = ap.parse_args()

    if args.addr:
        addr, adapter = args.addr, (args.adapter or "hci0")
    else:
        if args.adapter:
            # Force only the adapter the user specified
            addr, adapter = await find_by_name(args.name, adapters=[args.adapter])
        else:
            # Probe a small set (no system tweaks)
            addr, adapter = await find_by_name(args.name, adapters=None)

    print(f"[INFO] ROBOT_ID={args.id} adapter={adapter}")
    await run(addr, adapter=adapter, my_id=int(args.id), no_filter=args.no_filter)

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        cleanup_and_exit()
