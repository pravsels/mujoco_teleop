# leader_pub.py

# Reads Dynamixel PRESENT_POSITION (ticks) via GroupSyncRead,
# normalizes each joint to [0,1] using per-joint min/max from a calib JSON,
# and publishes on ZMQ as:  { "t": <unix_time>, "qnorm": [..] }

# Default calib file: ./yam.json

import argparse, json, sys, time, zmq
from dynamixel_sdk import PortHandler, PacketHandler, GroupSyncRead, COMM_SUCCESS

# ----------------- constants -----------------

PUB_ADDR = "tcp://*:6000"            # ZMQ PUB bind address
ADDR_PRESENT_POSITION = 132          # X-series, Protocol 2.0
LEN_PRESENT_POSITION = 4             # 4 bytes

# Friendly joint names per Dynamixel ID (matches lerobot_yam yam_leader.py)
DEFAULT_JOINT_NAMES = {
    1: "shoulder_pan",
    2: "shoulder_lift",
    3: "elbow_flex",
    4: "wrist_flex",
    5: "wrist_roll",
    6: "wrist_yaw",
    7: "gripper",
}

CLEAR_LINE = "\x1b[2K\r"

# ----------------- helpers -------------------

def make_pub(ctx, addr, topic):
    pub = ctx.socket(zmq.PUB)
    pub.bind(addr)
    print(f"Publishing on {addr}, topic = {topic}")
    return pub

def load_calib(path, ids):
    """Load calib JSON (ticks) and return dict id -> (name, min, max).

    Supports two formats:

    1. Legacy format (e.g. yam.json):
       { "J1": {"id": 1, "min": 970, "max": 2953}, ... }

    2. LeRobot format (e.g. yam_lerobot.json):
       { "shoulder_pan": {"id": 1, "drive_mode": 0, "homing_offset": 0,
                          "range_min": 1283, "range_max": 2807}, ... }

    Friendly joint names are taken from the JSON key when it is already
    friendly (e.g. 'shoulder_pan'); otherwise we fall back to
    DEFAULT_JOINT_NAMES (e.g. id 6 -> 'wrist_yaw').
    """
    with open(path, "r") as f:
        raw = json.load(f)

    by_id = {}
    for key, entry in raw.items():
        sid = int(entry["id"])
        if "range_min" in entry and "range_max" in entry:
            lo = int(entry["range_min"])
            hi = int(entry["range_max"])
        elif "min" in entry and "max" in entry:
            lo = int(entry["min"])
            hi = int(entry["max"])
        else:
            raise KeyError(
                f"Calib entry '{key}' missing 'range_min'/'range_max' or 'min'/'max'"
            )
        if hi <= lo:
            raise ValueError(f"Invalid span for joint '{key}' (id {sid}): min >= max")

        if key in DEFAULT_JOINT_NAMES.values():
            friendly = key
        else:
            friendly = DEFAULT_JOINT_NAMES.get(sid, key)
        by_id[sid] = (friendly, lo, hi)

    missing = [sid for sid in ids if sid not in by_id]
    if missing:
        raise KeyError(f"Calib missing for IDs: {missing}. Please update {path}.")
    return by_id

def norm01(x_ticks, lo, hi):
    """Clamp and normalize ticks to [0,1]."""
    if x_ticks <= lo: return 0.0
    if x_ticks >= hi: return 1.0
    return (float(x_ticks) - lo) / (hi - lo)

def in_range(ticks, lo, hi, tol):
    return (lo - tol) <= ticks <= (hi + tol)

def read_all_ticks(gsr, ids, max_attempts=10, retry_sleep_s=0.005):
    """Read ticks for every id with light retry. Returns dict id -> int ticks, or None on persistent failure."""
    for _ in range(max_attempts):
        if gsr.txRxPacket() == COMM_SUCCESS:
            return {sid: int(gsr.getData(sid, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)) for sid in ids}
        time.sleep(retry_sleep_s)
    return None

def preflight_range_check(gsr, ids, calib, tol_ticks, refresh_hz):
    """Phase 1: block until every joint is within its calibrated range (with tolerance).

    Prints a single live-updating line per refresh listing offending joints by
    friendly name and the direction the user needs to rotate.
    """
    period = 1.0 / max(1.0, float(refresh_hz))
    print("Preflight: checking GELLO joint ranges. Rotate any flagged joint until it is in range.",
          flush=True)
    try:
        while True:
            ticks_by_id = read_all_ticks(gsr, ids)
            if ticks_by_id is None:
                sys.stdout.write(CLEAR_LINE + "Preflight: read failure, retrying...")
                sys.stdout.flush()
                time.sleep(period)
                continue

            out = []
            for sid in ids:
                name, lo, hi = calib[sid]
                t = ticks_by_id[sid]
                if not in_range(t, lo, hi, tol_ticks):
                    direction = "rotate to DECREASE" if t > hi else "rotate to INCREASE"
                    out.append((name, t, lo, hi, direction))

            if not out:
                sys.stdout.write(CLEAR_LINE)
                sys.stdout.flush()
                print("Preflight: all joints in range. Starting publisher.", flush=True)
                return

            parts = [
                f"{name} (ticks={t}, valid=[{lo},{hi}], {direction})"
                for (name, t, lo, hi, direction) in out
            ]
            sys.stdout.write(CLEAR_LINE + "Out of range: " + " | ".join(parts))
            sys.stdout.flush()
            time.sleep(period)
    except KeyboardInterrupt:
        print("\nPreflight aborted by user.", flush=True)
        raise


# ----------------- main ----------------------

def main():
    ap = argparse.ArgumentParser("Dynamixel → normalized [0,1] publisher (ticks-based)")
    ap.add_argument("--port", required=True, help="e.g. /dev/ttyUSB0 or COM3")
    ap.add_argument("--baud", type=int, default=57600)
    ap.add_argument("--ids", default="1,2,3,4,5,6,7",
                    help="Comma list of servo IDs (order = joints)")
    ap.add_argument("--model", default="i2rt_yam",
                    help="Topic/model prefix (default i2rt_yam)")
    ap.add_argument("--hz", type=float, default=100.0,
                    help="Publish rate in Hz (<=0 = as fast as possible)")
    ap.add_argument("--calib", default="yam.json",
                    help="Calibration JSON with per-joint ticks. Supports both the "
                         "legacy {min,max} format and the LeRobot {range_min,range_max} "
                         "format (e.g. yam_lerobot.json). Default: yam.json")

    # Range-safety options (mirrors lerobot_yam yam_leader.py)
    ap.add_argument("--no-preflight", action="store_true",
                    help="Skip preflight range check at startup")
    ap.add_argument("--no-freeze-out-of-range", action="store_true",
                    help="Do not send NaN for out-of-range joints during runtime")
    ap.add_argument("--oor-tol-ticks", type=int, default=20,
                    help="Tolerance in ticks around calibrated [min,max] (default 20)")
    ap.add_argument("--oor-warn-period-s", type=float, default=1.0,
                    help="Min seconds between repeated out-of-range warnings per joint")
    ap.add_argument("--preflight-hz", type=float, default=10.0,
                    help="Preflight refresh rate while waiting for joints to enter range")

    args = ap.parse_args()

    ids = [int(x) for x in args.ids.split(",") if x.strip()]
    topic_norm = f"{args.model}.state_norm"
    dt = 0.0 if args.hz <= 0 else 1.0 / args.hz

    calib = load_calib(args.calib, ids)   # id -> (name, min, max)

    ph = PortHandler(args.port)
    if not ph.openPort():
        raise OSError(f"Cannot open port {args.port}")
    if not ph.setBaudRate(args.baud):
        raise OSError(f"Cannot set baudrate {args.baud} on {args.port}")
    pkt = PacketHandler(2.0)

    gsr = GroupSyncRead(ph, pkt, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
    for sid in ids:
        gsr.addParam(sid)

    # Phase 1: preflight
    if not args.no_preflight:
        preflight_range_check(
            gsr, ids, calib,
            tol_ticks=max(0, int(args.oor_tol_ticks)),
            refresh_hz=args.preflight_hz,
        )

    ctx = zmq.Context()
    pub = make_pub(ctx, PUB_ADDR, topic_norm)
    print("Streaming normalized joint state… Ctrl+C to stop.")

    # Phase 2A runtime state
    out_of_range_joints: set[int] = set()
    last_warn_time: dict[int, float] = {}
    tol = max(0, int(args.oor_tol_ticks))
    warn_period = max(0.0, float(args.oor_warn_period_s))
    freeze = not args.no_freeze_out_of_range

    try:
        while True:
            if gsr.txRxPacket() != COMM_SUCCESS:
                if dt > 0: time.sleep(dt)
                continue

            qnorm = []
            now = time.perf_counter()

            for sid in ids:
                ticks = int(gsr.getData(sid, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION))
                name, lo, hi = calib[sid]

                if in_range(ticks, lo, hi, tol):
                    if sid in out_of_range_joints:
                        print(f"[range] {name} back in range (ticks={ticks}). Resuming.",
                              flush=True)
                        out_of_range_joints.discard(sid)
                    qnorm.append(norm01(ticks, lo, hi))
                else:
                    last = last_warn_time.get(sid, 0.0)
                    if now - last >= warn_period:
                        direction = "rotate to DECREASE" if ticks > hi else "rotate to INCREASE"
                        action = "Holding joint." if freeze else "Clamping at limit."
                        print(
                            f"[range] {name} OUT OF RANGE (ticks={ticks}, "
                            f"valid=[{lo},{hi}]). {action} {direction} ticks.",
                            flush=True,
                        )
                        last_warn_time[sid] = now
                    out_of_range_joints.add(sid)
                    if freeze:
                        qnorm.append(float("nan"))
                    else:
                        qnorm.append(norm01(ticks, lo, hi))

            msg = {"t": time.time(), "qnorm": qnorm}
            pub.send_multipart([topic_norm.encode(), json.dumps(msg).encode()])

            if dt > 0:
                time.sleep(dt)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            pub.close(0)
            ctx.term()
        except Exception:
            pass
        ph.closePort()

if __name__ == "__main__":
    main()
