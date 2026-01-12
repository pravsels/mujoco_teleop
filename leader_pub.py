# leader_pub.py

# Reads Dynamixel PRESENT_POSITION (ticks) via GroupSyncRead,
# normalizes each joint to [0,1] using per-joint min/max from a calib JSON,
# and publishes on ZMQ as:  { "t": <unix_time>, "qnorm": [..] }

# Default calib file: ./yam.json

import argparse, json, time, zmq
from dynamixel_sdk import PortHandler, PacketHandler, GroupSyncRead, COMM_SUCCESS

# ----------------- constants -----------------

PUB_ADDR = "tcp://*:6000"            # ZMQ PUB bind address
ADDR_PRESENT_POSITION = 132          # X-series, Protocol 2.0
LEN_PRESENT_POSITION = 4             # 4 bytes

# ----------------- helpers -------------------

def make_pub(ctx, addr, topic):
    pub = ctx.socket(zmq.PUB)
    pub.bind(addr)
    print(f"Publishing on {addr}, topic = {topic}")
    return pub

def load_calib(path, ids):
    """Load calib JSON (ticks) and return dict id -> (min, max)."""
    with open(path, "r") as f:
        raw = json.load(f)

    by_id = {}
    for name, entry in raw.items():
        sid = int(entry["id"])
        lo  = int(entry["min"])
        hi  = int(entry["max"])
        if hi <= lo:
            raise ValueError(f"Invalid span for joint '{name}' (id {sid}): min >= max")
        by_id[sid] = (lo, hi)

    # ensure every requested id has calib
    missing = [sid for sid in ids if sid not in by_id]
    if missing:
        raise KeyError(f"Calib missing for IDs: {missing}. Please update {path}.")
    return by_id

def norm01(x_ticks, lo, hi):
    """Clamp and normalize ticks to [0,1]."""
    if x_ticks <= lo: return 0.0
    if x_ticks >= hi: return 1.0
    return (float(x_ticks) - lo) / (hi - lo)


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
    ap.add_argument("--calib", default="gello_jannik.json",
                    help="Calibration JSON with per-joint min/max ticks (default yam.json)")
    args = ap.parse_args()

    ids = [int(x) for x in args.ids.split(",") if x.strip()]
    topic_norm = f"{args.model}.state_norm"
    dt = 0.0 if args.hz <= 0 else 1.0 / args.hz

    # load calibration (ticks)
    calib = load_calib(args.calib, ids)   # id -> (min,max)

    # open bus
    ph = PortHandler(args.port)
    if not ph.openPort():
        raise OSError(f"Cannot open port {args.port}")
    if not ph.setBaudRate(args.baud):
        raise OSError(f"Cannot set baudrate {args.baud} on {args.port}")
    pkt = PacketHandler(2.0)

    # group read PRESENT_POSITION for all IDs
    gsr = GroupSyncRead(ph, pkt, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
    for sid in ids:
        gsr.addParam(sid)

    # ZMQ publisher
    ctx = zmq.Context()
    pub = make_pub(ctx, PUB_ADDR, topic_norm)
    print("Streaming normalized joint state… Ctrl+C to stop.")

    try:
        while True:
            # tries to read from all of the motors, using it's initial configuration set above 
            if gsr.txRxPacket() != COMM_SUCCESS:
                if dt > 0: time.sleep(dt)
                continue

            qnorm = []

            # accessing already read encoder ticks from each motor ID 
            for sid in ids:
                ticks = gsr.getData(sid, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
                lo, hi = calib[sid]
                qnorm.append(norm01(int(ticks), lo, hi))

            msg = {"t": time.time(), "qnorm": qnorm}
            pub.send_multipart([topic_norm.encode(), json.dumps(msg).encode()])

            if dt > 0:
                time.sleep(dt)
    except KeyboardInterrupt:
        pass
    finally:
        pub.close(0)
        ctx.term()
        ph.closePort()

if __name__ == "__main__":
    main()
