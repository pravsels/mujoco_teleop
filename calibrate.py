# calibrate.py

import argparse, sys, time
from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS
import select, json 

# Control Table (X-series, Protocol 2.0)
ADDR_TORQUE_ENABLE    = 64     # 1 byte
ADDR_PRESENT_POSITION = 132    # 4 bytes
MOTOR_RES = 4096               # ticks [0..4095]

def nb_enter_pressed(timeout_s):
    """Non-blocking ENTER detection with select."""
    try:
        # returns readable, writable and exception streams 
        readable_streams, _, _ = select.select([sys.stdin], [], [], timeout_s)

        if readable_streams:
            sys.stdin.readline()
            return True

        return False
    except Exception:
        time.sleep(timeout_s)
        return False

def torque_set_many(pkt, ph, ids, enable: bool):
    val = 1 if enable else 0
    for i in ids:
        pkt.write1ByteTxRx(ph, i, ADDR_TORQUE_ENABLE, val)

def read_pos(pkt, ph, dxl_id):
    pos, comm, err = pkt.read4ByteTxRx(ph, dxl_id, ADDR_PRESENT_POSITION)
    if comm != COMM_SUCCESS or err != 0:
        raise RuntimeError(f"read pos failed for id {dxl_id} (comm={comm}, err={err})")
    if pos >= 2**31:
        pos -= 2**32
    return int(pos)

def main():
    ap = argparse.ArgumentParser("Interactive Dynamixel joint reader (ticks only, no save)")
    ap.add_argument("--port", required=True, help="e.g. /dev/ttyUSB0 or COM3")
    ap.add_argument("--baud", type=int, default=57600)
    ap.add_argument("--ids", default="1,2,3,4,5,6,7", help="Comma list of servo IDs (order = joints)")
    ap.add_argument("--names", default="", help="Optional comma list of names matching --ids")
    ap.add_argument("--hz", type=float, default=20.0, help="Print refresh rate")
    ap.add_argument("--save", dest="save", action="store_true", default=True,
                    help="Save session min/max JSON (ticks) to yam.json (default: on)")
    ap.add_argument("--no-save", dest="save", action="store_false",
                    help="Do not write calibration JSON to disk")
    args = ap.parse_args()

    ids = [int(x) for x in args.ids.split(",") if x.strip()]
    names = [x.strip() for x in args.names.split(",")] if args.names else []
    if names and len(names) != len(ids):
        ap.error("--names must match number of --ids")
    labels = names if names else [f"J{i+1}" for i in range(len(ids))]

    ph = PortHandler(args.port)
    if not ph.openPort():
        raise OSError(f"Cannot open port {args.port}")
    if not ph.setBaudRate(args.baud):
        raise OSError(f"Cannot set baudrate {args.baud} on {args.port}")

    pkt = PacketHandler(2.0)

    print("\nDynamixel interactive reader")
    print("Default behavior: torque OFF on all IDs for maximum freedom.\n"
          "For each joint: press ENTER to start live read, move it through travel, press ENTER again to advance.\n")

    # FREE ALL BY DEFAULT
    print("Freeing all joints (torque OFF on all IDs)…")
    torque_set_many(pkt, ph, ids, False)

    period = 1.0 / max(1e-3, args.hz)
    calib = {}

    try:
        for idx, (dxl_id, label) in enumerate(zip(ids, labels), start=1):
            print(f"[{idx}/{len(ids)}] Joint {label} (ID {dxl_id})")
            input("  → Press ENTER to begin live read; press ENTER again to move to next joint… ")

            vmin, vmax = float("inf"), float("-inf")
            print("  Live ticks |   min …   max   (Ctrl+C to quit)")
            print("  ---------------------------------------------")

            while True:
                try:
                    val = read_pos(pkt, ph, dxl_id)
                except Exception as e:
                    sys.stdout.write("\r  read error: %s" % e)
                    sys.stdout.flush()
                    time.sleep(period)
                    continue

                vmin = min(vmin, val)
                vmax = max(vmax, val)

                calib[label] = {"id": dxl_id, "min": int(vmin), "max": int(vmax)}

                sys.stdout.write(f"\r     {val:6d} | {int(vmin):6d} … {int(vmax):6d}     ")
                sys.stdout.flush()

                if nb_enter_pressed(period):
                    sys.stdout.write(f"\n  ↳ final min/max for {label}: {int(vmin)} … {int(vmax)}\n\n")
                    sys.stdout.flush()
                    break

        print("Done reading and saving values!")
        if args.save:
            fname = f"yam.json"
            with open(fname, "w") as f:
                json.dump(calib, f, indent=2)
            print(f"Saved calibration to {fname}")

    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        # leave torques OFF (since that’s the default behavior here)
        ph.closePort()

if __name__ == "__main__":
    main()
