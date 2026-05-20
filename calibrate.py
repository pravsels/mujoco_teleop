# calibrate.py
#
# Interactive min/max tick capture for Dynamixel or Feetech servos.
# Saves a calibration JSON compatible with leader_pub.py.

import argparse, sys, time, select, json
from models import get_model, get_sdk


def nb_enter_pressed(timeout_s):
    """Non-blocking ENTER detection with select."""
    try:
        readable_streams, _, _ = select.select([sys.stdin], [], [], timeout_s)
        if readable_streams:
            sys.stdin.readline()
            return True
        return False
    except Exception:
        time.sleep(timeout_s)
        return False


def torque_set_many(pkt, ph, ids, addr_torque, enable: bool):
    val = 1 if enable else 0
    for i in ids:
        pkt.write1ByteTxRx(ph, i, addr_torque, val)


def read_pos_dynamixel(pkt, ph, dxl_id, addr, comm_success):
    pos, comm, err = pkt.read4ByteTxRx(ph, dxl_id, addr)
    if comm != comm_success or err != 0:
        raise RuntimeError(f"read pos failed for id {dxl_id} (comm={comm}, err={err})")
    if pos >= 2**31:
        pos -= 2**32
    return int(pos)


def read_pos_feetech(pkt, ph, dxl_id, addr, comm_success):
    pos, comm, err = pkt.read2ByteTxRx(ph, dxl_id, addr)
    if comm != comm_success or err != 0:
        raise RuntimeError(f"read pos failed for id {dxl_id} (comm={comm}, err={err})")
    return int(pos)


def setup_feetech(pkt, ph, ids, mcfg):
    """Feetech-specific setup: unlock, disable torque, clear Phase bit 4 (STS3215)."""
    addr_lock = mcfg.get("addr_lock", 55)
    addr_torque = mcfg["addr_torque_enable"]
    addr_phase = mcfg.get("addr_phase", 18)

    for sid in ids:
        pkt.write1ByteTxRx(ph, sid, addr_lock, 0)
        pkt.write1ByteTxRx(ph, sid, addr_torque, 0)
        phase, _, _ = pkt.read1ByteTxRx(ph, sid, addr_phase)
        if phase & 0x10:
            pkt.write1ByteTxRx(ph, sid, addr_phase, phase & ~0x10)
            print(f"  Servo {sid}: cleared Phase bit 4 (angle feedback mode)")


def main():
    ap = argparse.ArgumentParser("Interactive servo calibration (Dynamixel + Feetech)")
    ap.add_argument("--port", required=True, help="e.g. /dev/ttyUSB0 or COM3")
    ap.add_argument("--model", default="i2rt_yam",
                    help="Robot model: i2rt_yam, so101, so100")
    ap.add_argument("--baud", type=int, default=None,
                    help="Override baud rate (default: from model config)")
    ap.add_argument("--ids", default=None,
                    help="Override servo IDs as comma list (default: from model config)")
    ap.add_argument("--hz", type=float, default=20.0, help="Print refresh rate")
    ap.add_argument("--save", dest="save", action="store_true", default=True,
                    help="Save session min/max JSON (default: on)")
    ap.add_argument("--no-save", dest="save", action="store_false",
                    help="Do not write calibration JSON to disk")
    ap.add_argument("-o", "--output", default=None,
                    help="Output filename (default: from model config)")
    args = ap.parse_args()

    mcfg = get_model(args.model)
    sdk = get_sdk(mcfg["servo_type"])

    ids = [int(x) for x in args.ids.split(",") if x.strip()] if args.ids else mcfg["ids"]
    baud = args.baud or mcfg["baud"]
    joint_names = mcfg["joint_names"]
    labels = [joint_names.get(sid, f"J{sid}") for sid in ids]
    addr_pos = mcfg["addr_present_position"]
    addr_torque = mcfg["addr_torque_enable"]

    if mcfg["servo_type"] == "dynamixel":
        read_pos = lambda pkt, ph, sid: read_pos_dynamixel(pkt, ph, sid, addr_pos, sdk.COMM_SUCCESS)
    else:
        read_pos = lambda pkt, ph, sid: read_pos_feetech(pkt, ph, sid, addr_pos, sdk.COMM_SUCCESS)

    print(f"Model: {args.model} ({mcfg['servo_type']})")
    print(f"Port: {args.port}, Baud: {baud}, IDs: {ids}")

    ph = sdk.PortHandler(args.port)
    if not ph.openPort():
        raise OSError(f"Cannot open port {args.port}")
    if not ph.setBaudRate(baud):
        raise OSError(f"Cannot set baudrate {baud} on {args.port}")

    pkt = sdk.PacketHandler(mcfg["protocol_version"])

    if mcfg["servo_type"] == "feetech":
        setup_feetech(pkt, ph, ids, mcfg)
    else:
        torque_set_many(pkt, ph, ids, addr_torque, False)

    print(f"\nInteractive calibration for {args.model}")
    print("Torque OFF on all IDs for free movement.\n"
          "For each joint: press ENTER to start live read, move it through its full travel,\n"
          "press ENTER again to advance.\n")

    period = 1.0 / max(1e-3, args.hz)
    calib = {}

    try:
        for idx, (dxl_id, label) in enumerate(zip(ids, labels), start=1):
            print(f"[{idx}/{len(ids)}] Joint {label} (ID {dxl_id})")
            input("  → Press ENTER to begin live read… ")

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

        print("Done!")
        if args.save:
            fname = args.output or mcfg["default_calib"]
            with open(fname, "w") as f:
                json.dump(calib, f, indent=2)
            print(f"Saved calibration to {fname}")

    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        ph.closePort()

if __name__ == "__main__":
    main()
