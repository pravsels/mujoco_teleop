# cbf_relay.py
#
# Sits between leader_pub.py and mujoco_viewer.py, applying the OSCBF safety
# filter to leader arm commands before forwarding them to the viewer.
#
# leader_pub (port 6000) -> cbf_relay (sub 6000, pub 6001) -> mujoco_viewer (sub 6001)

import argparse, json, time, os
import numpy as np
import mujoco
import zmq

from utils import make_sub, make_pub

SUB_ADDR_DEFAULT = "tcp://localhost:6000"
PUB_ADDR_DEFAULT = "tcp://*:6001"
STATUS_INTERVAL = 1.0


def parse_offsets(raw_offsets):
    offsets = {}
    for raw in raw_offsets:
        name, value = raw.split("=", 1)
        offsets[name] = np.deg2rad(float(value))
    return offsets


class JointNormalizer:
    """Bidirectional qnorm [0,1] <-> radians using MuJoCo joint limits + offsets."""

    def __init__(self, model, offsets):
        self.n = model.njnt
        self.lo = np.zeros(self.n)
        self.hi = np.zeros(self.n)
        self.offset = np.zeros(self.n)

        for j in range(self.n):
            jtype = model.jnt_type[j]
            limited = model.jnt_limited[j]
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, j)

            if jtype == mujoco.mjtJoint.mjJNT_FREE:
                continue

            if limited:
                self.lo[j] = model.jnt_range[j, 0]
                self.hi[j] = model.jnt_range[j, 1]
            else:
                self.lo[j] = -np.pi
                self.hi[j] = np.pi

            self.offset[j] = offsets.get(name, 0.0)

    def to_rad(self, qnorm):
        x = np.clip(np.asarray(qnorm[:self.n], dtype=float), 0.0, 1.0)
        return self.lo + x * (self.hi - self.lo) + self.offset

    def to_norm(self, q_rad):
        q = np.asarray(q_rad, dtype=float) - self.offset
        span = self.hi - self.lo
        span = np.where(span == 0.0, 1.0, span)
        return np.clip((q - self.lo) / span, 0.0, 1.0)


def main():
    ap = argparse.ArgumentParser(
        "CBF relay: leader_pub -> safety filter -> mujoco_viewer"
    )
    ap.add_argument("--sub-addr", default=SUB_ADDR_DEFAULT,
                    help="ZMQ SUB address for leader state (default %(default)s)")
    ap.add_argument("--pub-addr", default=PUB_ADDR_DEFAULT,
                    help="ZMQ PUB address for filtered state (default %(default)s)")
    ap.add_argument("--model", default="so101",
                    help="Robot model (folder under robot_models/)")
    ap.add_argument("--min-safe-ee-z", type=float, required=True,
                    help="Table barrier height in metres (same as --table-z in viewer)")
    ap.add_argument("--hz", type=float, default=100.0,
                    help="Relay loop rate (Hz)")
    ap.add_argument("--offset-deg", action="append", default=[],
                    help="Joint offset as name=degrees, e.g. wrist_roll=90")
    ap.add_argument("--gain", type=float, default=1.0,
                    help="Velocity gain: how aggressively sim tracks leader (default 1.0)")
    ap.add_argument("--stop-margin", type=float, default=0.005,
                    help="FK gate stop margin in metres (default 0.005)")
    args = ap.parse_args()

    offsets = parse_offsets(args.offset_deg)
    dt = 1.0 / max(1e-6, args.hz)
    topic = f"{args.model}.state_norm"

    model_path = f"robot_models/{args.model}/scene.xml"
    if not os.path.exists(model_path):
        print(f"Model not found: {model_path} (cwd={os.getcwd()})")
        return

    print("Loading MuJoCo model for joint limits …")
    mj_model = mujoco.MjModel.from_xml_path(model_path)
    normalizer = JointNormalizer(mj_model, offsets)
    print(f"  joints={mj_model.njnt}, limits loaded")

    import jax.numpy as jnp
    from cbfpy import CBF
    from test_so101_real import load_robot, gate_candidate_by_fk
    from oscbf.examples.so101_table_avoidance import TableAvoidanceConfig

    print("Loading OSCBF robot and building CBF …")
    robot = load_robot()
    cbf = CBF.from_config(
        TableAvoidanceConfig(robot, min_safe_ee_z=args.min_safe_ee_z)
    )
    print(f"  min_safe_ee_z={args.min_safe_ee_z:.4f} m, gain={args.gain}")

    ctx = zmq.Context.instance()
    sub = make_sub(ctx, args.sub_addr, topic)
    pub = make_pub(ctx, args.pub_addr, topic)

    q_sim = None
    desired_q = None
    last_good_qnorm = None
    last_status = 0.0
    n_filtered = 0

    print(f"Relay running at {args.hz:.0f} Hz. Ctrl+C to stop.\n")

    try:
        while True:
            loop_start = time.monotonic()

            try:
                _, payload = sub.recv_multipart(flags=zmq.NOBLOCK)
                msg = json.loads(payload.decode("utf-8"))
                qnorm = np.array(msg.get("qnorm", []), dtype=float)
                if len(qnorm) >= normalizer.n:
                    nan_mask = np.isnan(qnorm[:normalizer.n])
                    if nan_mask.any():
                        if last_good_qnorm is not None:
                            qnorm[:normalizer.n] = np.where(
                                nan_mask, last_good_qnorm, qnorm[:normalizer.n]
                            )
                        else:
                            pass  # skip until we get a fully valid message
                    if not np.isnan(qnorm[:normalizer.n]).any():
                        last_good_qnorm = qnorm[:normalizer.n].copy()
                        desired_q = normalizer.to_rad(qnorm)
                        if q_sim is None:
                            q_sim = desired_q.copy()
            except zmq.Again:
                pass

            if q_sim is None or desired_q is None:
                time.sleep(dt)
                continue

            nominal_vel = args.gain * (desired_q - q_sim) / dt
            filtered_vel = np.asarray(
                cbf.safety_filter(jnp.asarray(q_sim), jnp.asarray(nominal_vel))
            )
            candidate_q = q_sim + filtered_vel * dt

            next_q, _ = gate_candidate_by_fk(
                robot=robot,
                current_q=q_sim,
                candidate_q=candidate_q,
                min_safe_ee_z=args.min_safe_ee_z,
                stop_margin=args.stop_margin,
            )
            q_sim = next_q
            n_filtered += 1

            qnorm_out = normalizer.to_norm(q_sim).tolist()
            out_msg = {"t": time.time(), "qnorm": qnorm_out}
            pub.send_multipart([topic.encode(), json.dumps(out_msg).encode()])

            now = time.monotonic()
            if now - last_status >= STATUS_INTERVAL:
                from test_so101_real import _table_clearance_detail
                table_z, idx = _table_clearance_detail(robot, q_sim)
                h = table_z - args.min_safe_ee_z
                vel_norm = float(np.linalg.norm(filtered_vel))
                print(
                    f"h={h:.4f}  table_z={table_z:.4f}  "
                    f"|vel|={vel_norm:.4f}  active_idx={idx}  "
                    f"n={n_filtered}"
                )
                last_status = now

            elapsed = time.monotonic() - loop_start
            if elapsed < dt:
                time.sleep(dt - elapsed)

    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        pub.close(0)
        sub.close(0)
        ctx.term()


if __name__ == "__main__":
    main()
