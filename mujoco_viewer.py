# mujoco_viewer.py 

import os
os.environ['MUJOCO_GL'] = 'glfw'
import argparse, json, time, zmq
import numpy as np
import mujoco, mujoco.viewer
from utils import make_sub

SUB_ADDR_DEFAULT = "tcp://localhost:6000"

def map_norm_to_qpos(model, qnorm):
    """
    Map normalized [0,1] values to model qpos using joint ranges.
    - Uses jnt_range when jnt_limited==1.
    - Fallbacks if unlimited: hinge → [-pi, pi], slide → [-1, 1].
    - Skips free joints.
    Yields tuples (qpos_address, q_value).
    """
    n = min(len(qnorm), model.njnt)
    for j in range(n):
        jtype   = model.jnt_type[j]
        limited = model.jnt_limited[j]
        qadr    = model.jnt_qposadr[j]
        x = float(qnorm[j])
        if x < 0.0: x = 0.0
        if x > 1.0: x = 1.0

        if jtype == mujoco.mjtJoint.mjJNT_FREE:
            continue  # free joints have 7 qpos slots; not handled here

        if jtype in (mujoco.mjtJoint.mjJNT_HINGE, mujoco.mjtJoint.mjJNT_SLIDE):
            if limited:
                lo, hi = model.jnt_range[j, 0], model.jnt_range[j, 1]
            else:
                if jtype == mujoco.mjtJoint.mjJNT_HINGE:
                    lo, hi = -np.pi, np.pi
                else:
                    lo, hi = -1.0, 1.0
            q = lo + x * (hi - lo)
            yield qadr, q

def main():
    ap = argparse.ArgumentParser("MuJoCo viewer for normalized joint states (<model>.state_norm)")
    ap.add_argument("--model", default="i2rt_yam",
                    help="Robot model name (folder under robot_models/)")
    ap.add_argument("--sub-addr", default=SUB_ADDR_DEFAULT,
                    help="ZMQ SUB address (default tcp://localhost:6000)")
    ap.add_argument("--rate", type=float, default=100.0,
                    help="Viewer update rate (Hz)")
    args = ap.parse_args()

    topic = f"{args.model}.state_norm"
    model_path = f"robot_models/{args.model}/scene.xml"

    if not os.path.exists(model_path):
        print(f"Model not found: {model_path}")
        print(f"CWD: {os.getcwd()}")
        return

    print("Loading robot model …")
    model = mujoco.MjModel.from_xml_path(model_path)
    data  = mujoco.MjData(model)
    print(f"Loaded. joints={model.njnt}, dof={model.nv}, nq={model.nq}")

    # ZMQ sub
    ctx = zmq.Context.instance()
    sub = make_sub(ctx, args.sub_addr, topic)

    poller = zmq.Poller()
    poller.register(sub, zmq.POLLIN)

    dt = 1.0 / max(1e-6, args.rate)
    print(f"\nSubscribing to {args.sub_addr} | topic='{topic}'")
    print("Launching viewer…")

    with mujoco.viewer.launch_passive(model, data) as viewer:
        try:
            last = time.time()
            while viewer.is_running():
                # receive latest (non-blocking)
                socks = dict(poller.poll(timeout=0))
                if sub in socks and socks[sub] == zmq.POLLIN:
                    try:
                        _, payload = sub.recv_multipart(flags=zmq.NOBLOCK)
                        msg = json.loads(payload.decode("utf-8"))
                        qnorm = msg.get("qnorm", [])
                        for adr, q in map_norm_to_qpos(model, qnorm):
                            data.qpos[adr] = q

                        mujoco.mj_forward(model, data)

                    except Exception:
                        pass  # ignore malformed messages

                viewer.sync()

                # simple pacing
                now = time.time()
                sleep = dt - (now - last)
                if sleep > 0:
                    time.sleep(sleep)
                last = now
        except KeyboardInterrupt:
            pass

    sub.close(0)
    ctx.term()
    print("Viewer closed.")

if __name__ == "__main__":
    main()

