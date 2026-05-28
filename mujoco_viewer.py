# mujoco_viewer.py 

import os, subprocess
os.environ['MUJOCO_GL'] = 'glfw'
import argparse, json, time, zmq
import numpy as np
import mujoco, mujoco.viewer
from utils import make_sub

_BELL_SOUND = "/usr/share/sounds/freedesktop/stereo/bell.oga"
_last_beep = 0.0

def beep():
    global _last_beep
    now = time.time()
    if now - _last_beep < 0.3:
        return
    _last_beep = now
    if os.path.exists(_BELL_SOUND):
        subprocess.Popen(["paplay", _BELL_SOUND], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

SUB_ADDR_DEFAULT = "tcp://localhost:6000"

def parse_offsets(raw_offsets):
    offsets = {}
    for raw in raw_offsets:
        name, value = raw.split("=", 1)
        offsets[name] = np.deg2rad(float(value))
    return offsets

def map_norm_to_qpos(model, qnorm, offsets):
    """
    Map normalized [0,1] values to model qpos using joint ranges.
    - Uses jnt_range when jnt_limited==1.
    - Fallbacks if unlimited: hinge → [-pi, pi], slide → [-1, 1].
    - Skips free joints.
    Yields tuples (qpos_address, q_value). 
    This is more robust than returning a full array, as it handles skipped joints correctly. 
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

            q = lo + x * (hi - lo)
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, j)
            q += offsets.get(name, 0.0)
            
            yield qadr, q

def qpos_to_joint_vector(model, data):
    q = []
    for j in range(model.njnt):
        if model.jnt_type[j] == mujoco.mjtJoint.mjJNT_FREE:
            continue
        q.append(float(data.qpos[model.jnt_qposadr[j]]))
    return np.asarray(q, dtype=float)

def add_sphere(scene, pos, radius, rgba):
    if scene.ngeom >= scene.maxgeom:
        return
    geom = scene.geoms[scene.ngeom]
    mujoco.mjv_initGeom(
        geom,
        mujoco.mjtGeom.mjGEOM_SPHERE,
        np.asarray([radius, 0.0, 0.0], dtype=float),
        np.asarray(pos, dtype=float),
        np.eye(3).reshape(-1),
        np.asarray(rgba, dtype=float),
    )
    scene.ngeom += 1

def draw_table_plane(scene, z, penetrated=False, half_extent=0.4):
    if scene.ngeom >= scene.maxgeom:
        return
    geom = scene.geoms[scene.ngeom]
    if penetrated:
        rgba = np.array([1.0, 0.0, 0.0, 0.5])
    else:
        rgba = np.array([0.2, 0.8, 0.2, 0.3])
    mujoco.mjv_initGeom(
        geom,
        mujoco.mjtGeom.mjGEOM_BOX,
        np.array([half_extent, half_extent, 0.001]),
        np.array([0.0, 0.0, z]),
        np.eye(3).reshape(-1),
        rgba,
    )
    scene.ngeom += 1

def draw_oscbf_spheres(viewer, oscbf_robot, q, table_z=None):
    import jax.numpy as jnp

    collision = np.asarray(oscbf_robot.link_collision_data(jnp.asarray(q)))
    if collision.size == 0:
        return False
    clearances = collision[:, 2] - collision[:, 3]
    active_idx = int(np.argmin(clearances))
    penetrated = table_z is not None and float(clearances[active_idx]) <= table_z
    for idx, row in enumerate(collision):
        sphere_z = row[2] - row[3]
        if table_z is not None and sphere_z <= table_z:
            rgba = (1.0, 0.0, 0.0, 0.9)
        elif idx == active_idx:
            rgba = (1.0, 0.6, 0.0, 0.75)
        else:
            rgba = (0.0, 0.7, 1.0, 0.35)
        add_sphere(viewer.user_scn, row[:3], float(row[3]), rgba)
    return penetrated

def main():
    ap = argparse.ArgumentParser("MuJoCo viewer for normalized joint states (<model>.state_norm)")
    ap.add_argument("--model", default="i2rt_yam",
                    help="Robot model name (folder under robot_models/)")
    ap.add_argument("--sub-addr", default=SUB_ADDR_DEFAULT,
                    help="ZMQ SUB address (default tcp://localhost:6000)")
    ap.add_argument("--rate", type=float, default=100.0,
                    help="Viewer update rate (Hz)")
    ap.add_argument("--offset-deg", action="append", default=[],
                    help="Joint offset as name=degrees, e.g. wrist_roll=90. Can be repeated.")
    ap.add_argument("--show-oscbf-spheres", action="store_true",
                    help="Overlay OSCBF collision spheres for the live SO101 pose.")
    ap.add_argument("--table-z", type=float, default=None,
                    help="Draw a translucent plane at this z height (calibrated table level).")
    args = ap.parse_args()
    offsets = parse_offsets(args.offset_deg)

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
    if args.table_z is not None:
        floor_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "floor")
        if floor_id >= 0:
            model.geom_rgba[floor_id, 3] = 0.0
    oscbf_robot = None
    if args.show_oscbf_spheres:
        from so101_robot import load_so101_robot
        oscbf_robot = load_so101_robot()

    # ZMQ sub
    ctx = zmq.Context.instance()
    sub = make_sub(ctx, args.sub_addr, topic)

    poller = zmq.Poller()
    poller.register(sub, zmq.POLLIN)

    # a safety mechanism against a rate of 0 
    dt = 1.0 / max(1e-6, args.rate)
    print(f"\nSubscribing to {args.sub_addr} | topic='{topic}'")
    print("Launching viewer…")

    with mujoco.viewer.launch_passive(model, data) as viewer:
        try:
            last = time.time()
            while viewer.is_running():
                # receive latest message. timeout=0 makes it non-blocking. 
                socks = dict(poller.poll(timeout=0))
                if sub in socks and socks[sub] == zmq.POLLIN:
                    try:
                        _, payload = sub.recv_multipart(flags=zmq.NOBLOCK)
                        msg = json.loads(payload.decode("utf-8"))

                        qnorm = msg.get("qnorm", [])

                        # loop through and set norm values for every joint 
                        for adr, q in map_norm_to_qpos(model, qnorm, offsets):
                            data.qpos[adr] = q

                        mujoco.mj_forward(model, data)

                    except Exception:
                        pass  # ignore malformed messages

                viewer.user_scn.ngeom = 0
                penetrated = False
                if oscbf_robot is not None:
                    penetrated = draw_oscbf_spheres(
                        viewer, oscbf_robot, qpos_to_joint_vector(model, data),
                        table_z=args.table_z,
                    )
                if args.table_z is not None:
                    draw_table_plane(viewer.user_scn, args.table_z, penetrated=penetrated)
                if penetrated:
                    beep()
                viewer.sync()

                # simple pacing to enforce target refresh rate 
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

