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
        try:
            subprocess.Popen(["paplay", _BELL_SOUND], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except FileNotFoundError:
            pass

SUB_ADDR_DEFAULT = "tcp://localhost:6000"

def parse_offsets(raw_offsets):
    offsets = {}
    for raw in raw_offsets:
        name, value = raw.split("=", 1)
        offsets[name] = np.deg2rad(float(value))
    return offsets

def parse_vec3(s):
    """Parse a space-separated 3-vector string like '-0.25 -0.25 0.08'."""
    parts = s.strip().split()
    if len(parts) != 3:
        raise argparse.ArgumentTypeError(f"Expected 3 values, got {len(parts)}: '{s}'")
    return tuple(float(x) for x in parts)

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

def ctrl_to_joint_vector(model, data):
    """Read data.ctrl as a joint vector (for position actuators)."""
    return np.asarray(data.ctrl[:model.njnt], dtype=float)

def write_ctrl_from_joint_vector(model, data, q):
    """Write a joint vector back to data.ctrl."""
    for i in range(min(len(q), model.nu)):
        data.ctrl[i] = q[i]

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


def draw_box(scene, box_lo, box_hi, sphere_positions=None, sphere_radii=None, buffer=0.01):
    """Draw 6 translucent box faces; flash red if any sphere is within buffer of a face."""
    lo = np.asarray(box_lo)
    hi = np.asarray(box_hi)
    center = (lo + hi) / 2.0
    half = (hi - lo) / 2.0

    # Determine which faces are "active" (sphere within buffer distance)
    face_active = [False] * 6  # lo_x, hi_x, lo_y, hi_y, lo_z, hi_z
    if sphere_positions is not None and sphere_radii is not None:
        for pos, r in zip(sphere_positions, sphere_radii):
            for ax in range(3):
                if pos[ax] - lo[ax] - r < buffer * 2:
                    face_active[ax * 2] = True
                if hi[ax] - pos[ax] - r < buffer * 2:
                    face_active[ax * 2 + 1] = True

    # Draw each face as a thin box
    face_thickness = 0.001
    for ax in range(3):
        for side in range(2):  # 0=lo, 1=hi
            if scene.ngeom >= scene.maxgeom:
                return
            face_idx = ax * 2 + side
            active = face_active[face_idx]
            rgba = np.array([1.0, 0.2, 0.2, 0.4]) if active else np.array([0.3, 0.6, 1.0, 0.15])

            pos = center.copy()
            pos[ax] = lo[ax] if side == 0 else hi[ax]

            size = half.copy()
            size[ax] = face_thickness

            geom = scene.geoms[scene.ngeom]
            mujoco.mjv_initGeom(
                geom,
                mujoco.mjtGeom.mjGEOM_BOX,
                size.astype(float),
                pos.astype(float),
                np.eye(3).reshape(-1),
                rgba.astype(float),
            )
            scene.ngeom += 1


def draw_filter_spheres(scene, sphere_positions, sphere_radii, box_lo=None, box_hi=None, buffer=0.01):
    """Draw collision spheres, coloured by proximity to box faces."""
    lo = np.asarray(box_lo) if box_lo is not None else None
    hi = np.asarray(box_hi) if box_hi is not None else None

    for pos, r in zip(sphere_positions, sphere_radii):
        near_wall = False
        if lo is not None and hi is not None:
            for ax in range(3):
                if pos[ax] - lo[ax] - r < buffer * 2:
                    near_wall = True
                if hi[ax] - pos[ax] - r < buffer * 2:
                    near_wall = True

        if near_wall:
            rgba = (1.0, 0.3, 0.0, 0.85)
        else:
            rgba = (0.0, 0.7, 1.0, 0.4)
        add_sphere(scene, pos, float(r), rgba)


def draw_oscbf_spheres(viewer, oscbf_robot, q, table_z=None, mj_model=None, mj_data=None):
    import jax.numpy as jnp
    from so101_collision_model import so101_collision_data

    q_arm = q[: oscbf_robot.num_joints]   # strip gripper / extra joints
    collision = np.asarray(oscbf_robot.link_collision_data(jnp.asarray(q_arm))).copy()
    if collision.size == 0:
        return False

    jaw_sphere_world = None
    if mj_model is not None and mj_data is not None:
        jaw_body_id = mujoco.mj_name2id(mj_model, mujoco.mjtObj.mjOBJ_BODY, "moving_jaw_so101_v1")
        if jaw_body_id >= 0:
            jaw_positions = so101_collision_data["positions"][5]
            jaw_radii = so101_collision_data["radii"][5]
            if jaw_positions:
                local_in_jaw = np.array(jaw_positions[0])
                R = mj_data.xmat[jaw_body_id].reshape(3, 3)
                t = mj_data.xpos[jaw_body_id]
                jaw_sphere_world = R @ local_in_jaw + t
                jaw_row = np.array([*jaw_sphere_world, jaw_radii[0]])
                collision = np.vstack([collision, jaw_row[np.newaxis, :]])

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
    # Safety filter args
    ap.add_argument("--enable-filter", action="store_true",
                    help="Enable the numpy SafetyFilter (clamps joint targets to stay in box).")
    ap.add_argument("--box-min", type=parse_vec3, default=(-0.25, -0.25, 0.05),
                    help="Lower corner of safe box, space-separated: 'x y z'")
    ap.add_argument("--box-max", type=parse_vec3, default=(0.25, 0.25, 0.55),
                    help="Upper corner of safe box, space-separated: 'x y z'")
    ap.add_argument("--buffer", type=float, default=0.01,
                    help="Inward buffer from box faces (metres). Default 0.01.")
    ap.add_argument("--max-vel-deg", type=float, default=None,
                    help="Max joint velocity per step (degrees). Default: no limit.")
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
    if args.show_oscbf_spheres and not args.enable_filter:
        from so101_robot import load_so101_robot
        oscbf_robot = load_so101_robot()

    # Safety filter setup
    safety_filter = None
    safety_kin = None
    if args.enable_filter:
        from so101_safety import SafetyFilter, NumpyKinematics
        safety_kin = NumpyKinematics()
        max_vel = np.deg2rad(args.max_vel_deg) if args.max_vel_deg else None
        safety_filter = SafetyFilter(
            safety_kin,
            args.box_min,
            args.box_max,
            buffer=args.buffer,
            max_vel=max_vel,
        )
        print(f"SafetyFilter enabled: box=[{args.box_min}, {args.box_max}], buffer={args.buffer}")

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
            last_safe_q = qpos_to_joint_vector(model, data)
            while viewer.is_running():
                # receive latest message. timeout=0 makes it non-blocking. 
                socks = dict(poller.poll(timeout=0))
                if sub in socks and socks[sub] == zmq.POLLIN:
                    try:
                        _, payload = sub.recv_multipart(flags=zmq.NOBLOCK)
                        msg = json.loads(payload.decode("utf-8"))

                        qnorm = msg.get("qnorm", [])

                        # Set desired qpos from ZMQ command
                        for adr, q in map_norm_to_qpos(model, qnorm, offsets):
                            data.qpos[adr] = q

                        if safety_filter is not None:
                            desired_q = qpos_to_joint_vector(model, data)
                            safe_q = safety_filter.filter(last_safe_q, desired_q)
                            idx = 0
                            for j in range(model.njnt):
                                if model.jnt_type[j] == mujoco.mjtJoint.mjJNT_FREE:
                                    continue
                                data.qpos[model.jnt_qposadr[j]] = safe_q[idx]
                                idx += 1
                            last_safe_q = safe_q.copy()

                        mujoco.mj_forward(model, data)

                    except Exception:
                        pass  # ignore malformed messages
                else:
                    # No ZMQ input — sliders drive the arm
                    if safety_filter is not None:
                        # Kinematic mode: read slider target, filter, set qpos directly.
                        # No physics (mj_step) — avoids gravity/velocity fighting the clamp.
                        desired_q = ctrl_to_joint_vector(model, data)
                        safe_q = safety_filter.filter(last_safe_q, desired_q)
                        idx = 0
                        for j in range(model.njnt):
                            if model.jnt_type[j] == mujoco.mjtJoint.mjJNT_FREE:
                                continue
                            data.qpos[model.jnt_qposadr[j]] = safe_q[idx]
                            idx += 1
                        mujoco.mj_forward(model, data)
                        last_safe_q = safe_q.copy()
                    else:
                        mujoco.mj_step(model, data)

                # Draw overlays
                viewer.user_scn.ngeom = 0
                penetrated = False

                if safety_filter is not None:
                    q_vec = qpos_to_joint_vector(model, data)
                    sph_pos = safety_kin.sphere_positions(q_vec)
                    sph_rad = safety_kin.sphere_radii(q_vec)
                    draw_filter_spheres(
                        viewer.user_scn, sph_pos, sph_rad,
                        box_lo=args.box_min, box_hi=args.box_max, buffer=args.buffer,
                    )
                    draw_box(
                        viewer.user_scn, args.box_min, args.box_max,
                        sphere_positions=sph_pos, sphere_radii=sph_rad,
                        buffer=args.buffer,
                    )
                    # Check if any sphere is touching the box
                    lo = np.asarray(args.box_min) + args.buffer
                    hi = np.asarray(args.box_max) - args.buffer
                    for pos, r in zip(sph_pos, sph_rad):
                        for ax in range(3):
                            if pos[ax] - lo[ax] - r < 0.001 or hi[ax] - pos[ax] - r < 0.001:
                                penetrated = True
                elif oscbf_robot is not None:
                    penetrated = draw_oscbf_spheres(
                        viewer, oscbf_robot, qpos_to_joint_vector(model, data),
                        table_z=args.table_z, mj_model=model, mj_data=data,
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
