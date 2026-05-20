# sim_to_real.py
#
# MuJoCo slider-driven CBF controller for a real SO101 arm.
#
# The user moves joint sliders in the MuJoCo viewer's Control panel.
# This script reads the desired positions, CBF-filters them, writes the
# filtered result to both the viewer (so it shows the safe pose) and the
# real arm over USB.

import os
os.environ['MUJOCO_GL'] = 'glfw'
import argparse, shutil, sys, tempfile, time
from pathlib import Path
import numpy as np
import mujoco, mujoco.viewer


N_JOINTS = 6
STATUS_INTERVAL = 1.0


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
    rgba = np.array([1.0, 0.0, 0.0, 0.5] if penetrated else [0.2, 0.8, 0.2, 0.3])
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


def _prepare_calibration(robot_id, calibration_file, calibration_dir):
    if calibration_file:
        source = Path(calibration_file).expanduser().resolve()
        if robot_id is None:
            robot_id = source.stem.removesuffix("_calib")
        target_dir = Path(tempfile.mkdtemp(prefix="so101-sim2real-calib-"))
        shutil.copy2(source, target_dir / f"{robot_id}.json")
        return robot_id, target_dir
    if calibration_dir:
        return robot_id, Path(calibration_dir).expanduser().resolve()
    return robot_id, None


def create_local_bus(port, robot_id, calibration_file, calibration_dir=None):
    from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig

    robot_id, prepared_calibration_dir = _prepare_calibration(
        robot_id, calibration_file, calibration_dir,
    )
    cfg = SO101FollowerConfig(
        port=port, id=robot_id,
        calibration_dir=prepared_calibration_dir,
        cameras={},
    )
    robot = SO101Follower(cfg)
    robot.connect(calibrate=False)
    if not robot.is_calibrated:
        raise RuntimeError(
            "SO101 is not calibrated. Pass --calibration-file or calibrate with LeRobot first."
        )

    class BusAdapter:
        def __init__(self, r):
            self._robot = r
            self.bus = r.bus

        def sync_read(self, data_name):
            if data_name != "Present_Position":
                return self.bus.sync_read(data_name)
            obs = self._robot.get_observation()
            return {
                key.removesuffix(".pos"): float(
                    value.item() if hasattr(value, "item") else value
                )
                for key, value in obs.items()
                if key.endswith(".pos")
            }

        def sync_write(self, data_name, values):
            if data_name != "Goal_Position":
                return self.bus.sync_write(data_name, values)
            return self._robot.send_action(
                {f"{key}.pos": value for key, value in values.items()}
            )

        def enable_torque(self, motors=None):
            return self.bus.enable_torque(motors)

    return robot, BusAdapter(robot)


def main():
    ap = argparse.ArgumentParser("MuJoCo slider -> CBF -> real SO101 arm")
    ap.add_argument("--port", required=True,
                    help="Serial port for follower arm, e.g. /dev/ttyACM0")
    ap.add_argument("--calibration-file", required=True,
                    help="LeRobot calibration JSON for the follower arm")
    ap.add_argument("--robot-id", default=None,
                    help="LeRobot robot id (defaults from calib filename)")
    ap.add_argument("--min-safe-ee-z", type=float, required=True,
                    help="Table barrier height in metres")
    ap.add_argument("--hz", type=float, default=20.0,
                    help="Control loop rate in Hz (default 20)")
    ap.add_argument("--gain", type=float, default=1.0,
                    help="Velocity gain for tracking slider positions")
    ap.add_argument("--table-z", type=float, default=None,
                    help="Draw translucent table plane at this z")
    ap.add_argument("--stop-margin", type=float, default=0.005,
                    help="FK gate stop margin in metres")
    ap.add_argument("--max-vel-deg", type=float, default=15.0,
                    help="Max joint velocity in deg/s per joint (default 15)")
    args = ap.parse_args()

    dt = 1.0 / max(1e-6, args.hz)
    max_vel_rad = np.deg2rad(args.max_vel_deg)
    model_path = "robot_models/so101/scene.xml"
    if not os.path.exists(model_path):
        print(f"Model not found: {model_path} (cwd={os.getcwd()})")
        return

    print("Loading MuJoCo model …")
    mj_model = mujoco.MjModel.from_xml_path(model_path)
    mj_data = mujoco.MjData(mj_model)

    if args.table_z is not None:
        floor_id = mujoco.mj_name2id(mj_model, mujoco.mjtObj.mjOBJ_GEOM, "floor")
        if floor_id >= 0:
            mj_model.geom_rgba[floor_id, 3] = 0.0

    import jax.numpy as jnp
    from cbfpy import CBF
    from test_so101_real import (
        load_robot, gate_candidate_by_fk, joint_degrees_to_radians,
        joint_radians_to_degrees, _table_clearance_detail, ARM_JOINT_NAMES,
        enable_arm_torque,
    )
    from oscbf.examples.so101_table_avoidance import TableAvoidanceConfig

    print("Loading OSCBF robot + CBF …")
    oscbf_robot = load_robot()
    cbf = CBF.from_config(
        TableAvoidanceConfig(oscbf_robot, min_safe_ee_z=args.min_safe_ee_z)
    )

    print(f"Connecting to real arm on {args.port} …")
    lerobot_robot, bus = create_local_bus(
        args.port, args.robot_id, args.calibration_file,
    )

    try:
        initial_deg = bus.sync_read("Present_Position")
        q_sim = joint_degrees_to_radians(initial_deg)
        mj_data.ctrl[:N_JOINTS] = q_sim
        mj_data.qpos[:N_JOINTS] = q_sim
        mujoco.mj_forward(mj_model, mj_data)

        table_z0, idx0 = _table_clearance_detail(oscbf_robot, q_sim)
        h0 = table_z0 - args.min_safe_ee_z
        print(f"Initial pose: h={h0:.4f}  table_z={table_z0:.4f}  active_idx={idx0}")
        print(f"min_safe_ee_z={args.min_safe_ee_z:.4f}  hz={args.hz}  gain={args.gain}")

        enable_arm_torque(bus)
        last_status = 0.0
        n_steps = 0

        print("\nStarting. Move sliders in the MuJoCo Control panel.\n")

        with mujoco.viewer.launch_passive(mj_model, mj_data) as viewer:
            while viewer.is_running():
                loop_start = time.monotonic()

                desired_q = mj_data.ctrl[:N_JOINTS].copy()

                nominal_vel = args.gain * (desired_q - q_sim) / dt
                nominal_vel = np.clip(nominal_vel, -max_vel_rad, max_vel_rad)
                filtered_vel = np.asarray(
                    cbf.safety_filter(jnp.asarray(q_sim), jnp.asarray(nominal_vel))
                )
                candidate_q = q_sim + filtered_vel * dt

                next_q, _ = gate_candidate_by_fk(
                    robot=oscbf_robot,
                    current_q=q_sim,
                    candidate_q=candidate_q,
                    min_safe_ee_z=args.min_safe_ee_z,
                    stop_margin=args.stop_margin,
                )
                q_sim = next_q
                n_steps += 1

                mj_data.qpos[:N_JOINTS] = q_sim
                mujoco.mj_forward(mj_model, mj_data)

                bus.sync_write("Goal_Position", joint_radians_to_degrees(q_sim))

                viewer.user_scn.ngeom = 0
                penetrated = draw_oscbf_spheres(
                    viewer, oscbf_robot, q_sim, table_z=args.table_z,
                )
                if args.table_z is not None:
                    draw_table_plane(viewer.user_scn, args.table_z, penetrated=penetrated)
                viewer.sync()

                now = time.monotonic()
                if now - last_status >= STATUS_INTERVAL:
                    table_z_val, idx = _table_clearance_detail(oscbf_robot, q_sim)
                    h = table_z_val - args.min_safe_ee_z
                    vel_norm = float(np.linalg.norm(filtered_vel))
                    print(
                        f"h={h:.4f}  table_z={table_z_val:.4f}  "
                        f"|vel|={vel_norm:.4f}  active_idx={idx}  n={n_steps}"
                    )
                    last_status = now

                elapsed = time.monotonic() - loop_start
                if elapsed < dt:
                    time.sleep(dt - elapsed)

    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        if getattr(lerobot_robot, "is_connected", False):
            lerobot_robot.disconnect()
        print("Disconnected.")


if __name__ == "__main__":
    main()
