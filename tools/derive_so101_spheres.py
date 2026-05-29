"""Derive a gripper-end collision sphere cluster for the SO101.

We approximate the wrist + gripper + moving-jaw geometry with a few spheres so
the workspace-containment CBF can keep the *whole gripper* (not just a single
end-effector point) inside the safe box.

The model produces 6 links:
  - Links 0-3: empty (shoulder_pan, shoulder_lift, elbow_flex, wrist_flex)
  - Link 4 (wrist_roll / gripper body): fixed jaw tip sphere
  - Link 5 (gripper joint / moving_jaw body): moving jaw tip sphere

The moving jaw sphere is expressed in the jaw pivot frame, so it rotates with
the gripper joint angle. This allows the safety filter to track the jaw tip at
any gripper opening.

Run:
    .venv/bin/python tools/derive_so101_spheres.py
which (over)writes ``so101_collision_model.py`` at the repo root.
"""

from __future__ import annotations

from pathlib import Path

import mujoco
import numpy as np

REPO_ROOT = Path(__file__).resolve().parent.parent
SCENE_XML = REPO_ROOT / "robot_models" / "so101" / "scene.xml"
OUTPUT_PY = REPO_ROOT / "so101_collision_model.py"

NUM_LINKS = 6  # 5 arm joints + 1 gripper joint
SPHERE_RADIUS = 0.005  # 5mm for jaw tips (thin geometry)


def _find_tip_centroid(model, data, geom_id: int, ref_pos: np.ndarray) -> np.ndarray:
    """Find the centroid of the tip vertices (20% furthest from ref_pos) in world frame."""
    if int(model.geom_type[geom_id]) != int(mujoco.mjtGeom.mjGEOM_MESH):
        return np.array(data.geom_xpos[geom_id], dtype=float)

    mesh_id = int(model.geom_dataid[geom_id])
    addr = int(model.mesh_vertadr[mesh_id])
    nvert = int(model.mesh_vertnum[mesh_id])
    verts_local = np.array(model.mesh_vert[addr : addr + nvert], dtype=float)
    xmat = np.array(data.geom_xmat[geom_id], dtype=float).reshape(3, 3)
    xpos = np.array(data.geom_xpos[geom_id], dtype=float)
    verts_world = xpos + verts_local @ xmat.T

    dists = np.linalg.norm(verts_world - ref_pos, axis=1)
    n_tip = max(1, nvert // 5)
    tip_idx = np.argpartition(dists, -n_tip)[-n_tip:]
    return verts_world[tip_idx].mean(axis=0)


def _derive_fixed_jaw_sphere(model, data) -> tuple[np.ndarray, float]:
    """Derive the fixed jaw tip sphere center in oscbf link 4 (wrist_roll) frame."""
    gripper_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "gripper")
    gripper_geoms = [
        i for i in range(model.ngeom)
        if model.geom_bodyid[i] == gripper_body_id and model.geom_group[i] == 3
    ]
    best_gid = max(gripper_geoms, key=lambda g: model.geom_rbound[g])

    wrist_jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "wrist_roll")
    ref_pos = np.array(data.xpos[model.jnt_bodyid[wrist_jid]], dtype=float)
    tip_world = _find_tip_centroid(model, data, best_gid, ref_pos)

    # Express in oscbf link 4 frame
    import jax.numpy as jnp
    from test_so101_real import load_robot
    robot = load_robot()
    transforms = np.asarray(robot.joint_to_world_transforms(jnp.zeros(robot.num_joints)))
    t4 = transforms[4]
    r4, p4 = t4[:3, :3], t4[:3, 3]
    tip_in_link4 = r4.T @ (tip_world - p4)

    return tip_in_link4, SPHERE_RADIUS


def _derive_moving_jaw_sphere(model, data) -> tuple[np.ndarray, float]:
    """Derive the moving jaw tip sphere center in the jaw pivot (body) frame."""
    jaw_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "moving_jaw_so101_v1")
    jaw_geoms = [
        i for i in range(model.ngeom)
        if model.geom_bodyid[i] == jaw_body_id and model.geom_group[i] == 3
    ]
    gid = max(jaw_geoms, key=lambda g: model.geom_rbound[g])

    jaw_xpos = np.array(data.xpos[jaw_body_id], dtype=float)
    jaw_xmat = np.array(data.xmat[jaw_body_id], dtype=float).reshape(3, 3)
    tip_world = _find_tip_centroid(model, data, gid, jaw_xpos)

    # Express in jaw body frame (the pivot frame that rotates with gripper joint)
    tip_in_jaw = jaw_xmat.T @ (tip_world - jaw_xpos)

    return tip_in_jaw, SPHERE_RADIUS


def _derive_jaw_pivot_in_link4(model, data) -> tuple[np.ndarray, np.ndarray]:
    """Get the jaw pivot position and base rotation in oscbf link 4 frame.

    Returns (pivot_pos_in_link4, pivot_rot_in_link4) at gripper=0.
    The portable FK core uses these to compose the 6th joint transform.
    """
    jaw_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "moving_jaw_so101_v1")
    jaw_xpos = np.array(data.xpos[jaw_body_id], dtype=float)
    jaw_xmat = np.array(data.xmat[jaw_body_id], dtype=float).reshape(3, 3)

    import jax.numpy as jnp
    from test_so101_real import load_robot
    robot = load_robot()
    transforms = np.asarray(robot.joint_to_world_transforms(jnp.zeros(robot.num_joints)))
    t4 = transforms[4]
    r4, p4 = t4[:3, :3], t4[:3, 3]

    pivot_pos = r4.T @ (jaw_xpos - p4)
    pivot_rot = r4.T @ jaw_xmat

    return pivot_pos, pivot_rot


def derive() -> dict:
    model = mujoco.MjModel.from_xml_path(str(SCENE_XML))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    fixed_tip, fixed_r = _derive_fixed_jaw_sphere(model, data)
    moving_tip, moving_r = _derive_moving_jaw_sphere(model, data)
    pivot_pos, pivot_rot = _derive_jaw_pivot_in_link4(model, data)

    positions = [() for _ in range(NUM_LINKS)]
    radii = [() for _ in range(NUM_LINKS)]

    # Link 4: fixed jaw sphere (in oscbf link 4 / wrist_roll frame)
    positions[4] = (tuple(float(v) for v in fixed_tip),)
    radii[4] = (round(fixed_r, 6),)

    # Link 5: moving jaw sphere (in jaw pivot frame)
    positions[5] = (tuple(float(v) for v in moving_tip),)
    radii[5] = (round(moving_r, 6),)

    return {
        "positions": tuple(positions),
        "radii": tuple(radii),
        "jaw_pivot_in_link4": tuple(float(v) for v in pivot_pos),
        "jaw_pivot_rot_in_link4": tuple(tuple(float(v) for v in row) for row in pivot_rot),
        "jaw_joint_axis": (0.0, 0.0, 1.0),  # gripper joint axis in jaw frame
    }


def _format_data(data: dict) -> str:
    lines = [
        '"""SO101 gripper collision spheres (auto-generated).',
        "",
        "Generated by tools/derive_so101_spheres.py from robot_models/so101/scene.xml.",
        "Do not edit by hand; re-run the tool instead.",
        "",
        "6-link model: links 0-4 match oscbf's 5-joint chain (shoulder_pan through",
        "wrist_roll). Link 5 is the gripper joint (moving jaw). The moving jaw sphere",
        "is expressed in the jaw pivot frame and rotates with the gripper angle.",
        "",
        "Format:",
        '    positions[link][sphere_idx] = (x, y, z) in the link\'s local frame',
        '    radii[link][sphere_idx] = radius',
        '    jaw_pivot_in_link4 = position of jaw pivot in oscbf link 4 frame',
        '    jaw_pivot_rot_in_link4 = rotation of jaw pivot in oscbf link 4 frame (at gripper=0)',
        '    jaw_joint_axis = joint axis in jaw pivot frame',
        '"""',
        "",
        "so101_collision_data = {",
        '    "positions": (',
    ]
    for link_positions in data["positions"]:
        if not link_positions:
            lines.append("        (),")
        else:
            inner = ", ".join(
                "(" + ", ".join(f"{v:.6f}" for v in pt) + ")" for pt in link_positions
            )
            lines.append(f"        ({inner},),")
    lines.append("    ),")
    lines.append('    "radii": (')
    for link_radii in data["radii"]:
        if not link_radii:
            lines.append("        (),")
        else:
            inner = ", ".join(f"{r:.6f}" for r in link_radii)
            lines.append(f"        ({inner},),")
    lines.append("    ),")

    piv = data["jaw_pivot_in_link4"]
    lines.append(f'    "jaw_pivot_in_link4": ({piv[0]:.6f}, {piv[1]:.6f}, {piv[2]:.6f}),')
    rot = data["jaw_pivot_rot_in_link4"]
    lines.append('    "jaw_pivot_rot_in_link4": (')
    for row in rot:
        lines.append(f"        ({row[0]:.6f}, {row[1]:.6f}, {row[2]:.6f}),")
    lines.append("    ),")
    axis = data["jaw_joint_axis"]
    lines.append(f'    "jaw_joint_axis": ({axis[0]:.1f}, {axis[1]:.1f}, {axis[2]:.1f}),')
    lines.append("}")
    lines.append("")
    return "\n".join(lines)


def main() -> None:
    data = derive()
    OUTPUT_PY.write_text(_format_data(data))
    n = sum(len(r) for r in data["radii"])
    print(f"Wrote {OUTPUT_PY.relative_to(REPO_ROOT)} with {n} spheres (6 links):")
    for link_idx, (pos, rad) in enumerate(zip(data["positions"], data["radii"])):
        if rad:
            for p, r in zip(pos, rad):
                label = "fixed jaw" if link_idx == 4 else "moving jaw"
                print(f"  link {link_idx} ({label}): center={np.round(p, 4)} r={r:.4f}")
    piv = data["jaw_pivot_in_link4"]
    print(f"  jaw pivot in link 4: ({piv[0]:.4f}, {piv[1]:.4f}, {piv[2]:.4f})")


if __name__ == "__main__":
    main()
