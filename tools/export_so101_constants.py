#!/usr/bin/env python
"""Export SO101 FK constants + sphere data to a numpy-only constants module.

Run this offline (requires JAX + oscbf) whenever the robot model changes:
    python tools/export_so101_constants.py

Writes: so101_safety/so101_safety/constants.py
"""
from __future__ import annotations

import os
import sys
import textwrap

import numpy as np

# Ensure the repo root is on the path so we can import so101_robot / test_so101_real.
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from so101_robot import load_so101_robot  # noqa: E402 (needs sys.path set above)


def _array_literal(arr: np.ndarray, indent: int = 4) -> str:
    """Format a numpy array as a compact np.array(...) literal."""
    prefix = " " * indent
    return np.array2string(
        arr,
        separator=", ",
        max_line_width=100,
        precision=10,
        floatmode="unique",
        prefix=prefix,
    )


def main() -> None:
    robot = load_so101_robot()

    joint_to_prev = np.array(robot.joint_to_prev_joint_tfs)        # (5, 4, 4)
    ee_offset = np.array(robot.ee_offset)                           # (4, 4)
    padded_pos = np.array(robot.padded_collision_positions)         # (5, k, 3)
    padded_radii = np.array(robot.padded_collision_radii)           # (5, k)
    slice_idx = np.array(list(robot.collision_slice_indices))       # (n_spheres,)
    joint_axes = np.array(robot.joint_axes)                         # (5, 3)
    joint_types = np.array(robot.joint_types)                       # (5,) — all 0 revolute

    # Derive which link (joint index) owns each active sphere.
    # Flattened index = link_idx * k + sphere_in_link → link_idx = index // k
    k = padded_pos.shape[1]
    sphere_parent_joints = (slice_idx // k).astype(np.int32)        # (n_spheres,)

    out_path = os.path.join(
        os.path.dirname(__file__), "..", "so101_safety", "so101_safety", "constants.py"
    )
    out_path = os.path.normpath(out_path)

    lines = [
        '"""Auto-generated SO101 FK + sphere constants — DO NOT EDIT.',
        "",
        "Re-generate with:  python tools/export_so101_constants.py",
        '"""',
        "from __future__ import annotations",
        "",
        "import numpy as np",
        "",
        "# Fixed transform from joint i-1 frame to joint i origin (at zero config).",
        "# Shape: (5, 4, 4)",
        f"JOINT_TO_PREV_JOINT_TFS: np.ndarray = np.array({_array_literal(joint_to_prev)})",
        "",
        "# End-effector offset from the last joint frame.",
        "# Shape: (4, 4)",
        f"EE_OFFSET: np.ndarray = np.array({_array_literal(ee_offset)})",
        "",
        "# Joint rotation axes in their own frame (all z for SO101).",
        "# Shape: (5, 3)",
        f"JOINT_AXES: np.ndarray = np.array({_array_literal(joint_axes)})",
        "",
        "# Joint types: 0 = revolute, 1 = prismatic.",
        "# Shape: (5,)",
        f"JOINT_TYPES: np.ndarray = np.array({_array_literal(joint_types)})",
        "",
        "# Collision sphere local positions, padded to uniform shape.",
        "# Shape: (num_joints, max_spheres_per_link, 3)",
        f"PADDED_COLLISION_POSITIONS: np.ndarray = np.array({_array_literal(padded_pos)})",
        "",
        "# Collision sphere radii, padded.",
        "# Shape: (num_joints, max_spheres_per_link)",
        f"PADDED_COLLISION_RADII: np.ndarray = np.array({_array_literal(padded_radii)})",
        "",
        "# Indices into the flattened (num_joints * max_spheres_per_link) arrays",
        "# that select the real (non-padded) spheres.",
        "# Shape: (n_spheres,)",
        f"COLLISION_SLICE_INDICES: np.ndarray = np.array({_array_literal(slice_idx)})",
        "",
        "# Joint index of the parent link for each active sphere.",
        "# Shape: (n_spheres,)",
        f"SPHERE_PARENT_JOINTS: np.ndarray = np.array({_array_literal(sphere_parent_joints)})",
        "",
        f"NUM_JOINTS: int = {int(robot.num_joints)}",
        f"NUM_SPHERES: int = {int(len(slice_idx))}",
        "",
    ]

    with open(out_path, "w") as f:
        f.write("\n".join(lines))

    print(f"Written: {out_path}")
    print(f"  {robot.num_joints} joints, {len(slice_idx)} active spheres")
    print(f"  padded shape: {padded_pos.shape}, slice_indices: {slice_idx}")
    print(f"  sphere_parent_joints: {sphere_parent_joints}")


if __name__ == "__main__":
    main()
