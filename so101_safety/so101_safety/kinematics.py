"""Kinematics protocol and numpy-only FK implementation for SO101.

No JAX, no torch, no transport — safe to import on the Pi.
"""
from __future__ import annotations

from typing import Protocol, runtime_checkable

import numpy as np

from so101_safety.constants import (
    COLLISION_SLICE_INDICES,
    EE_OFFSET,
    JAW_JOINT_AXIS,
    JAW_PIVOT_IN_LINK4,
    JAW_PIVOT_ROT_IN_LINK4,
    JAW_SPHERE_LOCAL,
    JAW_SPHERE_RADIUS,
    JOINT_TO_PREV_JOINT_TFS,
    NUM_JOINTS,
    PADDED_COLLISION_POSITIONS,
    PADDED_COLLISION_RADII,
    SPHERE_PARENT_JOINTS,
)


@runtime_checkable
class Kinematics(Protocol):
    """Minimal FK interface required by SafetyFilter."""

    def sphere_positions(self, q: np.ndarray) -> np.ndarray:
        """World-frame positions of gripper collision spheres. Shape: (n_spheres, 3)."""
        ...

    def sphere_radii(self, q: np.ndarray | None = None) -> np.ndarray:
        """Sphere radii. Shape: (n_spheres,). q needed if sphere count varies."""
        ...

    def ee_position(self, q: np.ndarray) -> np.ndarray:
        """World-frame end-effector position. Shape: (3,)."""
        ...

    def sphere_jacobians(self, q: np.ndarray) -> np.ndarray:
        """Positional Jacobian of each sphere w.r.t. arm joints. Shape: (n_spheres, 3, n_arm_joints)."""
        ...


def _revolute_transform(q: float, axis: np.ndarray) -> np.ndarray:
    """4×4 revolute joint transform (child → parent), matching oscbf's formulation."""
    axis = axis / np.linalg.norm(axis)
    a1, a2, a3 = axis
    c = np.cos(q)
    s = np.sin(q)
    t = 1.0 - c
    return np.array(
        [
            [t * a1 * a1 + c,       t * a1 * a2 - s * a3, t * a1 * a3 + s * a2, 0.0],
            [t * a1 * a2 + s * a3,  t * a2 * a2 + c,       t * a2 * a3 - s * a1, 0.0],
            [t * a1 * a3 - s * a2,  t * a2 * a3 + s * a1,  t * a3 * a3 + c,      0.0],
            [0.0,                   0.0,                   0.0,                  1.0],
        ]
    )


class NumpyKinematics:
    """Pure-numpy FK for SO101, matching the oscbf Manipulator chain exactly.

    Uses pre-exported DH constants so no mujoco or JAX is needed at runtime.
    Supports 5-joint (arm only) or 6-joint (arm + gripper) input. When 6 joints
    are provided, the moving jaw sphere is included in sphere outputs.
    """

    _RADII: np.ndarray  # cached constant
    _AXES: np.ndarray

    def __init__(self) -> None:
        flat_radii = PADDED_COLLISION_RADII.ravel()
        self._arm_radii = flat_radii[COLLISION_SLICE_INDICES].copy()

    # ------------------------------------------------------------------
    # Core FK
    # ------------------------------------------------------------------

    def _joint_transforms(self, q: np.ndarray) -> np.ndarray:
        """Return cumulative joint-to-world transforms. Shape: (n_joints, 4, 4)."""
        q = np.asarray(q, dtype=float)
        n = NUM_JOINTS
        transforms = np.empty((n, 4, 4))
        for i in range(n):
            T_joint = _revolute_transform(q[i], np.array([0.0, 0.0, 1.0]))
            transforms[i] = JOINT_TO_PREV_JOINT_TFS[i] @ T_joint

        cumulative = np.empty_like(transforms)
        cumulative[0] = transforms[0]
        for i in range(1, n):
            cumulative[i] = cumulative[i - 1] @ transforms[i]
        return cumulative

    def _jaw_sphere_world(self, cumulative: np.ndarray, q_jaw: float) -> np.ndarray:
        """Compute world-frame position of the moving jaw sphere."""
        T_link4 = cumulative[4]  # world → link 4

        # Build 4×4 pivot transform in link 4 frame
        T_pivot = np.eye(4)
        T_pivot[:3, :3] = JAW_PIVOT_ROT_IN_LINK4
        T_pivot[:3, 3] = JAW_PIVOT_IN_LINK4

        # Jaw joint rotation
        T_jaw_rot = _revolute_transform(q_jaw, JAW_JOINT_AXIS)

        # Full chain: world → link4 → pivot → jaw_rotation → local sphere
        T_world_jaw = T_link4 @ T_pivot @ T_jaw_rot
        pos_h = np.array([*JAW_SPHERE_LOCAL, 1.0])
        return (T_world_jaw @ pos_h)[:3]

    # ------------------------------------------------------------------
    # Kinematics protocol implementation
    # ------------------------------------------------------------------

    def ee_position(self, q: np.ndarray) -> np.ndarray:
        """World-frame EE position. Shape: (3,)."""
        q = np.asarray(q, dtype=float)
        transforms = self._joint_transforms(q[:NUM_JOINTS])
        ee_tf = transforms[-1] @ EE_OFFSET
        return ee_tf[:3, 3]

    def sphere_positions(self, q: np.ndarray) -> np.ndarray:
        """World-frame sphere centres. Shape: (n_spheres, 3).

        If q has 6 elements, the jaw sphere is appended.
        """
        q = np.asarray(q, dtype=float)
        transforms = self._joint_transforms(q[:NUM_JOINTS])
        arm_spheres = self._arm_sphere_positions(transforms)

        if len(q) > NUM_JOINTS:
            jaw_pos = self._jaw_sphere_world(transforms, q[NUM_JOINTS])
            return np.vstack([arm_spheres, jaw_pos[np.newaxis, :]])
        return arm_spheres

    def sphere_radii(self, q: np.ndarray | None = None) -> np.ndarray:
        """Sphere radii (constant). Shape: (n_spheres,).

        If q is provided and has 6 elements, jaw sphere radius is appended.
        """
        if q is not None and len(np.asarray(q)) > NUM_JOINTS:
            return np.append(self._arm_radii, JAW_SPHERE_RADIUS)
        return self._arm_radii.copy()

    def sphere_jacobians(self, q: np.ndarray) -> np.ndarray:
        """Positional Jacobians of each sphere w.r.t. arm joints (first 5).

        Uses the standard revolute formula: J[:, j] = z_j × (p_s - o_j)
        for joints j that are upstream of sphere s, else 0.

        Returns shape: (n_spheres, 3, NUM_JOINTS).
        """
        q = np.asarray(q, dtype=float)
        transforms = self._joint_transforms(q[:NUM_JOINTS])
        arm_spheres = self._arm_sphere_positions(transforms)

        has_jaw = len(q) > NUM_JOINTS
        if has_jaw:
            jaw_pos = self._jaw_sphere_world(transforms, q[NUM_JOINTS])
            all_pos = np.vstack([arm_spheres, jaw_pos[np.newaxis, :]])
            # Jaw sphere is downstream of all 5 arm joints (attached to link 4)
            parent_joints = np.append(SPHERE_PARENT_JOINTS, 4)
        else:
            all_pos = arm_spheres
            parent_joints = SPHERE_PARENT_JOINTS

        n_sph = len(all_pos)
        J = np.zeros((n_sph, 3, NUM_JOINTS))

        for s in range(n_sph):
            parent = int(parent_joints[s])
            p_s = all_pos[s]
            for j in range(parent + 1):
                z_j = transforms[j, :3, 2]
                o_j = transforms[j, :3, 3]
                J[s, :, j] = np.cross(z_j, p_s - o_j)

        return J

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _arm_sphere_positions(self, transforms: np.ndarray) -> np.ndarray:
        """Compute arm sphere world positions from pre-computed transforms."""
        ones = np.ones((*PADDED_COLLISION_POSITIONS.shape[:2], 1))
        pos_h = np.concatenate([PADDED_COLLISION_POSITIONS, ones], axis=-1)
        pos_world_h = np.einsum("nij,nkj->nki", transforms, pos_h)
        all_pts = pos_world_h[:, :, :3].reshape(-1, 3)
        return all_pts[COLLISION_SLICE_INDICES]
