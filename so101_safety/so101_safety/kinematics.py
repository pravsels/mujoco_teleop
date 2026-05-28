"""Kinematics protocol and numpy-only FK implementation for SO101.

No JAX, no torch, no transport — safe to import on the Pi.
"""
from __future__ import annotations

from typing import Protocol, runtime_checkable

import numpy as np

from so101_safety.constants import (
    COLLISION_SLICE_INDICES,
    EE_OFFSET,
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

    def sphere_radii(self) -> np.ndarray:
        """Sphere radii (constant). Shape: (n_spheres,)."""
        ...

    def ee_position(self, q: np.ndarray) -> np.ndarray:
        """World-frame end-effector position. Shape: (3,)."""
        ...

    def sphere_jacobians(self, q: np.ndarray) -> np.ndarray:
        """Positional Jacobian of each sphere w.r.t. q. Shape: (n_spheres, 3, n_joints)."""
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
    """

    _RADII: np.ndarray  # cached constant
    _AXES: np.ndarray

    def __init__(self) -> None:
        # Build the flat radii array once (select non-padded entries)
        flat_radii = PADDED_COLLISION_RADII.ravel()
        self._RADII = flat_radii[COLLISION_SLICE_INDICES].copy()

    # ------------------------------------------------------------------
    # Core FK
    # ------------------------------------------------------------------

    def _joint_transforms(self, q: np.ndarray) -> np.ndarray:
        """Return cumulative joint-to-world transforms. Shape: (n_joints, 4, 4)."""
        q = np.asarray(q, dtype=float)
        n = NUM_JOINTS
        transforms = np.empty((n, 4, 4))
        for i in range(n):
            axis = JOINT_TO_PREV_JOINT_TFS[i, :3, :3].T @ np.array([0.0, 0.0, 1.0])
            # joint axis in world coordinates doesn't matter here; we use the local
            # axis (0,0,1) because joint_transform is in the local frame before
            # being composed with the fixed transform.
            T_joint = _revolute_transform(q[i], np.array([0.0, 0.0, 1.0]))
            transforms[i] = JOINT_TO_PREV_JOINT_TFS[i] @ T_joint

        # Cumulative product: T_world_joint_i = T_0 @ T_1 @ ... @ T_i
        cumulative = np.empty_like(transforms)
        cumulative[0] = transforms[0]
        for i in range(1, n):
            cumulative[i] = cumulative[i - 1] @ transforms[i]
        return cumulative

    # ------------------------------------------------------------------
    # Kinematics protocol implementation
    # ------------------------------------------------------------------

    def ee_position(self, q: np.ndarray) -> np.ndarray:
        """World-frame EE position. Shape: (3,)."""
        transforms = self._joint_transforms(q)
        ee_tf = transforms[-1] @ EE_OFFSET
        return ee_tf[:3, 3]

    def sphere_positions(self, q: np.ndarray) -> np.ndarray:
        """World-frame sphere centres. Shape: (n_spheres, 3)."""
        transforms = self._joint_transforms(q)
        # Homogeneous padded positions: (n_joints, max_k, 4)
        ones = np.ones((*PADDED_COLLISION_POSITIONS.shape[:2], 1))
        pos_h = np.concatenate([PADDED_COLLISION_POSITIONS, ones], axis=-1)
        # Apply: result[n, k] = transforms[n] @ pos_h[n, k]
        # einsum 'nij, nkj -> nki' where the last axis is i (output row)
        pos_world_h = np.einsum("nij,nkj->nki", transforms, pos_h)  # (n, k, 4)
        all_pts = pos_world_h[:, :, :3].reshape(-1, 3)               # (n*k, 3)
        return all_pts[COLLISION_SLICE_INDICES]                       # (n_spheres, 3)

    def sphere_radii(self) -> np.ndarray:
        """Sphere radii (constant). Shape: (n_spheres,)."""
        return self._RADII.copy()

    def sphere_jacobians(self, q: np.ndarray) -> np.ndarray:
        """Positional Jacobians of each sphere w.r.t. q.

        Uses the standard revolute formula: J[:, j] = z_j × (p_s - o_j)
        for joints j that are upstream of sphere s, else 0.

        Returns shape: (n_spheres, 3, n_joints).
        """
        transforms = self._joint_transforms(q)
        sph_pos = self._sphere_positions_from_transforms(transforms)  # (n_spheres, 3)

        n_sph = len(COLLISION_SLICE_INDICES)
        J = np.zeros((n_sph, 3, NUM_JOINTS))

        for s in range(n_sph):
            parent = int(SPHERE_PARENT_JOINTS[s])
            p_s = sph_pos[s]
            for j in range(parent + 1):  # joints 0..parent are upstream
                z_j = transforms[j, :3, 2]   # z-axis of joint j in world frame
                o_j = transforms[j, :3, 3]   # origin of joint j in world frame
                J[s, :, j] = np.cross(z_j, p_s - o_j)

        return J

    def _sphere_positions_from_transforms(self, transforms: np.ndarray) -> np.ndarray:
        """Helper: compute sphere positions given pre-computed transforms."""
        ones = np.ones((*PADDED_COLLISION_POSITIONS.shape[:2], 1))
        pos_h = np.concatenate([PADDED_COLLISION_POSITIONS, ones], axis=-1)
        pos_world_h = np.einsum("nij,nkj->nki", transforms, pos_h)
        all_pts = pos_world_h[:, :, :3].reshape(-1, 3)
        return all_pts[COLLISION_SLICE_INDICES]
