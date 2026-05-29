"""Optional MuJoCo-backed kinematics for SO101.

Import this only when mujoco is available (sim/dev); it must never be imported
by the portable core directly. Use NumpyKinematics for the Pi runtime.
"""
from __future__ import annotations

import numpy as np

from so101_safety.constants import (
    COLLISION_SLICE_INDICES,
    JAW_JOINT_AXIS,
    JAW_PIVOT_IN_LINK4,
    JAW_PIVOT_ROT_IN_LINK4,
    JAW_SPHERE_LOCAL,
    JAW_SPHERE_RADIUS,
    PADDED_COLLISION_POSITIONS,
    PADDED_COLLISION_RADII,
    SPHERE_PARENT_JOINTS,
    NUM_JOINTS,
)


# Lazy import so the module can be imported without mujoco installed (will
# fail only when MujocoKinematics is instantiated).
def _require_mujoco():
    try:
        import mujoco
        return mujoco
    except ImportError as exc:
        raise ImportError(
            "mujoco is required for MujocoKinematics. "
            "Install it with: pip install mujoco"
        ) from exc


# Joint names in the SO101 MuJoCo model, in oscbf chain order.
_SO101_JOINT_NAMES = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
]


class MujocoKinematics:
    """FK backed by a live MuJoCo model — used in sim/dev for parity checking.

    Args:
        xml_path: path to the SO101 MuJoCo scene XML.
    """

    def __init__(self, xml_path: str) -> None:
        mujoco = _require_mujoco()
        self._model = mujoco.MjModel.from_xml_path(xml_path)
        self._data = mujoco.MjData(self._model)
        self._mujoco = mujoco

        # Resolve joint IDs once
        self._joint_ids = [
            mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_JOINT, name)
            for name in _SO101_JOINT_NAMES
        ]
        assert all(jid >= 0 for jid in self._joint_ids), (
            f"Some joint names not found in model. "
            f"Expected: {_SO101_JOINT_NAMES}, got ids: {self._joint_ids}"
        )

        # Resolve gripper joint
        self._gripper_jid = mujoco.mj_name2id(
            self._model, mujoco.mjtObj.mjOBJ_JOINT, "gripper"
        )

        # Cache radii
        flat_radii = PADDED_COLLISION_RADII.ravel()
        self._arm_radii = flat_radii[COLLISION_SLICE_INDICES].copy()

    def _set_qpos(self, q: np.ndarray) -> None:
        q = np.asarray(q, dtype=float)
        for chain_idx, jid in enumerate(self._joint_ids):
            self._data.qpos[self._model.jnt_qposadr[jid]] = q[chain_idx]
        if len(q) > NUM_JOINTS and self._gripper_jid >= 0:
            self._data.qpos[self._model.jnt_qposadr[self._gripper_jid]] = q[NUM_JOINTS]
        self._mujoco.mj_kinematics(self._model, self._data)

    def ee_position(self, q: np.ndarray) -> np.ndarray:
        """World-frame EE position. Shape: (3,)."""
        self._set_qpos(q)
        last_jid = self._joint_ids[-1]
        body_id = self._model.jnt_bodyid[last_jid]
        return self._data.xpos[body_id].copy()

    def sphere_positions(self, q: np.ndarray) -> np.ndarray:
        """World-frame sphere centres. Shape: (n_spheres, 3)."""
        self._set_qpos(q)
        q = np.asarray(q, dtype=float)

        # Arm spheres via body transforms
        parent_jids = [self._joint_ids[int(pj)] for pj in SPHERE_PARENT_JOINTS]
        arm_positions = np.empty((len(COLLISION_SLICE_INDICES), 3))
        for s, pjid in enumerate(parent_jids):
            body_id = self._model.jnt_bodyid[pjid]
            R = self._data.xmat[body_id].reshape(3, 3)
            t = self._data.xpos[body_id]
            flat_idx = int(COLLISION_SLICE_INDICES[s])
            k = PADDED_COLLISION_POSITIONS.shape[1]
            link_idx = flat_idx // k
            sph_in_link = flat_idx % k
            local_pos = PADDED_COLLISION_POSITIONS[link_idx, sph_in_link]
            arm_positions[s] = R @ local_pos + t

        if len(q) > NUM_JOINTS and self._gripper_jid >= 0:
            # Jaw sphere: use the gripper body's live transform
            jaw_body_id = self._model.jnt_bodyid[self._gripper_jid]
            R_jaw = self._data.xmat[jaw_body_id].reshape(3, 3)
            t_jaw = self._data.xpos[jaw_body_id]
            jaw_world = R_jaw @ JAW_SPHERE_LOCAL + t_jaw
            return np.vstack([arm_positions, jaw_world[np.newaxis, :]])

        return arm_positions

    def sphere_radii(self, q: np.ndarray | None = None) -> np.ndarray:
        """Sphere radii. Shape: (n_spheres,)."""
        if q is not None and len(np.asarray(q)) > NUM_JOINTS:
            return np.append(self._arm_radii, JAW_SPHERE_RADIUS)
        return self._arm_radii.copy()

    def sphere_jacobians(self, q: np.ndarray) -> np.ndarray:
        """Positional Jacobian of each sphere w.r.t. arm joints. Shape: (n_spheres, 3, NUM_JOINTS)."""
        from so101_safety.kinematics import NumpyKinematics
        return NumpyKinematics().sphere_jacobians(q)
