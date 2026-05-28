"""Optional MuJoCo-backed kinematics for SO101.

Import this only when mujoco is available (sim/dev); it must never be imported
by the portable core directly. Use NumpyKinematics for the Pi runtime.
"""
from __future__ import annotations

import numpy as np

from so101_safety.constants import (
    COLLISION_SLICE_INDICES,
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

        # Cache radii
        flat_radii = PADDED_COLLISION_RADII.ravel()
        self._radii = flat_radii[COLLISION_SLICE_INDICES].copy()

    def _set_qpos(self, q: np.ndarray) -> None:
        q = np.asarray(q, dtype=float)
        for chain_idx, jid in enumerate(self._joint_ids):
            self._data.qpos[self._model.jnt_qposadr[jid]] = q[chain_idx]
        self._mujoco.mj_kinematics(self._model, self._data)

    def ee_position(self, q: np.ndarray) -> np.ndarray:
        """World-frame EE position. Shape: (3,)."""
        self._set_qpos(q)
        # Use the last joint's child body xpos as an approximation, then apply
        # the ee_offset (identity for SO101). We find the body of the last joint.
        last_jid = self._joint_ids[-1]
        body_id = self._model.jnt_bodyid[last_jid]
        # Walk to the last body in the chain (child of last joint)
        # For SO101, the EE is the child body of WristRoll
        body_pos = self._data.xpos[body_id].copy()
        # Apply ee_offset (identity for SO101, so just return body pos)
        # More precisely: T_world_ee = T_world_last_joint @ ee_offset
        # ee_offset = identity → EE origin = last joint body origin
        return body_pos

    def sphere_positions(self, q: np.ndarray) -> np.ndarray:
        """World-frame sphere centres from MuJoCo xpos. Shape: (n_spheres, 3)."""
        self._set_qpos(q)

        # Replicate the same padded→slice logic as NumpyKinematics, but use
        # mujoco body transforms instead of the DH chain.
        # Map joint index 3 → body of joint 3, joint index 4 → body of joint 4.
        parent_jids = [self._joint_ids[int(pj)] for pj in SPHERE_PARENT_JOINTS]

        positions = np.empty((len(COLLISION_SLICE_INDICES), 3))
        for s, pjid in enumerate(parent_jids):
            body_id = self._model.jnt_bodyid[pjid]
            R = self._data.xmat[body_id].reshape(3, 3)
            t = self._data.xpos[body_id]
            # Sphere local position in the link/joint frame from constants
            flat_idx = int(COLLISION_SLICE_INDICES[s])
            k = PADDED_COLLISION_POSITIONS.shape[1]
            link_idx = flat_idx // k
            sph_in_link = flat_idx % k
            local_pos = PADDED_COLLISION_POSITIONS[link_idx, sph_in_link]
            positions[s] = R @ local_pos + t
        return positions

    def sphere_radii(self) -> np.ndarray:
        """Sphere radii (constant). Shape: (n_spheres,)."""
        return self._radii.copy()

    def sphere_jacobians(self, q: np.ndarray) -> np.ndarray:
        """Positional Jacobian of each sphere w.r.t. q. Shape: (n_spheres, 3, n_joints)."""
        # Use NumpyKinematics for the Jacobian — it is analytical and the parity
        # test only checks sphere_positions and ee_position via MuJoCo.
        from so101_safety.kinematics import NumpyKinematics
        return NumpyKinematics().sphere_jacobians(q)
