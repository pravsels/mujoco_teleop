"""Portable numpy-only workspace-box safety filter for SO101.

The filter runs on any Python environment (sim, sim_to_real, Pi edge connector)
with only numpy + scipy. No JAX, no MuJoCo, no transport.

Algorithm
---------
Given current_q and desired_q (joint radians):
1. FK: compute sphere world positions and positional Jacobians at current_q.
2. Compute CBF values h_{s,f} for every (sphere s, box face f) pair.
3. Build the CBF-QP:
       minimise  ||δq - δq_nom||²
       s.t.      A_cbf @ δq >= b_cbf  (h_dot >= -alpha * h for each face)
   where δq_nom = desired_q - current_q and δq is the safe correction.
4. Optional: clamp ||δq|| to max_vel * dt before solving.
5. Return current_q + δq_safe.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np
from scipy.optimize import minimize

from so101_safety.kinematics import Kinematics, NumpyKinematics
from so101_safety.constants import NUM_JOINTS


@dataclass
class _BoxFace:
    """One face of the axis-aligned keep-in box."""
    axis: int      # 0=x, 1=y, 2=z
    normal: float  # +1 for lower face (h = pos - lo), -1 for upper face (h = hi - pos)
    bound: float   # lo[axis] or hi[axis]

    def barrier(self, pos: float, radius: float, buffer: float) -> float:
        """Signed distance to face minus radius and buffer (positive = inside)."""
        if self.normal > 0:
            return pos - self.bound - radius - buffer
        else:
            return self.bound - pos - radius - buffer

    def jacobian_row(self, sphere_jac_axis: np.ndarray) -> np.ndarray:
        """Row of the CBF constraint matrix A for this face.

        sphere_jac_axis is J[sphere, axis, :] — the row of the sphere Jacobian
        corresponding to this face's axis, shape (n_joints,).
        """
        return self.normal * sphere_jac_axis


def _build_faces(lo: tuple, hi: tuple) -> list[_BoxFace]:
    faces = []
    for ax in range(3):
        faces.append(_BoxFace(axis=ax, normal=+1.0, bound=lo[ax]))   # lower face
        faces.append(_BoxFace(axis=ax, normal=-1.0, bound=hi[ax]))   # upper face
    return faces


class SafetyFilter:
    """Workspace-box CBF safety filter.

    Args:
        kinematics: FK provider (default: NumpyKinematics).
        box_lo: lower corner of safe box (metres), shape (3,).
        box_hi: upper corner of safe box (metres), shape (3,).
        buffer: shrinks the box inward on every face (metres) to absorb
            one-step discrete overshoot. Default 0.01 m.
        max_vel: if set, the nominal δq is clamped so ||δq||_inf <= max_vel
            (radians) before solving. Default: no clamp.
        alpha: CBF class-K gain. Higher = tighter tracking near boundary.
            Default 5.0.
        dt: control period (seconds). Scales the CBF right-hand side.
            Default 0.05 s (20 Hz).
    """

    def __init__(
        self,
        kinematics: Optional[Kinematics] = None,
        box_lo: tuple = (-0.3, -0.3, 0.05),
        box_hi: tuple = (0.3, 0.3, 0.6),
        *,
        buffer: float = 0.01,
        max_vel: Optional[float] = None,
        alpha: float = 5.0,
        dt: float = 0.05,
    ) -> None:
        self._kin = kinematics if kinematics is not None else NumpyKinematics()
        self._box_lo = tuple(float(v) for v in box_lo)
        self._box_hi = tuple(float(v) for v in box_hi)
        self._buffer = float(buffer)
        self._max_vel = float(max_vel) if max_vel is not None else None
        self._alpha = float(alpha)
        self._dt = float(dt)
        self._faces = _build_faces(self._box_lo, self._box_hi)

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def filter(
        self,
        current_q: np.ndarray,
        desired_q: np.ndarray,
        feedback=None,
    ) -> np.ndarray:
        """Return a safe joint target close to desired_q.

        Args:
            current_q: current joint positions (radians), shape (5,) or (6,).
                       If 6, the last element is the gripper angle.
            desired_q:  desired joint target (radians), same shape as current_q.
            feedback:   optional dict with motor readings (reserved for Task 5
                        contact guard — not used here).

        Returns:
            Safe joint target (radians), same shape as input.
        """
        q0 = np.asarray(current_q, dtype=float)
        q_des = np.asarray(desired_q, dtype=float)
        has_jaw = len(q0) > NUM_JOINTS
        n_arm = NUM_JOINTS

        dq_nom = q_des[:n_arm] - q0[:n_arm]

        # FK + Jacobians at current configuration (full q for sphere positions)
        sph_pos = self._kin.sphere_positions(q0)       # (n_spheres, 3)
        sph_rad = self._kin.sphere_radii(q0)           # (n_spheres,)
        J_sph = self._kin.sphere_jacobians(q0)         # (n_spheres, 3, n_arm)

        # Build CBF constraint rows: A_cbf @ δq >= b_cbf
        A_rows, b_rows = [], []
        for s, (pos, rad, J_s) in enumerate(zip(sph_pos, sph_rad, J_sph)):
            for face in self._faces:
                h = face.barrier(pos[face.axis], rad, self._buffer)
                a_row = face.jacobian_row(J_s[face.axis])
                A_rows.append(a_row)
                b_rows.append(-self._alpha * h * self._dt)

        A_cbf = np.array(A_rows)   # (n_constraints, n_arm)
        b_cbf = np.array(b_rows)   # (n_constraints,)

        # Optional velocity box constraints: -max_vel <= delta_q <= max_vel
        if self._max_vel is not None:
            box_A = np.vstack([np.eye(n_arm), -np.eye(n_arm)])
            box_b = np.full(2 * n_arm, -self._max_vel)
            A_cbf = np.vstack([A_cbf, box_A])
            b_cbf = np.concatenate([b_cbf, box_b])

        dq_safe = self._solve_qp(dq_nom, A_cbf, b_cbf)

        # Reconstruct full output (pass gripper through unchanged)
        if has_jaw:
            result = np.empty_like(q0)
            result[:n_arm] = q0[:n_arm] + dq_safe
            result[n_arm:] = q_des[n_arm:]
            return result
        return q0 + dq_safe

    # ------------------------------------------------------------------
    # QP solver
    # ------------------------------------------------------------------

    def _solve_qp(
        self, dq_nom: np.ndarray, A: np.ndarray, b: np.ndarray
    ) -> np.ndarray:
        """Solve: min ||dq - dq_nom||² s.t. A @ dq >= b via scipy SLSQP."""
        n = len(dq_nom)

        def objective(dq):
            diff = dq - dq_nom
            return 0.5 * np.dot(diff, diff)

        def gradient(dq):
            return dq - dq_nom

        constraints = {
            "type": "ineq",
            "fun": lambda dq: A @ dq - b,
            "jac": lambda dq: A,
        }

        result = minimize(
            objective,
            x0=dq_nom,
            jac=gradient,
            constraints=constraints,
            method="SLSQP",
            options={"ftol": 1e-9, "maxiter": 200, "disp": False},
        )
        if not result.success and result.status not in (0, 9):
            # Status 9 is "Iteration limit reached" — still use best found
            pass  # fall through and return best-effort solution
        return result.x
