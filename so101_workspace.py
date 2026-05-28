"""SO101 workspace-containment CBF (keep-in box: floor + 4 walls + ceiling).

Velocity-mode OSCBF config that keeps every gripper collision sphere inside an
axis-aligned box ``[lo, hi]``. Unlike ``TableAvoidanceConfig`` (a single
end-effector point above one plane), this constrains the whole gripper cluster
against all six box faces, which is what lets the filter handle a test rig with
walls, not just a table.

oscbf is a read-only dependency; this SO101-specific config lives here.
"""

from __future__ import annotations

import jax
import jax.numpy as jnp

from oscbf.core.manipulator import Manipulator
from oscbf.core.oscbf_configs import OSCBFVelocityConfig


@jax.tree_util.register_static
class WorkspaceContainmentConfig(OSCBFVelocityConfig):
    """Keep all SO101 gripper spheres inside an axis-aligned safe box.

    Args:
        robot: manipulator loaded with gripper collision spheres.
        lo, hi: box corners (meters) in the robot base frame, as 3-tuples.
        buffer: shrinks the box inward on every face (meters) to absorb
            one-step discrete overshoot at the control rate.
    """

    def __init__(
        self,
        robot: Manipulator,
        lo: tuple[float, float, float] = (-0.3, -0.3, 0.05),
        hi: tuple[float, float, float] = (0.3, 0.3, 0.6),
        buffer: float = 0.0,
    ):
        self.lo = (float(lo[0]), float(lo[1]), float(lo[2]))
        self.hi = (float(hi[0]), float(hi[1]), float(hi[2]))
        self.buffer = float(buffer)
        super().__init__(robot)

    def h_1(self, z, **kwargs):
        q = z[: self.num_joints]
        # (num_spheres, 4): world-frame sphere centers + radii
        collision = self.robot.link_collision_data(q)
        centers = collision[:, :3]
        radii = collision[:, 3:4]  # keep (N,1) for broadcasting

        lo = jnp.asarray(self.lo) + self.buffer
        hi = jnp.asarray(self.hi) - self.buffer

        # clearance to each face; positive = inside with room, negative = outside.
        lower = centers - lo - radii  # (N, 3)
        upper = hi - centers - radii  # (N, 3)
        return jnp.concatenate([lower.ravel(), upper.ravel()])

    def alpha(self, h):
        return 5.0 * h

    def P(self, z, u_des, *args, **kwargs):
        # SO101 is 5-DOF, so OSCBF's default 6D task objective is singular here.
        return jnp.eye(self.num_joints)

    def q(self, z, u_des, *args, **kwargs):
        return -u_des

    def alpha_2(self, h_2):
        return 5.0 * h_2
