"""Local SO101 robot builder.

`oscbf` is a read-only dependency, so all SO101-specific assembly lives here.
This wraps oscbf's ``Manipulator.from_urdf`` with the derived gripper collision
spheres (see ``so101_collision_model.py``) so the CBF can constrain the whole
gripper rather than a single end-effector point.
"""

from __future__ import annotations

from oscbf.core.manipulator import Manipulator
from test_so101_real import SO101_URDF

from so101_collision_model import so101_collision_data


def load_so101_robot(with_collision: bool = True) -> Manipulator:
    collision_data = so101_collision_data if with_collision else None
    return Manipulator.from_urdf(SO101_URDF, collision_data=collision_data)
