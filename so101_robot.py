"""Local SO101 robot builder.

`oscbf` is a read-only dependency, so all SO101-specific assembly lives here.
This wraps oscbf's ``Manipulator.from_urdf`` with the derived gripper collision
spheres (see ``so101_collision_model.py``) so the CBF can constrain the whole
gripper rather than a single end-effector point.

The collision model has 6 links (5 arm joints + 1 gripper joint). Since oscbf's
Manipulator only handles the 5-joint arm chain, we pass links 0-4 to it. Link 5
(the moving jaw) is handled by the viewer/filter using the jaw pivot transform.
"""

from __future__ import annotations

from oscbf.core.manipulator import Manipulator
from test_so101_real import SO101_URDF

from so101_collision_model import so101_collision_data


def _arm_collision_data() -> dict:
    """Extract links 0-4 collision data for the 5-joint oscbf Manipulator."""
    return {
        "positions": so101_collision_data["positions"][:5],
        "radii": so101_collision_data["radii"][:5],
    }


def load_so101_robot(with_collision: bool = True) -> Manipulator:
    collision_data = _arm_collision_data() if with_collision else None
    return Manipulator.from_urdf(SO101_URDF, collision_data=collision_data)
