import jax.numpy as jnp

from so101_robot import load_so101_robot
from so101_workspace import WorkspaceContainmentConfig


def _robot():
    return load_so101_robot()


def test_box_margin_shape_and_finite():
    robot = _robot()
    cfg = WorkspaceContainmentConfig(robot, lo=(-0.3, -0.3, 0.05), hi=(0.3, 0.3, 0.6))
    h = cfg.h_1(jnp.zeros(robot.num_joints))
    n_spheres = robot.link_collision_data(jnp.zeros(robot.num_joints)).shape[0]
    assert h.shape == (6 * n_spheres,)  # lower+upper face per sphere per axis
    assert bool(jnp.isfinite(h).all())


def test_large_box_all_inside():
    robot = _robot()
    cfg = WorkspaceContainmentConfig(robot, lo=(-1.0, -1.0, -1.0), hi=(1.0, 1.0, 1.0))
    h = cfg.h_1(jnp.zeros(robot.num_joints))
    assert bool((h > 0).all())  # comfortably contained -> all margins positive


def test_tiny_box_violated():
    robot = _robot()
    cfg = WorkspaceContainmentConfig(robot, lo=(-0.01, -0.01, -0.01), hi=(0.01, 0.01, 0.01))
    h = cfg.h_1(jnp.zeros(robot.num_joints))
    assert bool((h < 0).any())  # gripper pokes outside a tiny box


def test_buffer_shrinks_margin():
    robot = _robot()
    lo, hi = (-0.3, -0.3, 0.05), (0.3, 0.3, 0.6)
    h0 = WorkspaceContainmentConfig(robot, lo=lo, hi=hi, buffer=0.0).h_1(
        jnp.zeros(robot.num_joints)
    )
    hb = WorkspaceContainmentConfig(robot, lo=lo, hi=hi, buffer=0.05).h_1(
        jnp.zeros(robot.num_joints)
    )
    assert bool((hb <= h0 + 1e-6).all())  # buffer never increases any margin
