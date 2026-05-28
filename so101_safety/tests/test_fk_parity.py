"""FK parity tests: NumpyKinematics must agree with JAX oscbf and MuJoCo."""
from __future__ import annotations

import sys
import os

import numpy as np
import pytest

# Allow importing so101_robot from the parent repo root
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))


def test_numpy_fk_matches_oscbf():
    """NumpyKinematics.ee_position must match oscbf JAX reference to 1e-4 m."""
    import jax.numpy as jnp
    from so101_safety.kinematics import NumpyKinematics
    from so101_robot import load_so101_robot

    robot = load_so101_robot()
    k = NumpyKinematics()
    rng = np.random.default_rng(42)
    for _ in range(20):
        q = rng.uniform(-1.0, 1.0, 5)
        ref = np.asarray(robot.ee_position(jnp.asarray(q)))
        np.testing.assert_allclose(k.ee_position(q), ref, atol=1e-4,
                                   err_msg=f"FK mismatch at q={q}")


def test_numpy_sphere_positions_match_oscbf():
    """NumpyKinematics.sphere_positions must match oscbf to 1e-4 m."""
    import jax.numpy as jnp
    from so101_safety.kinematics import NumpyKinematics
    from so101_robot import load_so101_robot

    robot = load_so101_robot()
    k = NumpyKinematics()
    rng = np.random.default_rng(7)
    for _ in range(20):
        q = rng.uniform(-1.0, 1.0, 5)
        ref = np.asarray(robot.link_collision_data(jnp.asarray(q)))[:, :3]  # (n,3)
        np.testing.assert_allclose(k.sphere_positions(q), ref, atol=1e-4,
                                   err_msg=f"Sphere pos mismatch at q={q}")


def test_numpy_fk_matches_mujoco():
    """NumpyKinematics.sphere_positions must match MujocoKinematics to 1e-4 m."""
    from so101_safety.kinematics import NumpyKinematics
    from so101_safety.kinematics_mujoco import MujocoKinematics

    xml_path = os.path.abspath(os.path.join(
        os.path.dirname(__file__), "..", "..", "robot_models", "so101", "scene.xml"
    ))
    n = NumpyKinematics()
    m = MujocoKinematics(xml_path)
    rng = np.random.default_rng(13)
    for _ in range(20):
        q = rng.uniform(-1.0, 1.0, 5)
        np.testing.assert_allclose(
            n.sphere_positions(q), m.sphere_positions(q), atol=1e-4,
            err_msg=f"MuJoCo vs numpy sphere pos mismatch at q={q}"
        )
        np.testing.assert_allclose(
            n.ee_position(q), m.ee_position(q), atol=1e-4,
            err_msg=f"MuJoCo vs numpy EE pos mismatch at q={q}"
        )
