"""SafetyFilter unit tests: filter clamps wall-crossing targets, respects buffer."""
from __future__ import annotations

import sys
import os
import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))

# Default box that contains the SO101 home config (q=zeros).
# Sphere 4 reaches x=0.3506+0.0492=0.40 at q=zeros, so box_hi_x must be > 0.40.
# Sphere 3 reaches z_min=0.2235-0.0593=0.164, so box_lo_z must be < 0.164.
_SAFE_BOX_LO = (-0.45, -0.45, 0.10)
_SAFE_BOX_HI = (0.45, 0.45, 0.70)

_DT = 0.05   # control period — must match SafetyFilter default


def _make_filter(**kwargs):
    from so101_safety.filter import SafetyFilter
    from so101_safety.kinematics import NumpyKinematics

    box_lo = kwargs.pop("box_lo", _SAFE_BOX_LO)
    box_hi = kwargs.pop("box_hi", _SAFE_BOX_HI)
    return SafetyFilter(NumpyKinematics(), box_lo, box_hi, dt=_DT, **kwargs)


def test_home_config_is_inside_safe_box():
    """Sanity: all spheres at q=zeros must be inside the safe test box."""
    from so101_safety.kinematics import NumpyKinematics
    k = NumpyKinematics()
    q0 = np.zeros(5)
    sph = k.sphere_positions(q0)
    rad = k.sphere_radii()
    lo = np.array(_SAFE_BOX_LO)
    hi = np.array(_SAFE_BOX_HI)
    for i, (pos, r) in enumerate(zip(sph, rad)):
        for ax in range(3):
            assert pos[ax] - lo[ax] - r > 0, f"Sphere {i} axis {ax} below lower bound"
            assert hi[ax] - pos[ax] - r > 0, f"Sphere {i} axis {ax} above upper bound"


def test_filter_passes_safe_target():
    """A shoulder-pan-only command must pass through (nearly) unchanged.

    Shoulder-pan (joint 0) is a pure z-axis rotation — it does not change the
    height of any sphere, so no lower-z barrier is active and the filter should
    return the command essentially unchanged.
    """
    sf = _make_filter()
    q0 = np.zeros(5)
    q_des = q0.copy()
    q_des[0] = 0.10   # 0.1 rad pan only — confirmed constraint-free at q=zeros
    q_safe = sf.filter(q0, q_des)
    np.testing.assert_allclose(q_safe, q_des, atol=1e-6)


def test_filter_clamps_wall_crossing_target():
    """A target that drives the EE outside the box must be clamped."""
    # Tight floor at z=0.23 while spheres at q=zeros are near z=0.22.
    # A large downward command must be blocked.
    sf = _make_filter(box_lo=(-0.45, -0.45, 0.23), box_hi=(0.45, 0.45, 0.70),
                      buffer=0.01)
    q0 = np.zeros(5)
    q_des = np.array([0.0, 2.0, 2.0, 2.0, 0.0])
    q_safe = sf.filter(q0, q_des)
    assert not np.allclose(q_safe, q_des, atol=1e-2), (
        "Filter should have clamped the downward command"
    )


def test_filter_output_satisfies_cbf_constraints():
    """Starting inside the box, the filtered output must also stay inside.

    Uses a box where q=zeros is cleanly inside (box_lo_z=0.10, well below
    the lowest sphere at z=0.224-0.059=0.165).
    """
    from so101_safety.kinematics import NumpyKinematics

    box_lo = (-0.45, -0.45, 0.10)
    box_hi = (0.45, 0.45, 0.70)
    buffer = 0.01
    sf = _make_filter(box_lo=box_lo, box_hi=box_hi, buffer=buffer)
    k = NumpyKinematics()

    q0 = np.zeros(5)
    q_des = np.array([0.0, 2.0, 2.0, 2.0, 0.0])
    q_safe = sf.filter(q0, q_des)

    sph = k.sphere_positions(q_safe)
    lo = np.array(box_lo) + buffer
    hi = np.array(box_hi) - buffer
    radii = k.sphere_radii()

    for i, (pos, r) in enumerate(zip(sph, radii)):
        for axis in range(3):
            assert pos[axis] - lo[axis] - r >= -1e-3, (
                f"Sphere {i} violates lower bound on axis {axis}: "
                f"{pos[axis]:.4f} < {lo[axis]+r:.4f}"
            )
            assert hi[axis] - pos[axis] - r >= -1e-3, (
                f"Sphere {i} violates upper bound on axis {axis}: "
                f"{pos[axis]:.4f} > {hi[axis]-r:.4f}"
            )


def test_filter_agrees_with_jax_reference():
    """SafetyFilter output must be close to the JAX WorkspaceContainmentConfig reference.

    cbfpy safety_filter works in velocity units (rad/s). We convert:
      u_nom = δq / dt  →  cbfpy  →  u_safe  →  q_safe = q0 + u_safe * dt
    """
    import jax.numpy as jnp
    from cbfpy import CBF
    from so101_robot import load_so101_robot
    from so101_workspace import WorkspaceContainmentConfig

    box_lo = (-0.45, -0.45, 0.10)
    box_hi = (0.45, 0.45, 0.70)
    buffer = 0.01
    alpha = 5.0
    dt = _DT

    sf = _make_filter(box_lo=box_lo, box_hi=box_hi, buffer=buffer, alpha=alpha,
                      max_vel=10.0 * dt)   # match cbfpy's joint_max_velocities=10 rad/s
    robot = load_so101_robot()
    cfg = WorkspaceContainmentConfig(robot, lo=box_lo, hi=box_hi, buffer=buffer)
    cbf = CBF.from_config(cfg)

    rng = np.random.default_rng(99)
    for _ in range(15):
        q0 = rng.uniform(-0.5, 0.5, 5)
        q_des = rng.uniform(-0.5, 0.5, 5)

        q_safe_np = sf.filter(q0, q_des)

        # cbfpy expects velocity (rad/s); convert δq → velocity → position delta
        z = jnp.asarray(q0)
        u_nom_vel = jnp.asarray((q_des - q0) / dt)   # rad/s
        u_safe_vel = cbf.safety_filter(z, u_nom_vel)  # filtered velocity
        q_safe_jax = np.asarray(q0) + np.asarray(u_safe_vel) * dt  # back to position

        np.testing.assert_allclose(q_safe_np, q_safe_jax, atol=5e-3,
                                   err_msg=f"Numpy vs JAX CBF mismatch at q0={q0}")
