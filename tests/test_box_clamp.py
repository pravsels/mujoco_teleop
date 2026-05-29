"""Headless regression: SafetyFilter clamps slider targets at box boundaries."""
from __future__ import annotations

import numpy as np
import pytest


_BOX_LO = (-0.45, -0.45, 0.10)
_BOX_HI = (0.45, 0.45, 0.70)
_BUFFER = 0.01


def _make_filter(**kwargs):
    from so101_safety.filter import SafetyFilter
    from so101_safety.kinematics import NumpyKinematics

    return SafetyFilter(
        NumpyKinematics(),
        kwargs.pop("box_lo", _BOX_LO),
        kwargs.pop("box_hi", _BOX_HI),
        buffer=kwargs.pop("buffer", _BUFFER),
        **kwargs,
    )


def test_aggressive_target_stays_in_box_5joint():
    """An aggressive 5-joint target is clamped so all spheres remain inside."""
    from so101_safety.kinematics import NumpyKinematics

    sf = _make_filter()
    k = NumpyKinematics()
    lo = np.array(_BOX_LO) + _BUFFER
    hi = np.array(_BOX_HI) - _BUFFER

    q0 = np.zeros(5)
    q_des = np.array([0.0, 2.0, 2.0, 2.0, 0.0])
    q_safe = sf.filter(q0, q_des)

    sph = k.sphere_positions(q_safe)
    radii = k.sphere_radii(q_safe)
    for i, (pos, r) in enumerate(zip(sph, radii)):
        for ax in range(3):
            assert pos[ax] - lo[ax] - r >= -1e-3, (
                f"Sphere {i} violates lower bound axis {ax}"
            )
            assert hi[ax] - pos[ax] - r >= -1e-3, (
                f"Sphere {i} violates upper bound axis {ax}"
            )


def test_aggressive_target_stays_in_box_6joint():
    """An aggressive 6-joint target is clamped; gripper passes through."""
    from so101_safety.kinematics import NumpyKinematics

    sf = _make_filter()
    k = NumpyKinematics()
    lo = np.array(_BOX_LO) + _BUFFER
    hi = np.array(_BOX_HI) - _BUFFER

    q0 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.3])
    q_des = np.array([0.0, 2.0, 2.0, 2.0, 0.0, 0.7])
    q_safe = sf.filter(q0, q_des)

    assert q_safe[5] == 0.7, "Gripper should pass through"

    sph = k.sphere_positions(q_safe)
    radii = k.sphere_radii(q_safe)
    for i, (pos, r) in enumerate(zip(sph, radii)):
        for ax in range(3):
            assert pos[ax] - lo[ax] - r >= -1e-3, (
                f"Sphere {i} violates lower bound axis {ax}"
            )
            assert hi[ax] - pos[ax] - r >= -1e-3, (
                f"Sphere {i} violates upper bound axis {ax}"
            )


def test_safe_target_passes_unchanged():
    """A small safe command passes through without modification."""
    sf = _make_filter()
    q0 = np.zeros(5)
    q_des = q0.copy()
    q_des[0] = 0.1  # small pan only
    q_safe = sf.filter(q0, q_des)
    np.testing.assert_allclose(q_safe, q_des, atol=1e-6)
