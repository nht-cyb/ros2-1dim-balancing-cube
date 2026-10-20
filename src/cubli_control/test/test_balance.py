"""Offline checks of the model and LQR design (no ROS needed)."""

import math

import numpy as np
import pytest

from cubli_control.lqr import discretize, lqr_gain
from cubli_control.model import (
    FALL_ANGLE, CubliParams, apply_ground_contact, brake, jump_wheel_speed, linearized, rk4_step)

DT = 1.0 / 200.0
Q = [100.0, 1.0, 1e-4]
R = 100.0


def simulate(initial_angle, seconds=10.0, delay_steps=1):
    p = CubliParams()
    k = lqr_gain(p, DT, Q, R)
    x = np.array([initial_angle, 0.0, 0.0, 0.0])
    pending = [0.0] * delay_steps
    for _ in range(int(seconds / DT)):
        u = float(np.clip(-(k @ x[[0, 1, 3]]).item(), -p.max_torque, p.max_torque))
        pending.append(u)
        applied = pending.pop(0)
        for _ in range(5):
            x = rk4_step(x, applied, DT / 5, p)
        if abs(x[0]) > math.pi / 4:
            break
    return x


def test_upright_is_unstable_open_loop():
    a, _ = linearized(CubliParams())
    assert max(np.linalg.eigvals(a).real) > 0


def test_closed_loop_is_stable():
    p = CubliParams()
    a, b = linearized(p)
    ad, bd = discretize(a, b, DT)
    k = lqr_gain(p, DT, Q, R)
    assert max(abs(np.linalg.eigvals(ad - bd @ k))) < 1.0


@pytest.mark.parametrize('angle', [0.02, -0.05, 0.1, -0.15])
def test_recovers_from_tilt(angle):
    x = simulate(angle)
    assert abs(x[0]) < 1e-3      # back upright
    assert abs(x[3]) < 5.0       # wheel spun back down


def test_brake_conserves_momentum():
    p = CubliParams()
    x = brake([0.0, 0.3, 0.0, 50.0], p)
    before = p.inertia_body * 0.3 + p.theta_w * (0.3 + 50.0)
    assert x[3] == 0.0
    assert x[1] * (p.inertia_body + p.theta_w) == pytest.approx(before)


@pytest.mark.parametrize('side', [-1.0, 1.0])
@pytest.mark.parametrize('margin', [1.1, 1.2])
def test_jump_up_from_face_then_balance(side, margin):
    p = CubliParams()
    k = lqr_gain(p, DT, Q, R)
    x = np.array([side * FALL_ANGLE, 0.0, 0.0, -side * margin * jump_wheel_speed(p)])
    x = brake(x, p)
    for _ in range(int(5.0 / DT)):
        u = 0.0
        if abs(x[0]) < 0.2:
            u = float(np.clip(-(k @ x[[0, 1, 3]]).item(), -p.max_torque, p.max_torque))
        for _ in range(5):
            x = apply_ground_contact(rk4_step(x, u, DT / 5, p))
    assert abs(x[0]) < 0.01      # balanced on the edge
    assert abs(x[1]) < 0.01


def test_wheel_stays_still_while_lying_on_floor():
    p = CubliParams()
    x = np.array([-FALL_ANGLE, 0.0, 0.0, 0.0])
    for _ in range(200):
        x = apply_ground_contact(rk4_step(x, 0.0, DT, p))
    assert x[0] == pytest.approx(-FALL_ANGLE)
    assert abs(x[3]) < 1e-9


def test_spin_up_on_floor_only_moves_wheel():
    p = CubliParams()
    x = np.array([-FALL_ANGLE, 0.0, 0.0, 0.0])
    for _ in range(200):     # 1 s at 0.03 N m, reaction presses the cube into the floor
        x = apply_ground_contact(rk4_step(x, 0.03, DT, p))
    assert x[0] == pytest.approx(-FALL_ANGLE)
    # first-order spin-up against bearing friction: w = T/c_w (1 - exp(-c_w t / theta_w))
    expected = 0.03 / p.c_w * (1 - math.exp(-p.c_w * 1.0 / p.theta_w))
    assert x[3] == pytest.approx(expected, rel=1e-3)


def test_delay_aware_gain_shape_and_stability():
    p = CubliParams()
    k = lqr_gain(p, DT, Q, R, delay_steps=2)
    assert k.shape == (1, 5)
    a, b = linearized(p)
    ad, bd = discretize(a, b, DT)
    # Closed loop of the augmented system [z, u_{k-2}, u_{k-1}] must be stable.
    big = np.zeros((5, 5))
    big[:3, :3] = ad
    big[:3, 3:4] = bd
    big[3, 4] = 1.0
    b_aug = np.zeros((5, 1))
    b_aug[4, 0] = 1.0
    assert max(abs(np.linalg.eigvals(big - b_aug @ k))) < 1.0


def test_plain_gain_unchanged_by_zero_delay():
    p = CubliParams()
    assert np.allclose(lqr_gain(p, DT, Q, R), lqr_gain(p, DT, Q, R, delay_steps=0))
