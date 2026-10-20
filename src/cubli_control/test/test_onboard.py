"""On-board balance mode (Python twin of the firmware loop), closed loop with the sim."""

import math

import numpy as np
import pytest

from cubli_control.hardware import (
    MODE_BALANCE, MODE_CALIBRATING, MODE_IDLE, MODE_PASSTHROUGH, NidecMotor, encode,
    onboard_setup, parse)
from cubli_control.lqr import lqr_gain
from cubli_control.model import CubliParams, apply_ground_contact, dynamics, rk4_step
from cubli_control.onboard import OnboardBalancer
from cubli_control.sensors import SensorModel, SensorParams

DT = 1.0 / 200.0


def configured(max_torque=0.08):
    p = CubliParams(max_torque=max_torque)
    k = lqr_gain(p, DT, [100.0, 0.3, 1e-4], 100.0).ravel()
    ob = OnboardBalancer()
    params = {'engage': 0.12, 'disengage': 0.16, 'catch': 0.4, 'max_torque': max_torque}
    for line in onboard_setup(k, params, ['+x', '+y', '+z']):
        ob.handle(*parse(line), 0.0)
    return ob, p


def run(ob, p, x0, seconds, hold=1.2, heartbeat=True, seed=1, push_at=None):
    """Hold the cube still at x0 for `hold` s (calibration), then let go."""
    sensors = SensorModel(SensorParams(), seed=seed)
    motor = NidecMotor()
    x = np.array(x0, dtype=float)
    effort, log = 0.0, []
    for i in range(int(seconds / DT)):
        now = i * DT
        if heartbeat:
            ob.handle('T', ['0.000'], now)
        torque = motor.torque_for(effort, x[3])
        held = now < hold
        if held:
            x[3] += torque / p.theta_w * DT
            xdot = np.zeros(4)
        else:
            if push_at is not None and abs(now - push_at) < DT / 2:
                x[1] += 0.5
            for _ in range(5):
                x = apply_ground_contact(rk4_step(x, torque, DT / 5, p))
            xdot = dynamics(x, torque, p)
        gyro, accel = sensors.imu(x, xdot)
        out, est = ob.step(now, int(now * 1e6), tuple(accel), tuple(gyro),
                           sensors.encoder_counts(x[2]), True)
        effort = 0.0 if out is None else out
        log.append((now, x.copy(), ob.mode, est))
    return x, log


def test_passthrough_by_default():
    ob = OnboardBalancer()
    assert ob.step(0.0, 0, (0, 0, 9.81), (0, 0, 0), 0, True) == (None, None)
    assert ob.mode == MODE_PASSTHROUGH


def test_calibrates_then_balances_and_recovers_from_a_push():
    ob, p = configured()
    x, log = run(ob, p, [0.08, 0.0, 0.0, 0.0], 8.0, push_at=4.0)
    modes = [m for _, _, m, _ in log]
    assert modes[0] == MODE_CALIBRATING and MODE_BALANCE in modes
    assert ob.mode == MODE_BALANCE
    assert abs(x[0]) < math.radians(0.5)
    tilts = [abs(s[0]) for t, s, _, _ in log if t > 6.0]
    assert max(tilts) < math.radians(0.5)
    est = log[-1][3]
    assert est.angle == pytest.approx(x[0], abs=math.radians(0.5))   # estimate tracks truth


def test_does_not_engage_outside_engage_angle_and_lets_go_when_lost():
    ob, p = configured()
    _, log = run(ob, p, [0.3, 0.0, 0.0, 0.0], 3.0)
    assert MODE_BALANCE not in [m for _, _, m, _ in log]
    assert ob.mode == MODE_IDLE


def test_catch_window_after_brake():
    ob, _ = configured()
    ob.mode = MODE_IDLE
    ob.tilt.ready = True
    ob.tilt.angle = 0.3
    ob.handle('B', ['300'], 10.0)
    ob.step(10.1, 10_100_000, (0.0, 9.81 * math.sin(0.3), 9.81 * math.cos(0.3)),
            (0.0, 0.0, 0.0), 0, True)
    assert ob.mode == MODE_BALANCE          # 0.3 rad is inside catch (0.4) while armed


def test_link_loss_stops_balancing():
    ob, p = configured()
    run(ob, p, [0.05, 0.0, 0.0, 0.0], 2.0)
    assert ob.mode == MODE_BALANCE
    effort, est = ob.step(2.0 + 2.5, int(4.5e6), (0.0, 0.0, 9.81), (0.0, 0.0, 0.0), 0, True)
    assert ob.mode == MODE_IDLE and effort == 0.0 and est.torque == 0.0


def test_mode_off_returns_to_passthrough():
    ob, _ = configured()
    assert ob.mode == MODE_CALIBRATING
    ob.handle(*parse(encode('M', 0)), 0.0)
    assert ob.mode == MODE_PASSTHROUGH
