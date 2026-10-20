"""Part 2: sensor models and state estimator (no ROS needed)."""

import math

import numpy as np
import pytest

from cubli_control.estimator import TiltEstimator, WheelSpeedEstimator
from cubli_control.lqr import lqr_gain
from cubli_control.model import (
    FALL_ANGLE, CubliParams, apply_ground_contact, brake, dynamics, jump_wheel_speed, rk4_step)
from cubli_control.sensors import SensorModel, SensorParams, ideal_imu

DT = 1.0 / 200.0


@pytest.mark.parametrize('angle', [-FALL_ANGLE, -0.3, 0.0, 0.2, FALL_ANGLE])
def test_accelerometer_at_rest_gives_tilt(angle):
    _, f_y, f_z = ideal_imu([angle, 0.0, 0.0, 0.0], np.zeros(4), imu_offset=0.106)
    assert math.atan2(f_y, f_z) == pytest.approx(angle)


def test_calibration_finds_gyro_bias_and_start_angle():
    sensors = SensorModel(SensorParams(gyro_bias=0.05, accel_bias_y=0.0, accel_bias_z=0.0), seed=1)
    est = TiltEstimator()
    x = np.array([-FALL_ANGLE, 0.0, 0.0, 0.0])
    done = False
    while not done:
        gyro, accel = sensors.imu(x, np.zeros(4))
        done = est.calibrate(gyro[0], accel[1], accel[2])
    assert est.bias == pytest.approx(0.05, abs=1e-3)
    assert est.angle == pytest.approx(-FALL_ANGLE, abs=0.01)


def test_calibration_restarts_if_the_cube_moves():
    est = TiltEstimator(calib_samples=50)
    for k in range(50):
        assert not est.calibrate(0.5 * math.sin(k), 0.0, 9.81)   # shaking
    for _ in range(50):
        done = est.calibrate(0.01, 0.0, 9.81)
    assert done and est.bias == pytest.approx(0.01)


def test_wheel_speed_from_quantised_encoder():
    sensors = SensorModel(SensorParams(encoder_cpr=400))
    wheel = WheelSpeedEstimator(window=8)
    for k in range(100):
        t = k * DT
        speed = wheel.update(t, sensors.encoder_angle(50.0 * t))
    assert speed == pytest.approx(50.0, abs=0.5)


def run_with_sensors(seed, sensor_params=SensorParams(), seconds=8.0):
    """Jump up from a face and balance using only the noisy sensors + estimator."""
    p = CubliParams()
    k = lqr_gain(p, DT, [100.0, 1.0, 1e-4], 100.0)
    sensors = SensorModel(sensor_params, seed=seed)
    tilt = TiltEstimator(time_constant=0.3, imu_offset=sensor_params.imu_offset)
    wheel = WheelSpeedEstimator()
    x = np.array([-FALL_ANGLE, 0.0, 0.0, 0.0])
    mode, u, target = 'calibrate', 0.0, 0.0
    for n in range(int(seconds / DT)):
        t = n * DT
        gyro, accel = sensors.imu(x, dynamics(x, u, p))
        w = wheel.update(t, sensors.encoder_angle(x[2]))
        if mode == 'calibrate':
            if tilt.calibrate(gyro[0], accel[1], accel[2]):
                mode = 'spin'
                target = -math.copysign(1.0, tilt.angle) * 1.1 * jump_wheel_speed(p, abs(tilt.angle))
        else:
            angle, rate = tilt.update(gyro[0], accel[1], accel[2], DT)
            if mode == 'spin':
                if abs(target - w) < 1.0:
                    x, u, mode = brake(x, p), 0.0, 'jump'
                else:
                    u = float(np.clip(0.01 * (target - w), -0.03, 0.03))
            elif mode == 'jump' and abs(angle) < 0.2:
                mode = 'balance'
            if mode == 'balance':
                assert abs(angle) < 0.2, 'fell over'
                u = float(np.clip(-(k @ np.array([angle, rate, w])).item(),
                                  -p.max_torque, p.max_torque))
        for _ in range(5):
            x = apply_ground_contact(rk4_step(x, u, DT / 5, p))
    return mode, x


@pytest.mark.parametrize('seed', [0, 1, 2])
def test_jump_up_and_balance_with_noisy_sensors(seed):
    mode, x = run_with_sensors(seed)
    assert mode == 'balance'
    assert abs(x[0]) < math.radians(1.0)


def test_balance_with_poor_sensors():
    poor = SensorParams(gyro_noise_std=0.01, accel_noise_std=0.12, gyro_bias=0.15,
                        accel_bias_y=0.3, encoder_cpr=100)
    mode, x = run_with_sensors(3, poor)
    assert mode == 'balance'
    assert abs(x[0]) < math.radians(1.0)
