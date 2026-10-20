"""Sensor models for the simulated Cubli: an MPU6050-style IMU and a quadrature wheel encoder.

IMU frame = body frame: x along the balancing edge, z from the edge through the cube
centre, y completing the right-handed frame. The IMU sits `imu_offset` metres from the
edge along body z, so its accelerometer also feels the cube's own rotation.
"""

from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class SensorParams:
    imu_offset: float = 0.106        # edge -> IMU along body z [m] (cube centre)
    gyro_noise_std: float = 0.003    # [rad/s] per sample
    gyro_bias: float = 0.02          # [rad/s] constant bias on the edge axis (~1.1 deg/s)
    accel_noise_std: float = 0.04    # [m/s^2] per sample
    accel_bias_y: float = 0.03       # [m/s^2]
    accel_bias_z: float = -0.05      # [m/s^2]
    encoder_cpr: int = 400           # counts per wheel revolution (quadrature)
    g: float = 9.81


def ideal_imu(x, xdot, imu_offset: float, g: float = 9.81):
    """Noise-free gyro (edge axis) and accelerometer (body y, z) readings.

    Specific force at the IMU, expressed in the body frame:
      f_y = g sin(theta) - d * theta_ddot      (gravity + tangential acceleration)
      f_z = g cos(theta) - d * theta_dot^2     (gravity + centripetal acceleration)
    """
    theta, rate = x[0], x[1]
    accel = xdot[1]
    d = imu_offset
    return rate, g * math.sin(theta) - d * accel, g * math.cos(theta) - d * rate ** 2


class SensorModel:
    """Turns the true simulator state into noisy raw sensor readings."""

    def __init__(self, params: SensorParams, seed=None):
        self.p = params
        self.rng = np.random.default_rng(seed)

    def imu(self, x, xdot):
        """Return (gyro[3] rad/s, accel[3] m/s^2) as an MPU6050 would report them."""
        p = self.p
        rate, f_y, f_z = ideal_imu(x, xdot, p.imu_offset, p.g)
        noise_g = self.rng.normal(0.0, p.gyro_noise_std, 3)
        noise_a = self.rng.normal(0.0, p.accel_noise_std, 3)
        gyro = np.array([rate + p.gyro_bias, 0.0, 0.0]) + noise_g
        accel = np.array([0.0, f_y + p.accel_bias_y, f_z + p.accel_bias_z]) + noise_a
        return gyro, accel

    def encoder_counts(self, wheel_angle: float) -> int:
        return int(math.floor(wheel_angle * self.p.encoder_cpr / (2.0 * math.pi)))

    def encoder_angle(self, wheel_angle: float) -> float:
        """Wheel angle as the quantised encoder reports it [rad]."""
        return self.encoder_counts(wheel_angle) * 2.0 * math.pi / self.p.encoder_cpr
