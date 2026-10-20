"""State estimation from raw sensors: tilt from IMU, wheel speed from encoder.

Shared by the simulator path and the hardware path (no ROS imports here).
"""

from collections import deque
import math


class TiltEstimator:
    """Gyro-bias calibration + complementary filter for the tilt about the edge.

    - Calibration: while the cube lies still, average the gyro to get its bias and
      the accelerometer to get the starting angle.
    - Prediction: integrate the bias-corrected gyro.
    - Correction: blend in the accelerometer tilt with time constant `time_constant`,
      after removing the IMU's own centripetal/tangential acceleration (lever-arm
      compensation). The correction is skipped while the accelerometer is clearly
      not just measuring gravity (jumps, impacts).
    """

    def __init__(self, time_constant=0.5, imu_offset=0.106, calib_samples=200,
                 calib_max_rate_std=0.05, gate_accel=1.0, gate_rate=3.0, g=9.81):
        self.tau = time_constant
        self.d = imu_offset
        self.calib_samples = calib_samples
        self.calib_max_rate_std = calib_max_rate_std
        self.gate_accel = gate_accel
        self.gate_rate = gate_rate
        self.g = g
        self.bias = 0.0
        self.angle = 0.0
        self.rate = 0.0
        self.accel_f = 0.0   # filtered angular acceleration for lever-arm compensation
        self.ready = False
        self._calib = []

    def calibrate(self, gyro_x, acc_y, acc_z) -> bool:
        """Feed one still sample. Returns True once calibration has finished."""
        self._calib.append((gyro_x, acc_y, acc_z))
        if len(self._calib) < self.calib_samples:
            return False
        gyro = [s[0] for s in self._calib]
        mean = sum(gyro) / len(gyro)
        std = math.sqrt(sum((v - mean) ** 2 for v in gyro) / len(gyro))
        if std > self.calib_max_rate_std:
            self._calib.clear()  # it moved; start again
            return False
        self.bias = mean
        self.angle = math.atan2(sum(s[1] for s in self._calib), sum(s[2] for s in self._calib))
        self.ready = True
        return True

    def update(self, gyro_x, acc_y, acc_z, dt):
        """Return (angle, rate) after one IMU sample."""
        rate = gyro_x - self.bias
        if dt > 0.0:
            raw_accel = (rate - self.rate) / dt
            k = dt / (0.01 + dt)            # ~16 Hz low-pass on the differentiated gyro
            self.accel_f += k * (raw_accel - self.accel_f)
        self.rate = rate
        self.angle += rate * dt

        f_y = acc_y + self.d * self.accel_f
        f_z = acc_z + self.d * rate ** 2
        if abs(math.hypot(f_y, f_z) - self.g) < self.gate_accel and abs(rate) < self.gate_rate:
            alpha = self.tau / (self.tau + dt)
            self.angle = alpha * self.angle + (1.0 - alpha) * math.atan2(f_y, f_z)
        return self.angle, self.rate


class WheelSpeedEstimator:
    """Wheel speed from a quantised encoder angle by differencing over a short window."""

    def __init__(self, window=8):
        self.samples = deque(maxlen=window + 1)
        self.speed = 0.0

    def update(self, t, angle):
        self.samples.append((t, angle))
        t0, a0 = self.samples[0]
        if t > t0:
            self.speed = (angle - a0) / (t - t0)
        return self.speed
