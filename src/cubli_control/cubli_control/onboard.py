"""On-board balance mode: the estimator + LQR loop the ESP32 runs itself (no ROS imports).

A line-by-line Python twin of the on-board part of firmware/cubli_esp32/cubli_esp32.ino,
used by the fake ESP32 and the tests. Keep the two in step when changing either.
"""

import math

from cubli_control.estimator import TiltEstimator, WheelSpeedEstimator
from cubli_control.hardware import (
    MODE_BALANCE, MODE_CALIBRATING, MODE_IDLE, MODE_PASSTHROUGH, ONBOARD_PARAMS,
    EstimateFrame, NidecMotor, counts_to_rad, remap_axes)

LINK_TIMEOUT = 2.0   # s without any valid line from the PC -> stop balancing


class OnboardBalancer:

    def __init__(self):
        self.params = dict(ONBOARD_PARAMS)
        self.k = (0.0, 0.0, 0.0)
        self.axes = ['+x', '+y', '+z']
        self.mode = MODE_PASSTHROUGH
        self.catch_until = -math.inf
        self.last_rx = 0.0
        self.torque = 0.0
        self.reset()

    def reset(self):
        """Restart calibration (on $M,1 and $A)."""
        p = self.params
        self.tilt = TiltEstimator(time_constant=p['tau'], imu_offset=p['imu_offset'])
        self.wheel = WheelSpeedEstimator(window=8)
        self.last_t = None
        if self.mode != MODE_PASSTHROUGH:
            self.mode = MODE_CALIBRATING

    def handle(self, kind, fields, now):
        """Apply one PC command. Returns an info text for the PC, or None."""
        self.last_rx = now
        if kind == 'M' and fields:
            if fields[0] == '1':
                self.mode = MODE_CALIBRATING
                self.reset()
                return 'onboard balance on, calibrating'
            self.mode = MODE_PASSTHROUGH
            return 'onboard balance off'
        if kind == 'K' and len(fields) == 3:
            self.k = tuple(float(f) for f in fields)
            return 'K ' + ','.join(fields)
        if kind == 'P' and len(fields) == 2 and fields[0] in self.params:
            self.params[fields[0]] = float(fields[1])
            return None
        if kind == 'A' and len(fields) == 3:
            self.axes = list(fields)
            self.reset()
            return None
        if kind == 'B':
            ms = int(fields[0]) if fields else 0
            self.catch_until = now + ms * 1e-3 + self.params['jump_timeout']
        return None

    def step(self, now, t_us, accel, gyro, enc_counts, enabled):
        """One 200 Hz sample. Returns (effort or None to use the PC's $T, EstimateFrame or None)."""
        if self.mode == MODE_PASSTHROUGH:
            return None, None
        p = self.params
        t = t_us * 1e-6
        dt = 0.0 if self.last_t is None else min(max(t - self.last_t, 0.0), 0.05)
        self.last_t = t
        g = remap_axes(gyro, self.axes)
        a = remap_axes(accel, self.axes)
        wheel_angle = counts_to_rad(int(p['enc_sign']) * enc_counts, int(p['cpr']))
        speed = self.wheel.update(t, wheel_angle)

        if self.mode == MODE_CALIBRATING:
            if self.tilt.calibrate(g[0], a[1], a[2]):
                self.mode = MODE_IDLE
            return None, EstimateFrame(t_us, 0.0, 0.0, speed, 0.0, self.mode)
        raw_angle, rate = self.tilt.update(g[0], a[1], a[2], dt)

        angle = raw_angle - p['angle_offset']
        link_ok = now - self.last_rx < LINK_TIMEOUT
        if self.mode == MODE_BALANCE:
            moving_away = angle * rate > 0.0
            lost = (abs(angle) > p['catch']
                    or (abs(angle) > max(p['disengage'], p['engage']) and moving_away))
            if lost or not enabled or not link_ok:
                self.mode = MODE_IDLE
        elif enabled and link_ok:
            catching = now < self.catch_until and abs(angle) < max(p['catch'], p['engage'])
            if abs(angle) < p['engage'] or catching:
                self.mode = MODE_BALANCE
                self.catch_until = -math.inf

        effort = None
        self.torque = 0.0
        if self.mode == MODE_BALANCE:
            u = -(self.k[0] * angle + self.k[1] * rate + self.k[2] * speed)
            self.torque = max(-p['max_torque'], min(p['max_torque'], u))
            motor = NidecMotor(p['stall_torque'], p['no_load_speed'])
            effort = motor.effort_for(self.torque, speed, p['max_effort'])
        elif not link_ok:
            effort = 0.0
        return effort, EstimateFrame(t_us, raw_angle, rate, speed, self.torque, self.mode)
