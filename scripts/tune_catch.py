#!/usr/bin/python3
"""Offline study of the post-jump catch: jump from a face (brake), then catch and hold.

Varies the catch angle (when the LQR takes over during a jump), the "give up only when
moving away" rule, and the jump margin, under control delay + jitter and a torque limit,
with the Phase 2 noisy sensors + estimator. Reports the share of jumps that end balanced.

    python3 scripts/tune_catch.py --profile hw
"""

import argparse
import itertools
import math
import os
import sys
from multiprocessing import Pool

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src', 'cubli_control'))
from cubli_control.estimator import TiltEstimator, WheelSpeedEstimator  # noqa: E402
from cubli_control.lqr import lqr_gain  # noqa: E402
from cubli_control.model import (  # noqa: E402
    FALL_ANGLE, CubliParams, apply_ground_contact, brake, dynamics, jump_wheel_speed, rk4_step)
from cubli_control.sensors import SensorModel, SensorParams  # noqa: E402

DT = 1.0 / 200.0
PROFILES = {'sim': 0.25, 'hw': 0.08}
Q, R = (100.0, 0.3, 1e-4), 100.0


def run(args):
    catch_angle, hold_angle, away_rule, margin, max_torque, delay, jitter, seed = args
    p = CubliParams(max_torque=max_torque)
    k = lqr_gain(p, DT, Q, R).ravel()
    rng = np.random.default_rng(seed)
    sp = SensorParams()
    sensors = SensorModel(sp, seed=seed + 500)
    est = TiltEstimator(time_constant=0.5, imu_offset=sp.imu_offset)
    est.bias = sp.gyro_bias
    est.angle = -FALL_ANGLE + sp.accel_bias_y / sp.g
    est.ready = True
    wheel = WheelSpeedEstimator()
    side = 1.0 if seed % 2 else -1.0      # jump from both faces
    x = np.array([side * FALL_ANGLE, 0.0, 0.0, -side * margin * jump_wheel_speed(p)])
    est.angle = x[0] - side * 0.0
    x = brake(x, p)
    history, last_used = [], -1
    applied = [0.0] * (delay + 1)
    mode, u_now = 'JUMPING', 0.0
    for n in range(int(4.0 / DT)):
        gyro, accel = sensors.imu(x, dynamics(x, u_now, p))
        w = wheel.update(n * DT, sensors.encoder_angle(x[2]))
        history.append(est.update(gyro[0], accel[1], accel[2], DT) + (w,))
        lag = rng.integers(0, jitter + 1) if jitter else 0
        idx = max(last_used, len(history) - 1 - lag)
        last_used = idx
        angle, rate, w_est = history[idx]
        u = 0.0
        if mode == 'JUMPING' and abs(angle) < catch_angle:
            mode = 'BALANCE'
        if mode == 'BALANCE':
            moving_away = angle * rate > 0
            if abs(angle) > hold_angle and (moving_away or not away_rule):
                mode = 'FALLEN'
            else:
                u = float(np.clip(-(k @ np.array([angle, rate, w_est])), -max_torque, max_torque))
        applied.append(u)
        u_now = applied.pop(0)
        for _ in range(2):
            x = apply_ground_contact(rk4_step(x, u_now, DT / 2, p))
        if mode == 'FALLEN':
            break
    return mode == 'BALANCE' and abs(x[0]) < 0.05


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--profile', choices=PROFILES, default='hw')
    parser.add_argument('--seeds', type=int, default=20)
    args = parser.parse_args()
    mt = PROFILES[args.profile]
    hold = 0.16 if args.profile == 'hw' else 0.3
    print(f'Profile {args.profile}: max torque {mt} N m, give-up angle {hold} rad, '
          f'{args.seeds} jumps per cell; cells show % of jumps that end balanced')
    for delay, jitter in ((1, 0), (2, 2), (4, 4)):
        print(f'\n  delay {delay} steps + jitter 0..{jitter} steps')
        print('  catch_angle  away_rule | margin 1.03  1.05  1.10  1.15')
        for catch, away in itertools.product((0.12, 0.2, 0.3, 0.4), (False, True)):
            cells = []
            for margin in (1.03, 1.05, 1.10, 1.15):
                jobs = [(catch, hold, away, margin, mt, delay, jitter, s) for s in range(args.seeds)]
                with Pool() as pool:
                    ok = pool.map(run, jobs)
                cells.append(f'{100 * sum(ok) / len(ok):5.0f}')
            print(f'  {catch:11.2f}  {str(away):9s} |        ' + ' '.join(cells))


if __name__ == '__main__':
    main()
