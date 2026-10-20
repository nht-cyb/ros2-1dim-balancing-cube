#!/usr/bin/python3
"""Offline controller tuning for the 1D Cubli balance loop.

Simulates the full Phase 2 loop (noisy IMU + encoder -> estimator -> LQR -> motor) with
control delay, timing jitter, torque limits, a random disturbance torque and a push, and
scores each tuning on:

  tilt_rms   RMS true tilt from the balance point while holding still [deg]
  du_rms     RMS change of torque between control steps [mN m]   (smoothness)
  peak       largest tilt after a push [deg]
  settle     time from the push until the tilt stays within 0.5 deg [s]
  max_delay  largest constant delay [steps of 5 ms] it still balances with
  fails      runs (of all seeds and delays) where the cube fell

    python3 scripts/tune_controller.py                 # baseline + sweep, simulator profile
    python3 scripts/tune_controller.py --profile hw    # Nidec torque limits
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
from cubli_control.model import CubliParams, dynamics, rk4_step  # noqa: E402
from cubli_control.sensors import SensorModel, SensorParams  # noqa: E402

DT = 1.0 / 200.0
PROFILES = {'sim': 0.25, 'hw': 0.08}   # torque limit [N m]


def simulate(tuning, seed, delay=1, jitter=2, max_torque=0.25, seconds=9.0,
             push_time=5.0, push=1.0, dist_std=0.003):
    """One balance run. Returns a dict of metrics (fell=True if it fell over)."""
    q, r, tau, design_delay = tuning
    p = CubliParams(max_torque=max_torque)
    k = lqr_gain(p, DT, q, r, delay_steps=design_delay).ravel()
    rng = np.random.default_rng(seed)
    sp = SensorParams()
    sensors = SensorModel(sp, seed=seed + 1000)
    est = TiltEstimator(time_constant=tau, imu_offset=sp.imu_offset)
    est.bias = sp.gyro_bias + rng.normal(0.0, sp.gyro_noise_std / math.sqrt(200))
    x = np.array([rng.uniform(-0.05, 0.05), 0.0, 0.0, 0.0])
    est.angle = x[0] + sp.accel_bias_y / sp.g
    est.ready = True
    wheel = WheelSpeedEstimator()

    n_steps = int(seconds / DT)
    history = []              # estimates available to the controller, newest last
    commands = [0.0] * max(design_delay, 1)
    applied = [0.0] * (delay + 1)
    last_used = -1
    u_prev = 0.0
    tilts, dus, us = [], [], []
    push_step = int(push_time / DT)
    peak, settle_step = 0.0, None
    u_now = 0.0

    for n in range(n_steps):
        xdot = dynamics(x, u_now, p)
        gyro, accel = sensors.imu(x, xdot)
        w = wheel.update(n * DT, sensors.encoder_angle(x[2]))
        angle, rate = est.update(gyro[0], accel[1], accel[2], DT)
        history.append((angle, rate, w))

        # Timing jitter: the controller sometimes sees an older estimate (never older
        # than the one it used last time, like a burst of late serial frames).
        lag = rng.integers(0, jitter + 1) if jitter else 0
        idx = max(last_used, len(history) - 1 - lag)
        last_used = idx
        z = np.array(history[idx])
        state = np.concatenate([z, commands[-design_delay:]]) if design_delay else z
        u = float(np.clip(-(k @ state), -max_torque, max_torque))
        commands.append(u)
        dus.append(u - u_prev)
        u_prev = u

        # Constant transport/actuation delay.
        applied.append(u)
        u_now = applied.pop(0)
        us.append(u_now)

        if n == push_step:
            x[1] += push
        disturbance = rng.normal(0.0, dist_std)
        for _ in range(2):
            x = rk4_step(x, u_now, DT / 2, p)
            x[1] += disturbance / p.inertia_body * DT / 2
        if abs(x[0]) > 0.35:
            return {'fell': True}
        tilts.append(x[0])
        if n >= push_step:
            peak = max(peak, abs(x[0]))
            if abs(x[0]) > math.radians(0.5):
                settle_step = n

    tilts = np.degrees(np.array(tilts))
    hold = slice(int(2.0 / DT), push_step)   # after the start-up transient
    if hold.start >= hold.stop:
        return {'fell': False}               # stability-only run (max_stable_delay)
    return {
        'fell': False,
        'tilt_rms': float(np.sqrt(np.mean(tilts[hold] ** 2))),
        'du_rms': 1000.0 * float(np.sqrt(np.mean(np.array(dus[hold]) ** 2))),
        'u_rms': 1000.0 * float(np.sqrt(np.mean(np.array(us[hold]) ** 2))),
        'peak': math.degrees(peak),
        'settle': ((settle_step or push_step) - push_step) * DT,
    }


def max_stable_delay(tuning, max_torque, limit=16):
    for d in range(limit + 1):
        if simulate(tuning, seed=7, delay=d, jitter=0, max_torque=max_torque,
                    push=0.5, push_time=2.0, seconds=5.0)['fell']:
            return d - 1
    return limit


def evaluate(args):
    tuning, max_torque, seeds = args
    runs = [simulate(tuning, s, delay=d, max_torque=max_torque)
            for s in range(seeds) for d in (1, 2)]
    ok = [r for r in runs if not r['fell']]
    result = {'tuning': tuning, 'fails': len(runs) - len(ok), 'runs': len(runs)}
    if ok:
        for key in ('tilt_rms', 'du_rms', 'u_rms', 'peak', 'settle'):
            result[key] = float(np.mean([r[key] for r in ok]))
    result['max_delay'] = max_stable_delay(tuning, max_torque)
    return result


def score(res):
    """Lower is better: tilt error first, then smoothness and recovery."""
    if res['fails'] or 'tilt_rms' not in res:
        return math.inf
    return (res['tilt_rms'] / 0.1 + res['du_rms'] / 1.0 + res['peak'] / 5.0
            + res['settle'] / 1.0 + max(0, 6 - res['max_delay']) * 2.0)


def fmt(res):
    q, r, tau, dd = res['tuning']
    head = f"Q=({q[0]:g},{q[1]:g},{q[2]:g}) R={r:g} tau={tau:g} delay-aware={dd}"
    if 'tilt_rms' not in res:
        return f"{head:52s} FELL in all runs"
    return (f"{head:52s} tilt_rms={res['tilt_rms']:.3f}° du_rms={res['du_rms']:.2f}mNm "
            f"peak={res['peak']:.2f}° settle={res['settle']:.2f}s "
            f"max_delay={res['max_delay']} fails={res['fails']}/{res['runs']}")


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--profile', choices=PROFILES, default='sim')
    parser.add_argument('--seeds', type=int, default=3)
    parser.add_argument('--top', type=int, default=8)
    args = parser.parse_args()
    max_torque = PROFILES[args.profile]

    baseline = ((100.0, 1.0, 1e-4), 100.0, 0.3, 0)
    # Only Q/R matters for LQR, so R is fixed and Q spans a wide range.
    grid = list(itertools.product(
        [(qa, qr, qw) for qa in (10.0, 30.0, 100.0, 300.0, 1000.0, 3000.0)
         for qr in (0.3, 1.0, 3.0) for qw in (1e-5, 1e-4, 1e-3)],
        (100.0,),
        (0.15, 0.3, 0.5),
        (0, 1)))
    jobs = [(t, max_torque, args.seeds) for t in [baseline] + grid]
    print(f'Profile {args.profile} (max torque {max_torque} N m): evaluating {len(jobs)} tunings '
          f'x {2 * args.seeds} runs ...', flush=True)
    with Pool() as pool:
        results = pool.map(evaluate, jobs, chunksize=4)

    print('\nBaseline (current config):\n  ' + fmt(results[0]))
    ranked = sorted(results[1:], key=score)
    print(f'\nBest {args.top} by score (tilt error, smoothness, push recovery, delay margin):')
    for res in ranked[:args.top]:
        print(f'  score={score(res):6.2f}  ' + fmt(res))
    print(f"\nBaseline score: {score(results[0]):.2f}")


if __name__ == '__main__':
    main()
