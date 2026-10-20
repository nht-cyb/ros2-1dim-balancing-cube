#!/usr/bin/python3
"""Render the Phase 2 and Phase 3 result GIFs from record_demo.py recordings.

    # Phase 2: estimated vs true tilt (recorded from cubli_sim.launch.py with --truth)
    python3 scripts/render_results_gif.py estimator media/phase2_run.npz media/phase2_estimator.gif

    # Phase 3: PC loop vs on-board loop, side by side (fake ESP32, same CPU load)
    python3 scripts/render_results_gif.py compare media/phase3_passthrough.npz \\
        media/phase3_onboard.npz media/phase3_onboard_vs_pc.gif
"""

import argparse
import json
import math
import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.animation import FuncAnimation, PillowWriter  # noqa: E402
from matplotlib.patches import Circle, Polygon  # noqa: E402

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from render_demo_gif import MODE_STYLE, SIDE, WHEEL_R, rotate  # noqa: E402

HELD_AFTER = 2.0   # s balanced before a catch counts as "held"


def load(path):
    rec = np.load(path)
    data, modes = rec['data'], np.array([str(m) for m in rec['modes']])
    pushes = [t for t, name in json.loads(str(rec['events'])) if name == 'PUSH']
    return data, modes, pushes


class CubeView:
    """The cube on its edge, with the wheel and a spoke so the wheel's spin is visible."""

    def __init__(self, ax):
        ax.set_xlim(-0.2, 0.2)
        ax.set_ylim(-0.035, 0.235)
        ax.set_aspect('equal')
        ax.axis('off')
        ax.plot([-0.2, 0.2], [0, 0], color='#444444', lw=3)
        ax.fill_between([-0.2, 0.2], [-0.035, -0.035], [0, 0], color='#dddddd')
        h = SIDE / math.sqrt(2)
        self.shape = np.array([[0, 0], [h, h], [0, 2 * h], [-h, h]])
        self.centre = np.array([[0.0, h]])
        self.cube = Polygon(self.shape, closed=True, fc='#c9d3e0', ec='#34495e', lw=2)
        self.wheel = Circle((0, h), WHEEL_R, fc='#3b6fd6', ec='#1f3f80', lw=1.5)
        ax.add_patch(self.cube)
        ax.add_patch(self.wheel)
        self.spoke, = ax.plot([], [], color='#ff8c1a', lw=3, solid_capstyle='round')
        ax.plot([0], [0], 'o', color='#d64545', ms=5)

    def draw(self, tilt, wheel_angle):
        self.cube.set_xy(rotate(self.shape, tilt))
        c = rotate(self.centre, tilt)[0]
        self.wheel.center = tuple(c)
        a = tilt + wheel_angle
        tip = c + WHEEL_R * 0.9 * np.array([-math.sin(a), math.cos(a)])
        self.spoke.set_data([c[0], tip[0]], [c[1], tip[1]])


def mode_at(t, modes, pushes, i):
    if any(0.0 <= t[i] - pt < 0.6 for pt in pushes):
        return 'PUSH'
    return modes[i] if modes[i] in MODE_STYLE else 'FALLEN'


def mark_events(axes, t, modes, pushes):
    jumps = [t[k] for k in range(1, len(t)) if modes[k] == 'JUMPING' and modes[k - 1] != 'JUMPING']
    for ax in axes:
        for mark in jumps:
            ax.axvline(mark, color=MODE_STYLE['JUMPING'][1], lw=0.8, alpha=0.5)
        for mark in pushes:
            ax.axvline(mark, color=MODE_STYLE['PUSH'][1], lw=0.8, alpha=0.7)


def catch_stats(t, modes, pushes, upto):
    """Jumps, catches held >= HELD_AFTER s (or until a push / the end) and pushes, up to `upto`."""
    jumps = held = 0
    start = None
    for k in range(len(t)):
        if t[k] > upto:
            break
        if modes[k] == 'JUMPING' and (k == 0 or modes[k - 1] != 'JUMPING'):
            jumps += 1
        if modes[k] == 'BALANCE' and (k == 0 or modes[k - 1] != 'BALANCE'):
            start, counted = t[k], False
        if modes[k] == 'BALANCE' and start is not None and not counted and t[k] - start >= HELD_AFTER:
            held += 1
            counted = True
    return jumps, held, sum(1 for p in pushes if p <= upto)


def save(fig, draw, frame_times, fps, output):
    FuncAnimation(fig, draw, frames=frame_times, blit=False).save(
        output, writer=PillowWriter(fps=fps))
    print(f'Wrote {output} ({len(frame_times)} frames)')


def render_estimator(args):
    data, modes, pushes = load(args.recording)
    t, est, _, wheel_angle, wheel_speed, _, truth = data.T[:7]
    ok = ~np.isnan(truth)
    t, est, wheel_angle, truth, modes = t[ok], est[ok], wheel_angle[ok], truth[ok], modes[ok]
    est_d, truth_d = np.degrees(est), np.degrees(truth)
    err = est_d - truth_d
    steady = np.zeros(len(t), bool)    # balanced for > 1 s and not just pushed
    since = None
    for k in range(len(t)):
        since = (since if since is not None else t[k]) if modes[k] == 'BALANCE' else None
        steady[k] = since is not None and t[k] - since > 1.0
    rms = math.sqrt(np.mean(err[steady] ** 2)) if steady.any() else float('nan')

    fig = plt.figure(figsize=(8.4, 4.2), dpi=90)
    grid = fig.add_gridspec(2, 2, width_ratios=[1.0, 1.35], height_ratios=[1.3, 1.0],
                            hspace=0.4, wspace=0.22, left=0.03, right=0.97, top=0.86, bottom=0.12)
    cube = CubeView(fig.add_subplot(grid[:, 0]))
    ax_tilt = fig.add_subplot(grid[0, 1])
    ax_err = fig.add_subplot(grid[1, 1], sharex=ax_tilt)
    fig.text(0.62, 0.94, 'Phase 2: tilt estimated from a noisy IMU + encoder',
             ha='center', fontsize=11, fontweight='bold')
    mode_text = fig.text(0.23, 0.9, '', ha='center', fontsize=12, fontweight='bold')
    info = fig.text(0.23, 0.04, '', ha='center', fontsize=9, color='#444444')

    ax_tilt.plot(t, truth_d, color='#999999', alpha=0.25, lw=3)
    ax_tilt.set_ylabel('tilt [deg]', fontsize=9)
    ax_err.plot(t, err, color='#d64545', alpha=0.15, lw=1)
    ax_err.set_ylabel('estimate − truth [deg]', fontsize=9)
    ax_err.set_xlabel('time [s]', fontsize=9)
    lim = max(1.0, float(np.percentile(np.abs(err), 99.5)) * 1.15)
    ax_err.set_ylim(-lim, lim)
    ax_err.axhline(0, color='#2e9e5b', lw=0.8, ls='--')
    ax_tilt.set_xlim(t[0], t[-1])
    for ax in (ax_tilt, ax_err):
        ax.grid(alpha=0.3)
        ax.tick_params(labelsize=8)
    mark_events((ax_tilt, ax_err), t, modes, pushes)
    truth_line, = ax_tilt.plot([], [], color='#777777', lw=3.5, alpha=0.6, label='true tilt (sim)')
    est_line, = ax_tilt.plot([], [], color='#3b6fd6', lw=1.4, label='estimate')
    ax_tilt.legend(loc='lower right', fontsize=8, framealpha=0.9)
    err_line, = ax_err.plot([], [], color='#d64545', lw=1.2)
    ax_err.text(0.01, 0.9, f'RMS while balanced: {rms:.3f}°', transform=ax_err.transAxes,
                fontsize=8, va='top', color='#d64545',
                bbox=dict(fc='white', ec='none', alpha=0.8))

    def draw(ft):
        i = min(np.searchsorted(t, ft), len(t) - 1)
        cube.draw(truth[i], wheel_angle[i])
        text, colour = MODE_STYLE[mode_at(t, modes, pushes, i)]
        mode_text.set_text(text)
        mode_text.set_color(colour)
        info.set_text(f't = {t[i]:.1f} s   true {truth_d[i]:+.1f}°   estimate {est_d[i]:+.1f}°')
        truth_line.set_data(t[:i + 1], truth_d[:i + 1])
        est_line.set_data(t[:i + 1], est_d[:i + 1])
        err_line.set_data(t[:i + 1], err[:i + 1])

    save(fig, draw, np.arange(t[0], t[-1], args.speed / args.fps), args.fps, args.output)


def render_compare(args):
    runs = [load(path) for path in (args.left, args.right)]
    labels = [args.left_label, args.right_label]
    end = min(r[0][-1, 0] for r in runs)
    fig = plt.figure(figsize=(8.4, 4.8), dpi=90)
    grid = fig.add_gridspec(2, 2, height_ratios=[1.25, 1.0], hspace=0.28, wspace=0.18,
                            left=0.08, right=0.98, top=0.84, bottom=0.1)
    fig.text(0.53, 0.95, args.title, ha='center', fontsize=11, fontweight='bold')
    time_text = fig.text(0.53, 0.89, '', ha='center', fontsize=9, color='#555555')
    views = []
    for col, ((data, modes, pushes), label) in enumerate(zip(runs, labels)):
        t, tilt, _, wheel_angle = data.T[:4]
        ax_cube = fig.add_subplot(grid[0, col])
        ax_tilt = fig.add_subplot(grid[1, col])
        ax_cube.set_title(label, fontsize=10, fontweight='bold', pad=14)
        tilt_d = np.degrees(tilt)
        ax_tilt.plot(t, tilt_d, color='#34495e', alpha=0.12, lw=1)
        ax_tilt.set_xlim(0, end)
        ax_tilt.set_ylim(-50, 50)
        ax_tilt.set_yticks([-45, 0, 45])
        ax_tilt.axhline(0, color='#2e9e5b', lw=0.8, ls='--')
        ax_tilt.grid(alpha=0.3)
        ax_tilt.tick_params(labelsize=8)
        ax_tilt.set_xlabel('time [s]', fontsize=9)
        if col == 0:
            ax_tilt.set_ylabel('tilt [deg]', fontsize=9)
        mark_events((ax_tilt,), t, modes, pushes)
        line, = ax_tilt.plot([], [], color='#34495e', lw=1.5)
        mode_text = ax_cube.text(0.5, 1.0, '', transform=ax_cube.transAxes, ha='center',
                                 fontsize=10, fontweight='bold')
        stats = ax_cube.text(0.5, -0.04, '', transform=ax_cube.transAxes, ha='center',
                             fontsize=9, color='#444444')
        views.append((CubeView(ax_cube), t, tilt, tilt_d, wheel_angle, modes, pushes,
                      line, mode_text, stats))

    def draw(ft):
        for cube, t, tilt, tilt_d, wheel_angle, modes, pushes, line, mode_text, stats in views:
            i = min(np.searchsorted(t, ft), len(t) - 1)
            cube.draw(tilt[i], wheel_angle[i])
            text, colour = MODE_STYLE[mode_at(t, modes, pushes, i)]
            mode_text.set_text(text.replace(' (LQR)', ''))
            mode_text.set_color(colour)
            jumps, held, pushed = catch_stats(t, modes, pushes, t[i])
            stats.set_text(f'jumps {jumps}   held > {HELD_AFTER:.0f} s: {held}   knocked over: {pushed}')
            line.set_data(t[:i + 1], tilt_d[:i + 1])
        time_text.set_text(f't = {ft:.1f} s')

    save(fig, draw, np.arange(0.0, end, args.speed / args.fps), args.fps, args.output)


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = parser.add_subparsers(dest='kind', required=True)
    p = sub.add_parser('estimator', help='Phase 2: estimate vs ground truth')
    p.add_argument('recording')
    p.add_argument('output')
    p = sub.add_parser('compare', help='Phase 3: two runs side by side')
    p.add_argument('left')
    p.add_argument('right')
    p.add_argument('output')
    p.add_argument('--left-label', default='PC loop (passthrough)')
    p.add_argument('--right-label', default='On-board loop (ESP32)')
    p.add_argument('--title', default='Phase 3: balance loop on the PC vs on the ESP32, '
                                      'all 8 CPU cores at 100%')
    for p in sub.choices.values():
        p.add_argument('--fps', type=int, default=12)
        p.add_argument('--speed', type=float, default=1.0, help='playback speed factor')
    args = parser.parse_args()
    render_estimator(args) if args.kind == 'estimator' else render_compare(args)


if __name__ == '__main__':
    main()
