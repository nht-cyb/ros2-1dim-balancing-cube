#!/usr/bin/python3
"""Render a recorded Cubli demo (from record_demo.py) as an animated GIF.

    python3 scripts/render_demo_gif.py media/demo_run.npz media/cubli_demo.gif
"""

import argparse
import json
import math

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.animation import FuncAnimation, PillowWriter  # noqa: E402
from matplotlib.patches import Circle, Polygon  # noqa: E402

SIDE = 0.15          # cube side [m]
WHEEL_R = 0.06
MODE_STYLE = {
    'FALLEN': ('Lying on a face', '#7f7f7f'),
    'SPIN_UP': ('Spinning up the wheel', '#e8a33d'),
    'JUMPING': ('Brake! Jumping up', '#d64545'),
    'BALANCE': ('Balancing on the edge (LQR)', '#2e9e5b'),
    'PUSH': ('Pushed over!', '#8e44ad'),
}


def rotate(points, angle):
    """Rotate (y, z) points about the edge, matching body_joint in the URDF (+angle tilts toward -y)."""
    c, s = math.cos(angle), math.sin(angle)
    y, z = points[:, 0], points[:, 1]
    return np.column_stack([y * c - z * s, y * s + z * c])


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('recording')
    parser.add_argument('output')
    parser.add_argument('--fps', type=int, default=15)
    parser.add_argument('--speed', type=float, default=1.0, help='playback speed factor')
    args = parser.parse_args()

    rec = np.load(args.recording)
    data, modes, events = rec['data'], rec['modes'], json.loads(str(rec['events']))
    push_times = [et for et, name in events if name == 'PUSH']
    t, tilt, _, wheel_angle, wheel_speed, _ = data.T
    tilt_deg = np.degrees(tilt)

    def mode_at(i):
        if any(0.0 <= t[i] - pt < 0.6 for pt in push_times):
            return 'PUSH'
        return str(modes[i])

    fig = plt.figure(figsize=(8.0, 4.0), dpi=90)
    grid = fig.add_gridspec(2, 2, width_ratios=[1.0, 1.25], hspace=0.35, wspace=0.25,
                            left=0.03, right=0.97, top=0.88, bottom=0.12)
    ax_cube = fig.add_subplot(grid[:, 0])
    ax_tilt = fig.add_subplot(grid[0, 1])
    ax_wheel = fig.add_subplot(grid[1, 1], sharex=ax_tilt)

    ax_cube.set_xlim(-0.24, 0.24)
    ax_cube.set_ylim(-0.03, 0.26)
    ax_cube.set_aspect('equal')
    ax_cube.axis('off')
    ax_cube.plot([-0.24, 0.24], [0, 0], color='#444444', lw=3)
    ax_cube.fill_between([-0.24, 0.24], [-0.03, -0.03], [0, 0], color='#dddddd')

    h = SIDE / math.sqrt(2)
    cube_shape = np.array([[0, 0], [h, h], [0, 2 * h], [-h, h]])
    centre = np.array([[0.0, h]])
    cube = Polygon(cube_shape, closed=True, fc='#c9d3e0', ec='#34495e', lw=2)
    wheel = Circle((0, h), WHEEL_R, fc='#3b6fd6', ec='#1f3f80', lw=1.5)
    ax_cube.add_patch(cube)
    ax_cube.add_patch(wheel)
    spoke, = ax_cube.plot([], [], color='#ff8c1a', lw=3, solid_capstyle='round')
    pivot, = ax_cube.plot([0], [0], 'o', color='#d64545', ms=6)
    mode_text = fig.text(0.25, 0.92, '', ha='center', fontsize=12, fontweight='bold')
    time_text = ax_cube.text(0.0, -0.022, '', ha='center', fontsize=9, color='#555555')
    fig.text(0.62, 0.95, 'Cubli 1D: jump-up and balance (ROS 2 Jazzy)', ha='center',
             fontsize=11, fontweight='bold')

    for ax, y, label, colour in ((ax_tilt, tilt_deg, 'tilt [deg]', '#34495e'),
                                 (ax_wheel, wheel_speed, 'wheel speed [rad/s]', '#3b6fd6')):
        ax.plot(t, y, color=colour, alpha=0.15, lw=1)
        ax.set_ylabel(label, fontsize=9)
        ax.tick_params(labelsize=8)
        ax.grid(alpha=0.3)
        pad = 0.08 * (y.max() - y.min() + 1e-6)
        ax.set_ylim(y.min() - pad, y.max() + pad)
    ax_tilt.axhline(0, color='#2e9e5b', lw=0.8, ls='--')
    ax_tilt.set_xlim(t[0], t[-1])
    ax_wheel.set_xlabel('time [s]', fontsize=9)
    marks = push_times + [t[k] for k in range(1, len(t))
                          if modes[k] == 'JUMPING' and modes[k - 1] != 'JUMPING']
    for mark in marks:
        colour = MODE_STYLE['PUSH' if mark in push_times else 'JUMPING'][1]
        for ax in (ax_tilt, ax_wheel):
            ax.axvline(mark, color=colour, lw=0.8, alpha=0.6)
    tilt_line, = ax_tilt.plot([], [], color='#34495e', lw=1.8)
    wheel_line, = ax_wheel.plot([], [], color='#3b6fd6', lw=1.8)
    tilt_dot, = ax_tilt.plot([], [], 'o', color='#d64545', ms=4)
    wheel_dot, = ax_wheel.plot([], [], 'o', color='#d64545', ms=4)

    frame_times = np.arange(t[0], t[-1], args.speed / args.fps)

    def draw(frame_t):
        i = min(np.searchsorted(t, frame_t), len(t) - 1)
        angle = tilt[i]
        cube.set_xy(rotate(cube_shape, angle))
        c = rotate(centre, angle)[0]
        wheel.center = tuple(c)
        spoke_angle = angle + wheel_angle[i]
        tip = c + WHEEL_R * 0.9 * np.array([-math.sin(spoke_angle), math.cos(spoke_angle)])
        spoke.set_data([c[0], tip[0]], [c[1], tip[1]])

        text, colour = MODE_STYLE[mode_at(i)]
        mode_text.set_text(text)
        mode_text.set_color(colour)
        time_text.set_text(f't = {t[i]:.1f} s   tilt = {tilt_deg[i]:+.1f}°')

        tilt_line.set_data(t[:i + 1], tilt_deg[:i + 1])
        wheel_line.set_data(t[:i + 1], wheel_speed[:i + 1])
        tilt_dot.set_data([t[i]], [tilt_deg[i]])
        wheel_dot.set_data([t[i]], [wheel_speed[i]])
        return cube, wheel, spoke, pivot, tilt_line, wheel_line, tilt_dot, wheel_dot

    anim = FuncAnimation(fig, draw, frames=frame_times, blit=False)
    anim.save(args.output, writer=PillowWriter(fps=args.fps))
    print(f'Wrote {args.output} ({len(frame_times)} frames)')


if __name__ == '__main__':
    main()
