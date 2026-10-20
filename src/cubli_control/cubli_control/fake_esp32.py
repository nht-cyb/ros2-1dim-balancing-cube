"""Fake ESP32: the Cubli physics behind a virtual serial port, speaking the real protocol.

Lets you test the hardware path (driver -> estimator -> controller) with no board:

    ros2 run cubli_control fake_esp32                    # creates /tmp/cubli_fake_esp32
    ros2 launch cubli_control cubli_hw.launch.py port:=/tmp/cubli_fake_esp32

It also implements the on-board balance mode ($M/$K/$P/$A, $X frames) with the same
estimator and LQR code as the firmware (cubli_control/onboard.py). The physics and the
on-board loop step together, so PC load slows it down but never delays the loop itself.

Push the cube:  kill -USR1 <pid>  (+6 rad/s)   kill -USR2 <pid>  (-6 rad/s)
"""

import argparse
import math
import os
import signal
import time
import tty

import numpy as np

from cubli_control.hardware import MODE_NAMES, NidecMotor, SensorFrame, encode, parse
from cubli_control.model import FALL_ANGLE, CubliParams, apply_ground_contact, brake, dynamics, rk4_step
from cubli_control.onboard import OnboardBalancer
from cubli_control.sensors import SensorModel, SensorParams


def main():
    parser = argparse.ArgumentParser(description='Fake ESP32 Cubli on a virtual serial port')
    parser.add_argument('--link', default='/tmp/cubli_fake_esp32', help='symlink to the pty')
    parser.add_argument('--rate', type=float, default=200.0)
    parser.add_argument('--brake-efficiency', type=float, default=1.0)
    parser.add_argument('--initial-angle', type=float, default=-FALL_ANGLE)
    parser.add_argument('--stall-torque', type=float, default=NidecMotor.stall_torque)
    parser.add_argument('--no-load-speed', type=float, default=NidecMotor.no_load_speed)
    parser.add_argument('--seed', type=int, default=None)
    args, _ = parser.parse_known_args()   # ignore --ros-args when started by ros2 launch

    master, slave = os.openpty()
    tty.setraw(slave)
    os.set_blocking(master, False)
    if os.path.islink(args.link):
        os.unlink(args.link)
    os.symlink(os.ttyname(slave), args.link)
    print(f'Fake ESP32 on {args.link} -> {os.ttyname(slave)} (pid {os.getpid()})', flush=True)

    p = CubliParams()
    sensors = SensorModel(SensorParams(), seed=args.seed)
    motor = NidecMotor(args.stall_torque, args.no_load_speed)
    x = np.array([args.initial_angle, 0.0, 0.0, 0.0])
    onboard = OnboardBalancer()
    state = {'effort': 0.0, 'enabled': False, 'last_cmd': 0.0, 'push': 0.0,
             'onboard_effort': None}

    def on_push(signum, _frame):
        state['push'] += 6.0 if signum == signal.SIGUSR1 else -6.0
    signal.signal(signal.SIGUSR1, on_push)
    signal.signal(signal.SIGUSR2, on_push)

    def on_term(signum, _frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, on_term)
    signal.signal(signal.SIGINT, on_term)   # also when started with SIGINT ignored

    def handle(line, x):
        msg = parse(line)
        if msg is None:
            return x
        kind, fields = msg
        info = onboard.handle(kind, fields, time.monotonic())
        if info:
            print(f'Fake ESP32: {info}', flush=True)
            os.write(master, encode('I', info))
        if kind == 'T' and fields:
            state['effort'] = max(-1.0, min(1.0, float(fields[0])))
            state['last_cmd'] = time.monotonic()
        elif kind == 'E' and fields:
            state['enabled'] = fields[0] == '1'
            text = f'motor {"enabled" if state["enabled"] else "disabled"}'
            print(f'Fake ESP32: {text}', flush=True)
            os.write(master, encode('I', text))
        elif kind == 'B':
            before = x[3]
            x = brake(x, p, args.brake_efficiency)
            state['effort'] = 0.0
            os.write(master, encode('I', f'brake wheel {before:.1f} rad/s'))
        return x

    dt = 1.0 / args.rate
    start = time.monotonic()
    next_t = start
    step = 0   # simulated time = step * dt, like the ESP32's own timer
    buffer = b''
    os.write(master, encode('I', 'fake ESP32 ready'))
    try:
        while True:
            try:
                buffer += os.read(master, 4096)
            except BlockingIOError:
                pass
            except OSError:
                buffer = b''   # no reader attached yet
            *lines, buffer = buffer.split(b'\n')
            for line in lines:
                x = handle(line, x)

            if state['push']:
                x[1] += state['push']
                state['push'] = 0.0

            effort = state['effort']
            if time.monotonic() - state['last_cmd'] > 0.1:
                effort = 0.0   # firmware watchdog
            if state['onboard_effort'] is not None:
                effort = state['onboard_effort']   # on-board LQR is balancing
            if not state['enabled']:
                effort = 0.0
            torque = motor.torque_for(effort, x[3]) if effort != 0.0 else 0.0
            for _ in range(5):
                x = apply_ground_contact(rk4_step(x, torque, dt / 5, p))

            gyro, accel = sensors.imu(x, dynamics(x, torque, p))
            step += 1
            t_us = int(step * dt * 1e6) & 0xFFFFFFFF   # wraps like micros()
            frame = SensorFrame(t_us, tuple(accel), tuple(gyro), sensors.encoder_counts(x[2]))
            mode_before = onboard.mode
            state['onboard_effort'], estimate = onboard.step(
                time.monotonic(), t_us, frame.accel, frame.gyro, frame.encoder, state['enabled'])
            out = frame.encode() + (estimate.encode() if estimate else b'')
            if onboard.mode != mode_before:
                print(f'Fake ESP32: on-board mode {MODE_NAMES[onboard.mode]} '
                      f'(true tilt {math.degrees(x[0]):+.1f} deg)', flush=True)
            try:
                os.write(master, out)
            except (BlockingIOError, OSError):
                pass   # nobody reading: drop the frame like a real UART would

            next_t += dt
            time.sleep(max(0.0, next_t - time.monotonic()))
            if time.monotonic() - next_t > 0.5:
                next_t = time.monotonic()   # fell far behind; don't try to catch up
    except KeyboardInterrupt:
        pass
    finally:
        if os.path.islink(args.link):
            os.unlink(args.link)
        print(f'Fake ESP32 stopped; final tilt {math.degrees(x[0]):.2f} deg', flush=True)


if __name__ == '__main__':
    main()
