#!/usr/bin/python3
"""Record a jump-up / knock-over / jump-up demo from a running Cubli launch.

    ros2 launch cubli_control cubli_sim.launch.py          # in one terminal
    python3 scripts/record_demo.py media/demo_run.npz      # in another

Once connected it resets the cube onto a face and starts recording. After the cube
has balanced for `hold` seconds it is pushed over; recording stops `hold` seconds
after it is balancing again. Render with scripts/render_demo_gif.py.

Other uses:
    --truth             also record the simulator's true tilt (estimator vs truth, Phase 2)
    --fake-pid N        hardware path with the fake ESP32: no reset, push with SIGUSR1 to N
    --pushes K          knock it over K times      --duration S   record exactly S seconds
Times come from the message stamps, so a starved recorder doesn't distort them.
"""

import argparse
import json
import os
import signal

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, String
from std_srvs.srv import Trigger


class DemoRecorder(Node):

    def __init__(self, push: float, hold: float, max_time: float, truth=False, fake_pid=None,
                 pushes=1, duration=None):
        super().__init__('cubli_demo_recorder')
        self.push, self.hold, self.max_time = push, hold, max_time
        self.fake_pid, self.pushes, self.duration = fake_pid, pushes, duration
        self.rows = []      # t, tilt, body_rate, wheel_angle, wheel_speed, torque[, true tilt]
        self.truth = None if not truth else float('nan')
        self.modes = []     # controller mode at each row
        self.events = []    # (t, label) for pushes
        self.mode = None
        self.mode_changed = None
        self.torque = 0.0
        self.recording = False
        self.reset_pending = None
        self.t0 = None
        self.pushed = 0
        self.done = False

        self.push_pub = self.create_publisher(Float64, 'cubli/push', 10)
        self.reset_client = self.create_client(Trigger, 'cubli/reset')
        self.create_subscription(JointState, 'joint_states', self.on_joint_states, 50)
        self.create_subscription(Float64, 'cubli/wheel_torque', self.on_torque, 50)
        if truth:
            self.create_subscription(JointState, 'cubli/ground_truth/joint_states',
                                     self.on_truth, 50)
        self.create_subscription(
            String, 'cubli/mode', self.on_mode,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def on_torque(self, msg: Float64):
        self.torque = msg.data

    def on_truth(self, msg: JointState):
        self.truth = msg.position[msg.name.index('body_joint')]

    def on_mode(self, msg: String):
        self.mode = msg.data
        self.mode_changed = self.now()

    def on_joint_states(self, msg: JointState):
        if self.done:
            return
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if not self.recording and self.fake_pid is not None:
            if self.mode is not None:   # fake ESP32: it already starts lying on a face
                self.get_logger().info('Connected; recording')
                self.recording = True
                self.t0 = stamp
            return
        if not self.recording:
            # Wait for the controller to be live, then put the cube back on a face.
            if (self.reset_pending is None and self.mode is not None
                    and self.reset_client.service_is_ready()):
                self.get_logger().info('Connected; resetting cube and recording')
                self.reset_pending = self.reset_client.call_async(Trigger.Request())
            if self.reset_pending is not None and self.reset_pending.done():
                self.recording = True
                self.t0 = stamp
            return

        ib, iw = msg.name.index('body_joint'), msg.name.index('wheel_joint')
        t = stamp - self.t0
        row = (t, msg.position[ib], msg.velocity[ib], msg.position[iw], msg.velocity[iw],
               self.torque)
        self.rows.append(row if self.truth is None else row + (self.truth,))
        self.modes.append(self.mode)

        balanced_for = self.now() - self.mode_changed if self.mode == 'BALANCE' else 0.0
        recent_push = self.events and self.events[-1][0] > t - 1.0
        if self.pushed < self.pushes and balanced_for > self.hold and not recent_push:
            if self.fake_pid is not None:
                os.kill(self.fake_pid, signal.SIGUSR1)
            else:
                self.push_pub.publish(Float64(data=self.push))
            self.events.append((t, 'PUSH'))
            self.pushed += 1
        elif self.duration is not None:
            self.done = t > self.duration
        elif (self.pushed and not recent_push and balanced_for > self.hold) or t > self.max_time:
            self.done = True


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('output', help='output .npz file')
    parser.add_argument('--push', type=float, default=6.0, help='knock-over push [rad/s]')
    parser.add_argument('--hold', type=float, default=2.0, help='seconds balanced before push / stop')
    parser.add_argument('--max-time', type=float, default=30.0)
    parser.add_argument('--truth', action='store_true', help='record the sim ground truth too')
    parser.add_argument('--fake-pid', type=int, default=None,
                        help='fake ESP32 pid: push it with SIGUSR1 (+6 rad/s), no reset')
    parser.add_argument('--pushes', type=int, default=1)
    parser.add_argument('--duration', type=float, default=None,
                        help='record exactly this long [s]')
    args = parser.parse_args()

    rclpy.init()
    node = DemoRecorder(args.push, args.hold, args.max_time, args.truth, args.fake_pid,
                        args.pushes, args.duration)
    node.get_logger().info('Waiting for the Cubli launch ...')
    while rclpy.ok() and not node.done:
        rclpy.spin_once(node, timeout_sec=0.1)

    np.savez(args.output, data=np.array(node.rows), modes=np.array(node.modes),
             events=json.dumps(node.events))
    changes = [(round(r[0], 2), m) for k, (r, m) in enumerate(zip(node.rows, node.modes))
               if k == 0 or m != node.modes[k - 1]]
    print(f'Saved {len(node.rows)} samples; modes: {changes}; events: {node.events}')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
