"""Jump-up and LQR balancing controller for the 1D Cubli.

Subscribes:  /joint_states       (sensor_msgs/JointState)
Publishes:   /cubli/wheel_torque (std_msgs/Float64)
             /cubli/brake        (std_msgs/Empty)
             /cubli/mode         (std_msgs/String, latched)  current mode, see below

Modes:
  FALLEN   cube lying on a face; after rest_time it starts a jump (if auto_jump_up)
  SPIN_UP  spin the wheel up to the jump speed (reaction torque presses the cube to the floor);
           brake only once the speed has held steady for spin_settle_time, so late or
           bursty sensor data cannot fire the brake at the wrong speed
  JUMPING  brake fired, cube is flipping up; the LQR takes over at catch_angle, early
           enough to slow the cube down before it overshoots
  BALANCE  LQR keeps the cube on its edge. It only gives up beyond disengage_angle while
           still moving away from upright (or beyond catch_angle), so it neither flickers
           near the limit nor abandons a cube that is already coming back

If a jump falls short, the wheel speed margin grows by jump_margin_step and it retries.

With onboard_balance: true the ESP32 balances by itself (see hw_driver_node.py): this node
still runs the jump-up, but BALANCE starts and ends when /cubli/onboard_mode says so, and
it sends zero torque while balancing (the ESP32 ignores it then anyway).
Talks to the simulator today; a hardware driver publishing the same topics can replace it.
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty, Float64, String

from cubli_control.lqr import lqr_gain
from cubli_control.model import FALL_ANGLE, CubliParams, jump_wheel_speed
from cubli_control.ros_utils import spin_until_shutdown

FALLEN, SPIN_UP, JUMPING, BALANCE = 'FALLEN', 'SPIN_UP', 'JUMPING', 'BALANCE'


class BalanceController(Node):

    def __init__(self):
        super().__init__('cubli_controller')
        self.declare_parameter('control_rate_hz', 200.0)
        self.declare_parameter('max_torque', CubliParams.max_torque)
        self.declare_parameter('engage_angle', 0.2)
        self.declare_parameter('disengage_angle', 0.3)
        self.declare_parameter('catch_angle', 0.4)
        self.declare_parameter('lqr_delay_steps', 0)
        self.declare_parameter('q_angle', 100.0)
        self.declare_parameter('q_rate', 0.3)
        self.declare_parameter('q_wheel_speed', 1e-4)
        self.declare_parameter('r_torque', 100.0)
        self.declare_parameter('angle_offset', 0.0)
        self.declare_parameter('auto_jump_up', True)
        self.declare_parameter('rest_time', 1.0)
        self.declare_parameter('spin_up_torque', 0.03)
        self.declare_parameter('jump_margin', 1.1)
        self.declare_parameter('jump_margin_step', 0.05)
        self.declare_parameter('jump_margin_max', 1.4)
        self.declare_parameter('spin_settle_time', 0.15)
        self.declare_parameter('spin_tolerance', 2.0)
        self.declare_parameter('spin_timeout', 4.0)
        self.declare_parameter('jump_timeout', 1.5)
        self.declare_parameter('onboard_balance', False)

        gp = lambda name: self.get_parameter(name).value  # noqa: E731
        self.params = CubliParams(max_torque=gp('max_torque'))
        self.engage_angle = gp('engage_angle')
        self.disengage_angle = max(gp('disengage_angle'), self.engage_angle)
        self.catch_angle = max(gp('catch_angle'), self.engage_angle)
        self.delay_steps = int(gp('lqr_delay_steps'))
        self.angle_offset = gp('angle_offset')
        self.auto_jump_up = gp('auto_jump_up')
        self.rest_time = gp('rest_time')
        self.spin_up_torque = min(gp('spin_up_torque'), self.params.max_torque)
        self.jump_margin = gp('jump_margin')
        self.jump_margin_step = gp('jump_margin_step')
        self.jump_margin_max = gp('jump_margin_max')
        self.spin_settle_time = gp('spin_settle_time')
        self.spin_tolerance = gp('spin_tolerance')
        self.spin_timeout = gp('spin_timeout')
        self.jump_timeout = gp('jump_timeout')
        self.onboard = bool(gp('onboard_balance'))
        self.onboard_balancing = False
        self.k = lqr_gain(
            self.params, 1.0 / gp('control_rate_hz'),
            [gp('q_angle'), gp('q_rate'), gp('q_wheel_speed')], gp('r_torque'),
            delay_steps=self.delay_steps).ravel()
        self.get_logger().info(
            f'LQR gain K = {np.array2string(self.k, precision=4)}'
            + (f' (delay-aware, {self.delay_steps} step)' if self.delay_steps else ''))
        self.recent_torques = [0.0] * self.delay_steps   # commands still "in flight"

        self.torque_pub = self.create_publisher(Float64, 'cubli/wheel_torque', 10)
        self.brake_pub = self.create_publisher(Empty, 'cubli/brake', 10)
        self.mode_pub = self.create_publisher(
            String, 'cubli/mode', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

        self.mode = None
        self.mode_since = self.now()
        self.resting_since = None
        self.settled_since = None
        self.jump_target = 0.0
        self.set_mode(FALLEN)
        self.create_subscription(JointState, 'joint_states', self.on_joint_states, 10)
        if self.onboard:
            self.get_logger().info('On-board balance: the ESP32 runs the LQR')
            self.create_subscription(
                String, 'cubli/onboard_mode', self.on_onboard_mode,
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def on_onboard_mode(self, msg: String):
        self.onboard_balancing = msg.data == BALANCE

    def now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def set_mode(self, mode: str, note: str = ''):
        if mode != self.mode:
            self.mode = mode
            self.mode_since = self.now()
            self.mode_pub.publish(String(data=mode))
            self.get_logger().info(f'{mode}{"  " + note if note else ""}')

    def on_joint_states(self, msg: JointState):
        try:
            ib = msg.name.index('body_joint')
            iw = msg.name.index('wheel_joint')
        except ValueError:
            return

        angle = msg.position[ib] - self.angle_offset
        rate = msg.velocity[ib]
        wheel_speed = msg.velocity[iw]
        in_range = self.onboard_balancing if self.onboard else abs(angle) < self.engage_angle
        t = self.now()

        torque = 0.0
        if self.mode == BALANCE:
            moving_away = angle * rate > 0.0
            lost = (abs(angle) > self.catch_angle
                    or (abs(angle) > self.disengage_angle and moving_away))
            if self.onboard:
                lost = not self.onboard_balancing   # the ESP32 decides and drives the motor
            if lost:
                self.set_mode(FALLEN, 'lost balance')
            elif not self.onboard:
                state = np.concatenate([[angle, rate, wheel_speed], self.recent_torques])
                torque = -float(self.k @ state)

        elif self.mode == SPIN_UP:
            error = self.jump_target - wheel_speed
            torque = float(np.clip(0.01 * error, -self.spin_up_torque, self.spin_up_torque))
            if abs(error) >= self.spin_tolerance:
                self.settled_since = None
            elif self.settled_since is None:
                self.settled_since = t
            settled = (self.settled_since is not None
                       and t - self.settled_since >= self.spin_settle_time)
            # Fallback: noisy speed readings must not stall the jump forever.
            timed_out = (t - self.mode_since > self.spin_timeout
                         and abs(error) < 1.5 * self.spin_tolerance)
            if settled or timed_out:
                self.brake_pub.publish(Empty())
                torque = 0.0
                self.settled_since = None
                self.set_mode(JUMPING, f'brake at {wheel_speed:.1f} rad/s')

        elif self.mode == JUMPING:
            caught = self.onboard_balancing if self.onboard else abs(angle) < self.catch_angle
            if caught:
                self.set_mode(BALANCE, f'caught (margin {self.jump_margin:.2f})')
            elif t - self.mode_since > self.jump_timeout:
                self.jump_margin = min(self.jump_margin + self.jump_margin_step,
                                       self.jump_margin_max)
                self.set_mode(FALLEN, f'jump fell short, retrying with margin {self.jump_margin:.2f}')

        else:  # FALLEN
            if in_range:
                self.set_mode(BALANCE)
            else:
                lying = abs(angle) > FALL_ANGLE - 0.02 and abs(rate) < 0.05
                if not lying:
                    self.resting_since = None
                elif self.resting_since is None:
                    self.resting_since = t
                elif self.auto_jump_up and t - self.resting_since > self.rest_time:
                    # Lying at -45 deg needs a positive body rate, so spin the wheel positive.
                    direction = -math.copysign(1.0, angle)
                    self.jump_target = direction * self.jump_margin * jump_wheel_speed(
                        self.params, abs(angle))
                    self.resting_since = None
                    self.set_mode(SPIN_UP, f'target wheel speed {self.jump_target:.1f} rad/s')

        torque = float(np.clip(torque, -self.params.max_torque, self.params.max_torque))
        if self.delay_steps:
            self.recent_torques = self.recent_torques[1:] + [torque if self.mode == BALANCE else 0.0]
        self.torque_pub.publish(Float64(data=torque))


def main(args=None):
    rclpy.init(args=args)
    node = BalanceController()
    try:
        spin_until_shutdown(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
