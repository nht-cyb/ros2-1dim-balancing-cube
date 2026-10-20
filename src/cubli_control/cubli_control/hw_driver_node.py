"""Hardware driver: ESP32 (MPU6050 + Nidec 24H) over USB serial <-> ROS.

Publishes the same raw sensor topics as the simulator, so the estimator and the
controller run unchanged on the real cube:
Publishes:   /imu/data_raw         (sensor_msgs/Imu)         body-frame axes (see imu_axes)
             /cubli/wheel_encoder  (sensor_msgs/JointState)  wheel angle [rad]
Subscribes:  /cubli/wheel_torque   (std_msgs/Float64)  torque [N m] -> Nidec effort
             /cubli/brake          (std_msgs/Empty)    brake pulse (jump-up)
             /cubli/raw_effort     (std_msgs/Float64)  bring-up only: effort in [-1, 1],
                                                       bypassing the motor model

On-board balance (onboard_balance: true): the ESP32 runs the estimator and the LQR itself,
so the balance loop no longer waits for the PC. On connect the driver designs K from the
controller parameters (q_*, r_torque, control_rate_hz, max_torque), sends it with the
angles, motor and estimator parameters, and switches the ESP32 to mode 1. Then instead of
the estimator node it publishes:
             /joint_states         (sensor_msgs/JointState)  the ESP32's own estimate
             /cubli/onboard_mode   (std_msgs/String, latched) CALIBRATING | IDLE | BALANCE
The controller (onboard_balance: true) keeps the jump-up state machine and takes BALANCE
from /cubli/onboard_mode; while the ESP32 balances it ignores /cubli/wheel_torque.

The serial port is reopened automatically if the ESP32 is unplugged.
"""

import threading
import time

import numpy as np
import rclpy
import serial
from rclpy._rclpy_pybind11 import RCLError
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Empty, Float64, String

from cubli_control.estimator import WheelSpeedEstimator
from cubli_control.hardware import (
    MODE_CALIBRATING, MODE_NAMES, EstimateFrame, NidecMotor, SensorFrame, counts_to_rad,
    encode, onboard_setup, parse, remap_axes)
from cubli_control.lqr import lqr_gain
from cubli_control.model import CubliParams
from cubli_control.ros_utils import spin_until_shutdown


class HardwareDriver(Node):

    def __init__(self):
        super().__init__('cubli_hw_driver')
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 460800)
        self.declare_parameter('imu_axes', ['+x', '+y', '+z'])
        self.declare_parameter('encoder_cpr', 400)
        self.declare_parameter('encoder_sign', 1)
        self.declare_parameter('stall_torque', NidecMotor.stall_torque)
        self.declare_parameter('no_load_speed', NidecMotor.no_load_speed)
        self.declare_parameter('max_effort', 1.0)
        self.declare_parameter('brake_ms', 300)
        self.declare_parameter('command_timeout', 0.1)
        self.declare_parameter('onboard_balance', False)
        # On-board mode only; the launch file copies these from cubli_controller and
        # cubli_estimator in cubli_hw.yaml so both loops use the same tuning.
        self.declare_parameter('control_rate_hz', 200.0)
        self.declare_parameter('max_torque', 0.08)
        self.declare_parameter('engage_angle', 0.12)
        self.declare_parameter('disengage_angle', 0.16)
        self.declare_parameter('catch_angle', 0.4)
        self.declare_parameter('jump_timeout', 1.5)
        self.declare_parameter('angle_offset', 0.0)
        self.declare_parameter('q_angle', 100.0)
        self.declare_parameter('q_rate', 0.3)
        self.declare_parameter('q_wheel_speed', 1e-4)
        self.declare_parameter('r_torque', 100.0)
        self.declare_parameter('lqr_delay_steps', 0)
        self.declare_parameter('time_constant', 0.5)
        self.declare_parameter('imu_offset', 0.106)

        gp = lambda name: self.get_parameter(name).value  # noqa: E731
        self.port, self.baud = gp('port'), int(gp('baud'))
        self.imu_axes = list(gp('imu_axes'))
        self.cpr, self.enc_sign = int(gp('encoder_cpr')), int(gp('encoder_sign'))
        self.motor = NidecMotor(gp('stall_torque'), gp('no_load_speed'))
        self.max_effort = gp('max_effort')
        self.brake_ms = int(gp('brake_ms'))
        self.timeout = gp('command_timeout')
        self.onboard = bool(gp('onboard_balance'))
        if self.onboard:
            self.setup_lines = self.onboard_commands(gp)

        self.wheel = WheelSpeedEstimator()
        self.serial = None
        self.write_lock = threading.Lock()
        self.time_offset = None       # ROS time - ESP32 time [s]
        self.last_cmd = 0.0
        self.frames = self.bad_lines = 0
        self.wheel_angle = 0.0
        self.onboard_mode = None
        self.last_estimate = self.last_setup = 0.0

        self.imu_pub = self.create_publisher(Imu, 'imu/data_raw', 10)
        self.encoder_pub = self.create_publisher(JointState, 'cubli/wheel_encoder', 10)
        if self.onboard:
            self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
            self.mode_pub = self.create_publisher(
                String, 'cubli/onboard_mode',
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(Float64, 'cubli/wheel_torque', self.on_torque, 10)
        self.create_subscription(Empty, 'cubli/brake', self.on_brake, 10)
        self.create_subscription(Float64, 'cubli/raw_effort', self.on_raw_effort, 10)
        self.create_timer(self.timeout, self.watchdog)
        self.create_timer(5.0, self.report)

        self.running = True
        self.connected_at = time.monotonic()
        self.reader = threading.Thread(target=self.read_loop, daemon=True)
        self.reader.start()

    # ---- serial ---------------------------------------------------------------
    def send(self, data: bytes):
        with self.write_lock:
            if self.serial is None:
                return
            try:
                self.serial.write(data)
            except (serial.SerialException, OSError) as exc:
                self.get_logger().warn(f'Serial write failed: {exc}')

    def read_loop(self):
        buffer = b''
        while self.running:
            if self.serial is None:
                try:
                    port = serial.serial_for_url(self.port, baudrate=self.baud, timeout=0.05)
                except (serial.SerialException, OSError) as exc:
                    self.get_logger().warn(f'Cannot open {self.port}: {exc}; retrying',
                                           throttle_duration_sec=5.0)
                    time.sleep(1.0)
                    continue
                with self.write_lock:
                    self.serial = port
                self.time_offset = None
                self.get_logger().info(f'Connected to ESP32 on {self.port}')
                self.send(encode('E', 1))
                self.connected_at = time.monotonic()
                if self.onboard:
                    self.send_setup()
            try:
                # Read whatever has arrived (at least 1 byte, up to the timeout). A fixed
                # read(512) would wait for ~6 frames and deliver them 30 ms late in a burst.
                buffer += self.serial.read(max(1, self.serial.in_waiting))
            except (serial.SerialException, OSError) as exc:
                self.get_logger().error(f'Serial read failed: {exc}; reconnecting')
                with self.write_lock:
                    self.serial = None
                continue
            *lines, buffer = buffer.split(b'\n')
            for line in lines:
                if not self.context.ok():
                    return   # ROS is shutting down; stop() sends the motor-off commands
                try:
                    self.handle_line(line)
                except RCLError:
                    return   # context closed between the check and the publish

    def handle_line(self, line: bytes):
        msg = parse(line)
        if msg is None:
            if line.strip():
                self.bad_lines += 1
            return
        kind, fields = msg
        if kind == 'I':
            self.get_logger().info(f'ESP32: {",".join(fields)}')
        elif kind == 'S':
            try:
                frame = SensorFrame.from_fields(fields)
            except ValueError:
                self.bad_lines += 1
                return
            self.publish_frame(frame)
        elif kind == 'X' and self.onboard:
            try:
                estimate = EstimateFrame.from_fields(fields)
            except ValueError:
                self.bad_lines += 1
                return
            self.publish_estimate(estimate)

    def onboard_commands(self, gp):
        delay = int(gp('lqr_delay_steps'))
        if delay:
            self.get_logger().warn(
                f'lqr_delay_steps={delay} ignored: the on-board LQR uses the plain 3-state gain')
        params = CubliParams(max_torque=gp('max_torque'))
        k = lqr_gain(params, 1.0 / gp('control_rate_hz'),
                     [gp('q_angle'), gp('q_rate'), gp('q_wheel_speed')], gp('r_torque')).ravel()
        engage = gp('engage_angle')
        values = {
            'engage': engage,
            'disengage': max(gp('disengage_angle'), engage),
            'catch': max(gp('catch_angle'), engage),
            'jump_timeout': gp('jump_timeout'),
            'max_torque': params.max_torque,
            'stall_torque': self.motor.stall_torque,
            'no_load_speed': self.motor.no_load_speed,
            'max_effort': self.max_effort,
            'angle_offset': gp('angle_offset'),
            'tau': gp('time_constant'),
            'imu_offset': gp('imu_offset'),
            'cpr': self.cpr,
            'enc_sign': self.enc_sign,
        }
        self.get_logger().info(
            f'On-board balance: K = {np.array2string(k, precision=4)} sent to the ESP32')
        return onboard_setup(k, values, self.imu_axes)

    def send_setup(self):
        self.last_setup = time.monotonic()
        for line in self.setup_lines:
            self.send(line)

    def publish_estimate(self, est: EstimateFrame):
        now = time.monotonic()
        self.last_estimate = now
        if est.mode != self.onboard_mode:
            self.onboard_mode = est.mode
            name = MODE_NAMES.get(est.mode, str(est.mode))
            self.mode_pub.publish(String(data=name))
            self.get_logger().info(f'ESP32 on-board mode: {name}')
        if est.mode == MODE_CALIBRATING or self.time_offset is None:
            return   # like the estimator node: nothing until calibrated
        out = JointState()
        out.header.stamp = rclpy.time.Time(seconds=est.t_us * 1e-6 + self.time_offset).to_msg()
        out.name = ['body_joint', 'wheel_joint']
        out.position = [est.angle, self.wheel_angle]
        out.velocity = [est.rate, est.wheel_speed]
        self.joint_pub.publish(out)

    def publish_frame(self, frame: SensorFrame):
        self.frames += 1
        now = self.get_clock().now().nanoseconds * 1e-9
        esp_t = frame.t_us * 1e-6
        # Stamp with the ESP32 clock (jitter-free dt) mapped onto ROS time. Serial bursts
        # only delay frames, so resync only on a big jump (reconnect, micros() wrap).
        if self.time_offset is None or abs(now - (esp_t + self.time_offset)) > 0.25:
            self.time_offset = now - esp_t
        t = esp_t + self.time_offset
        stamp = rclpy.time.Time(seconds=t).to_msg()

        gyro = remap_axes(frame.gyro, self.imu_axes)
        accel = remap_axes(frame.accel, self.imu_axes)
        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = 'imu_link'
        imu.orientation_covariance[0] = -1.0
        imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z = gyro
        imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z = accel
        self.imu_pub.publish(imu)

        angle = counts_to_rad(self.enc_sign * frame.encoder, self.cpr)
        self.wheel_angle = angle
        self.wheel.update(t, angle)
        enc = JointState()
        enc.header.stamp = stamp
        enc.name = ['wheel_joint']
        enc.position = [angle]
        self.encoder_pub.publish(enc)

    # ---- commands ---------------------------------------------------------------
    def on_torque(self, msg: Float64):
        self.last_cmd = time.monotonic()
        effort = self.motor.effort_for(msg.data, self.wheel.speed, self.max_effort)
        self.send(encode('T', f'{effort:.3f}'))

    def on_raw_effort(self, msg: Float64):
        self.last_cmd = time.monotonic()
        effort = max(-self.max_effort, min(self.max_effort, msg.data))
        self.send(encode('T', f'{effort:.3f}'))

    def on_brake(self, msg: Empty):
        self.get_logger().info(f'Brake pulse {self.brake_ms} ms')
        self.send(encode('B', self.brake_ms))

    def watchdog(self):
        now = time.monotonic()
        if now - self.last_cmd > self.timeout:
            self.send(encode('T', '0.000'))   # also the heartbeat the on-board mode needs
        # An ESP32 that reset by itself (brownout) comes back in passthrough: set it up again.
        if (self.onboard and self.serial is not None and now - self.connected_at > 1.0
                and now - self.last_estimate > 1.0 and now - self.last_setup > 2.0):
            self.get_logger().warn('No on-board estimates from the ESP32; sending the setup again')
            self.send_setup()

    def report(self):
        if self.serial is not None:
            self.get_logger().info(
                f'{self.frames / 5.0:.0f} frames/s, {self.bad_lines} bad lines',
                throttle_duration_sec=30.0)
            if self.frames == 0:
                self.get_logger().warn('No sensor frames from the ESP32')
        self.frames = self.bad_lines = 0

    def stop(self):
        self.send(encode('T', '0.000'))
        if self.onboard:
            self.send(encode('M', 0))
        self.send(encode('E', 0))
        self.running = False
        self.reader.join(timeout=1.0)
        if self.serial is not None:
            self.serial.close()


def main(args=None):
    rclpy.init(args=args)
    node = HardwareDriver()
    try:
        spin_until_shutdown(node)
    finally:
        node.stop()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
