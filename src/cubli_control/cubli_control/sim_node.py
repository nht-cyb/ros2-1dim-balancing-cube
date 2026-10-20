"""Physics simulator for the 1D Cubli.

Subscribes:  /cubli/wheel_torque (std_msgs/Float64)  motor torque command [N m]
             /cubli/push         (std_msgs/Float64)  impulse: adds this to the body rate [rad/s]
             /cubli/brake        (std_msgs/Empty)    lock the wheel to the body instantly (jump-up)
Services:    /cubli/reset        (std_srvs/Trigger)  restart from initial_angle

With sensors:=true (default) it behaves like the real hardware driver and publishes
raw sensor data for the estimator, plus the true state for analysis:
             /imu/data_raw              (sensor_msgs/Imu)         noisy, biased MPU6050 model
             /cubli/wheel_encoder       (sensor_msgs/JointState)  quantised wheel angle
             /cubli/ground_truth/joint_states (sensor_msgs/JointState)
With sensors:=false it publishes the true state straight to /joint_states (Phase 1).
"""

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Empty, Float64
from std_srvs.srv import Trigger

from cubli_control.model import (
    FALL_ANGLE, CubliParams, apply_ground_contact, brake, dynamics, rk4_step)
from cubli_control.sensors import SensorModel, SensorParams
from cubli_control.ros_utils import spin_until_shutdown


class CubliSim(Node):

    def __init__(self):
        super().__init__('cubli_sim')
        self.declare_parameter('rate_hz', 200.0)
        self.declare_parameter('substeps', 5)
        self.declare_parameter('initial_angle', -FALL_ANGLE)
        self.declare_parameter('max_torque', CubliParams.max_torque)
        self.declare_parameter('command_timeout', 0.1)
        self.declare_parameter('brake_efficiency', 1.0)
        self.declare_parameter('start_timeout', 5.0)
        self.declare_parameter('sensors', True)
        defaults = SensorParams()
        for name in ('imu_offset', 'gyro_noise_std', 'gyro_bias', 'accel_noise_std',
                     'accel_bias_y', 'accel_bias_z'):
            self.declare_parameter(name, getattr(defaults, name))
        self.declare_parameter('encoder_cpr', defaults.encoder_cpr)

        gp = lambda name: self.get_parameter(name).value  # noqa: E731
        self.params = CubliParams(max_torque=gp('max_torque'))
        self.dt = 1.0 / gp('rate_hz')
        self.substeps = int(gp('substeps'))
        self.timeout = gp('command_timeout')
        self.brake_efficiency = gp('brake_efficiency')
        self.sensors_enabled = gp('sensors')
        self.sensors = SensorModel(SensorParams(
            imu_offset=gp('imu_offset'), gyro_noise_std=gp('gyro_noise_std'),
            gyro_bias=gp('gyro_bias'), accel_noise_std=gp('accel_noise_std'),
            accel_bias_y=gp('accel_bias_y'), accel_bias_z=gp('accel_bias_z'),
            encoder_cpr=int(gp('encoder_cpr'))))

        self.torque = 0.0
        self.last_cmd_time = None
        self.reset_state()

        # Physics stays frozen until the controller sends its first command (or
        # start_timeout passes), so a slow-starting controller does not miss the cube.
        self.running = False
        self.start_time = self.get_clock().now()
        self.start_timeout = gp('start_timeout')

        if self.sensors_enabled:
            self.joint_pub = self.create_publisher(
                JointState, 'cubli/ground_truth/joint_states', 10)
            self.imu_pub = self.create_publisher(Imu, 'imu/data_raw', 10)
            self.encoder_pub = self.create_publisher(JointState, 'cubli/wheel_encoder', 10)
        else:
            self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.create_subscription(Float64, 'cubli/wheel_torque', self.on_torque, 10)
        self.create_subscription(Float64, 'cubli/push', self.on_push, 10)
        self.create_subscription(Empty, 'cubli/brake', self.on_brake, 10)
        self.create_service(Trigger, 'cubli/reset', self.on_reset)
        self.create_timer(self.dt, self.step)
        self.get_logger().info(
            f'Cubli sim ready at {1.0 / self.dt:.0f} Hz, initial tilt {self.x[0]:.3f} rad, '
            f'sensors {"on" if self.sensors_enabled else "off (perfect state)"}; '
            'waiting for the controller')

    def reset_state(self):
        self.x = np.array([self.get_parameter('initial_angle').value, 0.0, 0.0, 0.0])

    def on_torque(self, msg: Float64):
        limit = self.params.max_torque
        self.torque = float(np.clip(msg.data, -limit, limit))
        self.last_cmd_time = self.get_clock().now()
        if not self.running:
            self.running = True
            self.get_logger().info('Controller connected, physics started')

    def on_brake(self, msg: Empty):
        before = self.x[3]
        self.x = brake(self.x, self.params, self.brake_efficiency)
        self.get_logger().info(
            f'Brake: wheel {before:.1f} rad/s -> body rate {self.x[1]:.2f} rad/s')

    def on_push(self, msg: Float64):
        self.x[1] += msg.data
        self.get_logger().info(f'Push: body rate += {msg.data:.2f} rad/s')

    def on_reset(self, request, response):
        self.reset_state()
        self.torque = 0.0
        response.success = True
        response.message = f'Reset to tilt {self.x[0]:.3f} rad'
        return response

    def step(self):
        if not self.context.ok():
            return
        if not self.running and (
                (self.get_clock().now() - self.start_time).nanoseconds * 1e-9 > self.start_timeout):
            self.running = True
            self.get_logger().info('No controller command yet, starting physics anyway')

        torque = self.torque
        if self.last_cmd_time is None or (
                (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9 > self.timeout):
            torque = 0.0

        xdot = np.zeros(4)
        if self.running:
            h = self.dt / self.substeps
            for _ in range(self.substeps):
                self.x = apply_ground_contact(rk4_step(self.x, torque, h, self.params))
            xdot = dynamics(self.x, torque, self.params)

        stamp = self.get_clock().now().to_msg()
        msg = JointState()
        msg.header.stamp = stamp
        msg.name = ['body_joint', 'wheel_joint']
        msg.position = [float(self.x[0]), float(self.x[2])]
        msg.velocity = [float(self.x[1]), float(self.x[3])]
        msg.effort = [0.0, float(torque)]
        self.joint_pub.publish(msg)

        if self.sensors_enabled:
            self.publish_sensors(stamp, xdot)

    def publish_sensors(self, stamp, xdot):
        gyro, accel = self.sensors.imu(self.x, xdot)
        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = 'imu_link'
        imu.orientation_covariance[0] = -1.0  # no orientation estimate
        imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z = map(float, gyro)
        (imu.linear_acceleration.x, imu.linear_acceleration.y,
         imu.linear_acceleration.z) = map(float, accel)
        self.imu_pub.publish(imu)

        enc = JointState()
        enc.header.stamp = stamp
        enc.name = ['wheel_joint']
        enc.position = [self.sensors.encoder_angle(self.x[2])]
        self.encoder_pub.publish(enc)


def main(args=None):
    rclpy.init(args=args)
    node = CubliSim()
    try:
        spin_until_shutdown(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
