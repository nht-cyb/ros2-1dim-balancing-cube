"""State estimator: raw IMU + wheel encoder -> /joint_states for the controller.

Subscribes:  /imu/data_raw         (sensor_msgs/Imu)         from the sim or the ESP32 driver
             /cubli/wheel_encoder  (sensor_msgs/JointState)  wheel angle [rad]
Publishes:   /joint_states         (sensor_msgs/JointState)  body_joint + wheel_joint estimates

The cube must lie still for the first `calib_samples` IMU samples (gyro bias calibration);
nothing is published until then, so the controller (and the sim) wait for it.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState

from cubli_control.estimator import TiltEstimator, WheelSpeedEstimator
from cubli_control.ros_utils import spin_until_shutdown


class StateEstimator(Node):

    def __init__(self):
        super().__init__('cubli_estimator')
        self.declare_parameter('time_constant', 0.5)
        self.declare_parameter('imu_offset', 0.106)
        self.declare_parameter('calib_samples', 200)
        self.declare_parameter('wheel_speed_window', 8)

        gp = lambda name: self.get_parameter(name).value  # noqa: E731
        self.tilt = TiltEstimator(time_constant=gp('time_constant'), imu_offset=gp('imu_offset'),
                                  calib_samples=int(gp('calib_samples')))
        self.wheel = WheelSpeedEstimator(window=int(gp('wheel_speed_window')))
        self.wheel_angle = 0.0
        self.last_imu_t = None

        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.create_subscription(Imu, 'imu/data_raw', self.on_imu, 50)
        self.create_subscription(JointState, 'cubli/wheel_encoder', self.on_encoder, 50)
        self.get_logger().info('Calibrating gyro bias: keep the cube still ...')

    def on_encoder(self, msg: JointState):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.wheel_angle = msg.position[0]
        self.wheel.update(t, self.wheel_angle)

    def on_imu(self, msg: Imu):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        dt = 0.0 if self.last_imu_t is None else min(max(t - self.last_imu_t, 0.0), 0.05)
        self.last_imu_t = t
        gyro_x = msg.angular_velocity.x
        acc_y, acc_z = msg.linear_acceleration.y, msg.linear_acceleration.z

        if not self.tilt.ready:
            if self.tilt.calibrate(gyro_x, acc_y, acc_z):
                self.get_logger().info(
                    f'Calibrated: gyro bias {self.tilt.bias:+.4f} rad/s, '
                    f'start tilt {self.tilt.angle:+.3f} rad')
            return

        angle, rate = self.tilt.update(gyro_x, acc_y, acc_z, dt)
        out = JointState()
        out.header.stamp = msg.header.stamp
        out.name = ['body_joint', 'wheel_joint']
        out.position = [float(angle), float(self.wheel_angle)]
        out.velocity = [float(rate), float(self.wheel.speed)]
        self.joint_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = StateEstimator()
    try:
        spin_until_shutdown(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
