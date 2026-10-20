"""Run the real Cubli: ESP32 driver, state estimator, controller, robot_state_publisher, RViz.

    ros2 launch cubli_control cubli_hw.launch.py                       # /dev/ttyUSB0
    ros2 launch cubli_control cubli_hw.launch.py port:=/dev/ttyACM0
    ros2 launch cubli_control cubli_hw.launch.py fake_hardware:=true   # no board: fake ESP32
    ros2 launch cubli_control cubli_hw.launch.py controller:=false     # sensors only (bring-up)
    ros2 launch cubli_control cubli_hw.launch.py onboard:=true         # ESP32 runs the balance loop

Keep the cube still, lying on a face, for the first second (gyro calibration).
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

FAKE_PORT = '/tmp/cubli_fake_esp32'
# Tuning the driver passes to the ESP32 in on-board mode, copied from the other nodes' sections.
ONBOARD_SHARED = {
    'cubli_controller': ['control_rate_hz', 'max_torque', 'engage_angle', 'disengage_angle',
                         'catch_angle', 'jump_timeout', 'angle_offset', 'q_angle', 'q_rate',
                         'q_wheel_speed', 'r_torque', 'lqr_delay_steps'],
    'cubli_estimator': ['time_constant', 'imu_offset'],
}


def generate_launch_description():
    description_dir = get_package_share_directory('cubli_description')
    control_dir = get_package_share_directory('cubli_control')

    with open(os.path.join(description_dir, 'urdf', 'cubli_1d.urdf')) as f:
        robot_description = f.read()
    params_file = os.path.join(control_dir, 'config', 'cubli_hw.yaml')
    fake = LaunchConfiguration('fake_hardware')
    port = PythonExpression([
        "'", FAKE_PORT, "' if '", fake, "' == 'true' else '", LaunchConfiguration('port'), "'"])
    onboard = LaunchConfiguration('onboard')
    onboard_param = ParameterValue(onboard, value_type=bool)
    with open(params_file) as f:
        sections = yaml.safe_load(f)
    shared = {name: sections[node]['ros__parameters'][name]
              for node, names in ONBOARD_SHARED.items() for name in names
              if name in sections[node]['ros__parameters']}

    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('fake_hardware', default_value='false'),
        DeclareLaunchArgument('jump_up', default_value='false',
                              description='auto jump-up (needs a strong brake)'),
        DeclareLaunchArgument('controller', default_value='true'),
        DeclareLaunchArgument('onboard', default_value='false',
                              description='balance loop on the ESP32 (the PC keeps the jump-up)'),
        DeclareLaunchArgument('rviz', default_value='true'),

        Node(
            package='cubli_control',
            executable='fake_esp32',
            name='fake_esp32',
            arguments=['--link', FAKE_PORT],
            output='screen',
            condition=IfCondition(fake),
        ),
        Node(
            package='cubli_control',
            executable='hw_driver_node',
            name='cubli_hw_driver',
            parameters=[params_file, shared, {'port': port, 'onboard_balance': onboard_param}],
            output='screen',
        ),
        Node(
            package='cubli_control',
            executable='estimator_node',
            name='cubli_estimator',
            parameters=[params_file],
            output='screen',
            condition=UnlessCondition(onboard),   # the ESP32 publishes the estimate then
        ),
        Node(
            package='cubli_control',
            executable='controller_node',
            name='cubli_controller',
            parameters=[params_file, {
                'auto_jump_up': ParameterValue(LaunchConfiguration('jump_up'), value_type=bool),
                'onboard_balance': onboard_param}],
            output='screen',
            condition=IfCondition(LaunchConfiguration('controller')),
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(description_dir, 'rviz', 'cubli.rviz')],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ])
