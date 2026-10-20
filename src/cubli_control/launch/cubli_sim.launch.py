"""Start the 1D Cubli simulator, state estimator, controller, robot_state_publisher and RViz.

    ros2 launch cubli_control cubli_sim.launch.py                   # noisy sensors + estimator
    ros2 launch cubli_control cubli_sim.launch.py sensors:=false    # perfect state (Phase 1)
    ros2 launch cubli_control cubli_sim.launch.py rviz:=false controller:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    description_dir = get_package_share_directory('cubli_description')
    control_dir = get_package_share_directory('cubli_control')

    with open(os.path.join(description_dir, 'urdf', 'cubli_1d.urdf')) as f:
        robot_description = f.read()
    params_file = os.path.join(control_dir, 'config', 'cubli.yaml')
    sensors = LaunchConfiguration('sensors')

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('controller', default_value='true'),
        DeclareLaunchArgument('sensors', default_value='true',
                              description='simulate IMU + encoder and run the estimator'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='cubli_control',
            executable='sim_node',
            name='cubli_sim',
            parameters=[params_file, {'sensors': ParameterValue(sensors, value_type=bool)}],
            output='screen',
        ),
        Node(
            package='cubli_control',
            executable='estimator_node',
            name='cubli_estimator',
            parameters=[params_file],
            output='screen',
            condition=IfCondition(sensors),
        ),
        Node(
            package='cubli_control',
            executable='controller_node',
            name='cubli_controller',
            parameters=[params_file],
            output='screen',
            condition=IfCondition(LaunchConfiguration('controller')),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(description_dir, 'rviz', 'cubli.rviz')],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ])
