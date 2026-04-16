from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_path = get_package_share_directory('my_robot_description')
    xacro_file = os.path.join(pkg_path, 'urdf', 'robot.urdf.xacro')
    rviz_config = os.path.join(pkg_path, 'rviz', 'config.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_gui',
            default_value='true',
            description='Use joint state publisher GUI'
        ),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': ParameterValue(
                    Command(['xacro ', xacro_file]),
                    value_type=str
                ),
                'publish_frequency': 30.0,
            }]
        ),

        # Joint State Publisher GUI
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            output='screen'
        ),

        # RViz2 (запуск с задержкой чтобы robot_state_publisher успел стартовать)
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='rviz2',
                    executable='rviz2',
                    arguments=['-d', rviz_config],
                    output='screen'
                ),
            ]
        ),
    ])
