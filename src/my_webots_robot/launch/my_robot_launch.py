from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    webots_pkg = get_package_share_directory('webots_ros2_driver')
    my_pkg = get_package_share_directory('my_webots_robot')

    world = os.path.join(my_pkg, 'worlds', 'my_scene.wbt')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(webots_pkg, 'launch', 'driver.launch.py')
            ),
            launch_arguments={
                'world': world,
                'additional_robot_components': 'my_webots_robot::MyRobotController'
            }.items(),
        ),
    ])
