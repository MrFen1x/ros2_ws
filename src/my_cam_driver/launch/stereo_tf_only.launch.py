from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # odom → base_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', '1', 'odom', 'base_link']
        ),
        # base_link → camera_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0.2', '0', '0.3', '0', '0', '0', '1', 'base_link', 'camera_link']
        ),
        # camera_link → left_camera_frame
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', '1', 'camera_link', 'left_camera_frame']
        ),
        # camera_link → right_camera_frame
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '-0.06', '0', '0', '0', '0', '1', 'camera_link', 'right_camera_frame']
        ),
    ])
