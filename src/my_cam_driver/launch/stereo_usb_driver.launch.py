from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_cam_driver',
            executable='stereo_usb_driver',
            name='stereo_usb_driver',
            output='screen',
            parameters=[{
                'camera_device': '/dev/video0',
                'width': 640,
                'height': 480,
                'fps': 15,
            }]
        )
    ])
