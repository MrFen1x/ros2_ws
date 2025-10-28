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
                'left_device': '/dev/video0',
                'right_device': '/dev/video1',
                'left_topic': '/stereo/left/image_raw',
                'right_topic': '/stereo/right/image_raw',
                'left_info_url': 'file:///home/ubuntu/calib/left.yaml',
                'right_info_url': 'file:///home/ubuntu/calib/right.yaml',
                'fps': 15.0,
                'width': 640,
                'height': 480,
            }]
        )
    ])
