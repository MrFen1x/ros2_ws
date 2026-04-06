from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    
    # === 1. Статические TF ===
    odom_to_baselink = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'odom', 'base_link']
    )

    baselink_to_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.2', '0', '0.3', '0', '0', '0', '1', 'base_link', 'camera_link']
    )

    camera_to_left = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'camera_link', 'left_camera_frame']
    )

    camera_to_right = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '-0.06', '0', '0', '0', '0', '1', 'camera_link', 'right_camera_frame']
    )

    # === 2. Нода камеры ===
    camera_node = Node(
        package='my_cam_driver',
        executable='stereo_usb_driver',
        name='stereo_camera',
        output='screen',
        parameters=[{
            'width': 640,
            'height': 480,
            'fps': 10,
            'use_compression': False,
        }]
    )

    # === 3. Disparity из стерео пары ===
    disparity_node = Node(
        package='stereo_image_proc',
        executable='disparity_node',
        name='disparity',
        output='screen',
        remappings=[
            ('left/image_rect', '/stereo/left/image_raw'),
            ('left/camera_info', '/stereo/left/camera_info'),
            ('right/image_rect', '/stereo/right/image_raw'),
            ('right/camera_info', '/stereo/right/camera_info'),
        ],
        parameters=[{
            'approximate_sync': True,
            'approximate_sync_tolerance_sec': 0.1,
            'stereo_algorithm': 1,  # Semi-Global Block Matching
            'disparity_range': 128,
        }]
    )

    # === 4. PointCloud2 из disparity ===
    point_cloud_node = Node(
        package='stereo_image_proc',
        executable='point_cloud_node',
        name='point_cloud',
        output='screen',
        remappings=[
            ('left/image_rect', '/stereo/left/image_raw'),
            ('left/camera_info', '/stereo/left/camera_info'),
            ('right/image_rect', '/stereo/right/image_raw'),
            ('right/camera_info', '/stereo/right/camera_info'),
        ],
        parameters=[{
            'approximate_sync': True,
            'approximate_sync_tolerance_sec': 0.1,
        }]
    )

    return LaunchDescription([
        odom_to_baselink,
        baselink_to_camera,
        camera_to_left,
        camera_to_right,
        camera_node,
        disparity_node,
        point_cloud_node,
    ])
