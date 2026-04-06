from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    
    # === 1. Нода камеры ===
    camera_node = Node(
        package='my_cam_driver',
        executable='stereo_usb_driver',
        name='stereo_camera',
        output='screen',
        parameters=[{
            'width': 640,
            'height': 480,
            'fps': 10,
            'use_compression': False,  # Для 3D нужен RAW
        }]
    )

    # === 2. Stereo Image Proc (disparity → pointcloud) ===
    # В Jazzy это отдельные ноды
    
    # Disparity из стерео пары
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
            'approximate_sync_tolerance_sec': 0.01,
        }]
    )

    # PointCloud2 из disparity
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
            'approximate_sync_tolerance_sec': 0.01,
        }]
    )

    # === 3. RTAB-Map (3D SLAM) ===
    rtabmap_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        remappings=[
            ('left/image_rect', '/stereo/left/image_raw'),
            ('right/image_rect', '/stereo/right/image_raw'),
            ('left/camera_info', '/stereo/left/camera_info'),
            ('right/camera_info', '/stereo/right/camera_info'),
        ],
        parameters=[{
            # Общие настройки
            'frame_id': 'base_link',
            'publish_tf': False,
            'approx_sync': True,
            
            # Настройки стерео
            'subscribe_stereo': True,
            
            # Для WiFi - реже обновления
            'Rtabmap/DetectionRate': '1.0',
            'RGBD/ImagePreDecimation': '2',
        }]
    )

    return LaunchDescription([
        camera_node,
        disparity_node,
        point_cloud_node,
        rtabmap_node,
    ])
