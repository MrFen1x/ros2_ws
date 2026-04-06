from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    
    # === 1. Нода камеры (уже есть) ===
    camera_node = Node(
        package='my_cam_driver',
        executable='stereo_usb_driver',
        name='stereo_camera',
        output='screen',
        parameters=[{
            'width': 640,
            'height': 480,
            'fps': 10,
            'use_compression': False,  # Для stereo_image_proc нужен RAW
            'jpeg_quality': 60,
        }]
    )

    # === 2. Stereo Image Proc (disparity → pointcloud) ===
    stereo_image_proc = Node(
        package='stereo_image_proc',
        executable='stereo_image_proc',
        name='stereo_image_proc',
        output='screen',
        remappings=[
            ('left/image_rect', '/stereo/left/image_raw'),
            ('left/camera_info', '/stereo/left/camera_info'),
            ('right/image_rect', '/stereo/right/image_raw'),
            ('right/camera_info', '/stereo/right/camera_info'),
        ],
        parameters=[{
            'approximate_sync': True,
            'approximate_sync_tolerance': '0.01',
            'disparity_range': 128,
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
            ('stereo_camera', '/stereo'),  # Базовый топик
        ],
        parameters=[{
            # Общие настройки
            'frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'publish_tf': False,  # Пока без одометрии
            'approx_sync': True,
            'visual_odometry': True, 
            
            # Настройки стерео
            'stereo': True,
            'subscribe_stereo': True,
            'stereo_sync': True,
            
            # Качество карты
            'RGBD/ProximityBySpace': 'true',
            'RGBD/OptimizeFromGraphEnd': 'false',
            'Grid/FromDepth': 'true',
            
            # Для WiFi - уменьшить трафик
            'Rtabmap/DetectionRate': '1.0',  # 1 Hz
            'RGBD/ImagePreDecimation': '2',   # Уменьшить разрешение
        }]
    )

    return LaunchDescription([
        camera_node,
        stereo_image_proc,
        rtabmap_node,
    ])
