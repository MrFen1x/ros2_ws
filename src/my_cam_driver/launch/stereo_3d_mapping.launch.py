from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    
    # === 1. Статические TF ===
    # odom → base_link (пока статичный, потом можно подключить одометрию)
    # Формат: [x, y, z, roll, pitch, yaw, quaternion_w, parent, child]
    odom_to_baselink = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_to_baselink',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'odom', 'base_link']
    )

    # base_link → camera_link (камера на роботе)
    baselink_to_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='baselink_to_camera',
        arguments=['0.2', '0', '0.3', '0', '0', '0', '1', 'base_link', 'camera_link']
    )

    # camera_link → left_camera_frame
    camera_to_left = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_to_left',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'camera_link', 'left_camera_frame']
    )

    # camera_link → right_camera_frame
    camera_to_right = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_to_right',
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
            'use_compression': False,  # Для 3D нужен RAW
        }]
    )

    # === 3. RTAB-Map со встроенной одометрией ===
    rtabmap_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        arguments=['-d'],  # Дебаг режим для начала
        remappings=[
            ('left/image_rect', '/stereo/left/image_raw'),
            ('right/image_rect', '/stereo/right/image_raw'),
            ('left/camera_info', '/stereo/left/camera_info'),
            ('right/camera_info', '/stereo/right/camera_info'),
        ],
        parameters=[{
            # TF настройки
            'frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'publish_tf': True,
            
            # Синхронизация
            'approx_sync': True,
            'approx_sync_max_interval': '0.01',
            
            # Настройки стерео
            'stereo': True,
            'subscribe_stereo': True,
            
            # Встроенная визуальная одометрия
            'RGBD/ProximityBySpace': 'true',
            'RGBD/OptimizeFromGraphEnd': 'false',
            
            # Для WiFi - реже обновления
            'Rtabmap/DetectionRate': '1.0',
            'RGBD/ImagePreDecimation': '2',
        }]
    )

    return LaunchDescription([
        odom_to_baselink,
        baselink_to_camera,
        camera_to_left,
        camera_to_right,
        camera_node,
        rtabmap_node,
    ])
