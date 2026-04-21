from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # === 1. TF Tree ===
        Node(package='tf2_ros', executable='static_transform_publisher', name='tf_map_odom',
             arguments=['0', '0', '0', '0', '0', '0', '1', 'map', 'odom']),
        Node(package='tf2_ros', executable='static_transform_publisher', name='tf_odom_base',
             arguments=['0', '0', '0', '0', '0', '0', '1', 'odom', 'base_link']),
        Node(package='tf2_ros', executable='static_transform_publisher', name='tf_base_cam',
             arguments=['0.2', '0', '0.3', '0', '0', '0', '1', 'base_link', 'camera_link']),
        Node(package='tf2_ros', executable='static_transform_publisher', name='tf_cam_left',
             arguments=['0', '0', '0', '0', '0', '0', '1', 'camera_link', 'left_camera_frame']),
        Node(package='tf2_ros', executable='static_transform_publisher', name='tf_cam_right',
             arguments=['0', '-0.06', '0', '0', '0', '0', '1', 'camera_link', 'right_camera_frame']),

        # === 2. Camera Node (Публикует RAW) ===
        Node(package='my_cam_driver', executable='stereo_usb_driver', name='stereo_camera',
             parameters=[{
                 'width': 640,
                 'height': 480,
                 'fps': 10
             }]),

        # === 3. Rectification (Исправление искажений) ===
        # RTAB-Map не может работать с сырыми данными, ему нужно выпрямление
        
        # Left Rectify
        Node(
            package='image_proc',
            executable='rectify_node',
            name='rectify_left',
            remappings=[
                ('image_raw', '/stereo/left/image_raw'),
                ('camera_info', '/stereo/left/camera_info'),
                ('image_rect', '/stereo/left/image_rect')
            ]
        ),

        # Right Rectify
        Node(
            package='image_proc',
            executable='rectify_node',
            name='rectify_right',
            remappings=[
                ('image_raw', '/stereo/right/image_raw'),
                ('camera_info', '/stereo/right/camera_info'),
                ('image_rect', '/stereo/right/image_rect')
            ]
        ),

        # === 4. Stereo Odometry (Подписывается на ИСПРАВЛЕННЫЕ изображения) ===
        Node(package='rtabmap_odom', executable='stereo_odometry', name='stereo_odometry',
             output='screen',
             parameters=[{
                'frame_id': 'base_link',
                'odom_frame_id': 'odom',
                'publish_tf': False,
                'approx_sync': True,
                'queue_size': 10,
                
                'Odom/Strategy': '0',
                'Odom/EstimationType': '1',
                'Odom/MinInliers': '10',
                'GFTT/MinDistance': '5',    # Уменьшил для лучшего поиска точек
                'GFTT/QualityLevel': '0.001'
             }],
             remappings=[
                ('left/image_rect', '/stereo/left/image_rect'),   # <-- Теперь тут rect!
                ('right/image_rect', '/stereo/right/image_rect'), # <-- И тут rect!
                ('left/camera_info', '/stereo/left/camera_info'),
                ('right/camera_info', '/stereo/right/camera_info'),
                ('odom', '/odom'),
             ]),

        # === 5. RTAB-Map SLAM (Подписывается на ИСПРАВЛЕННЫЕ изображения) ===
        Node(package='rtabmap_slam', executable='rtabmap', name='rtabmap',
             output='screen',
             arguments=['-d'],
             parameters=[{
                'frame_id': 'base_link',
                'odom_frame_id': 'odom',
                'publish_tf': True,
                'subscribe_stereo': True,
                'approx_sync': True,
                'queue_size': 10,
                'Rtabmap/DetectionRate': '1.0',
             }],
             remappings=[
                ('left/image_rect', '/stereo/left/image_rect'),   # <-- Теперь тут rect!
                ('right/image_rect', '/stereo/right/image_rect'), # <-- И тут rect!
                ('left/camera_info', '/stereo/left/camera_info'),
                ('right/camera_info', '/stereo/right/camera_info'),
                ('odom', '/odom'),
             ]),
    ])
