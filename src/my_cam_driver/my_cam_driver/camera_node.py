#!/usr/bin/env python3
import os
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo
from sensor_msgs.srv import SetCameraInfo
from cv_bridge import CvBridge
from camera_info_manager import CameraInfoManager
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np


class StereoCameraNode(Node):
    def __init__(self):
        super().__init__('stereo_camera_node')

        self.bridge = CvBridge()
        self.cap = None

        # Настройки камеры (теперь параметризуемые)
        self.declare_parameter('camera_device', '/dev/video0')
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 15)
        
        # Настройки сжатия JPEG
        self.declare_parameter('use_compression', True)  # Включить сжатие
        self.declare_parameter('jpeg_quality', 60)       # Качество JPEG (0-100)

        self.camera_device = self.get_parameter('camera_device').value
        self.WIDTH = self.get_parameter('width').value
        self.HEIGHT = self.get_parameter('height').value
        self.FPS = self.get_parameter('fps').value
        self.use_compression = self.get_parameter('use_compression').value
        self.jpeg_quality = self.get_parameter('jpeg_quality').value

        # Папка для хранения калибровок
        calib_dir = os.path.expanduser("~/.ros/camera_info")
        os.makedirs(calib_dir, exist_ok=True)

        left_url = f"file://{calib_dir}/left.yaml"
        right_url = f"file://{calib_dir}/right.yaml"

        # Менеджеры калибровки
        self.left_info_mgr = CameraInfoManager(self, cname='left', namespace='left', url=left_url)
        self.right_info_mgr = CameraInfoManager(self, cname='right', namespace='right', url=right_url)

        self.left_info_mgr.loadCameraInfo()
        self.right_info_mgr.loadCameraInfo()

        # Сервисы для установки калибровки
        self.create_service(SetCameraInfo, 'left_camera/set_camera_info', self.handle_set_camera_info_left)
        self.create_service(SetCameraInfo, 'right_camera/set_camera_info', self.handle_set_camera_info_right)

        # QoS профиль для видео по WiFi (BEST_EFFORT)
        image_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST
        )

        # Паблишеры для СЖАТЫХ изображений (для WiFi)
        if self.use_compression:
            self.left_image_pub = self.create_publisher(CompressedImage, 'stereo/left/image_raw/compressed', image_qos)
            self.right_image_pub = self.create_publisher(CompressedImage, 'stereo/right/image_raw/compressed', image_qos)
            self.get_logger().info(f"✅ Сжатие JPEG включено (качество: {self.jpeg_quality})")
        else:
            # Паблишеры для сырых изображений
            self.left_image_pub = self.create_publisher(Image, 'stereo/left/image_raw', image_qos)
            self.right_image_pub = self.create_publisher(Image, 'stereo/right/image_raw', image_qos)
            self.get_logger().info("❌ Сжатие отключено (RAW режим)")

        self.left_info_pub = self.create_publisher(CameraInfo, 'stereo/left/camera_info', 10)
        self.right_info_pub = self.create_publisher(CameraInfo, 'stereo/right/camera_info', 10)

        self.get_logger().info(f"Настройки камеры: {self.WIDTH}x{self.HEIGHT} @ {self.FPS} FPS")

        # Инициализация камеры
        if not self.init_camera_mjpg():
            self.get_logger().error("Не удалось инициализировать камеру. Завершение работы.")
            rclpy.shutdown()
            return

        self.timer = self.create_timer(1.0 / self.FPS, self.timer_callback)
        self.get_logger().info("Стерео-нода успешно запущена ✅")

    # ---------------------- SET CAMERA INFO ----------------------

    def handle_set_camera_info_left(self, request, response):
        """Обработчик сервиса установки калибровки для левой камеры"""
        if self.left_info_mgr.setCameraInfo(request.camera_info):
            response.success = True
            response.status_message = f"Left camera info saved to {self.left_info_mgr.getURL()}"
            self.get_logger().info(response.status_message)
        else:
            response.success = False
            response.status_message = "Failed to update left camera info."
            self.get_logger().error(response.status_message)
        return response

    def handle_set_camera_info_right(self, request, response):
        """Обработчик сервиса установки калибровки для правой камеры"""
        if self.right_info_mgr.setCameraInfo(request.camera_info):
            response.success = True
            response.status_message = f"Right camera info saved to {self.right_info_mgr.getURL()}"
            self.get_logger().info(response.status_message)
        else:
            response.success = False
            response.status_message = "Failed to update right camera info."
            self.get_logger().error(response.status_message)
        return response

    # ---------------------- CAMERA INIT ----------------------

    def init_camera_mjpg(self):
        """Инициализация камеры с MJPG"""
        self.cap = cv2.VideoCapture(self.camera_device, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            self.get_logger().error(f"Не удалось открыть {self.camera_device}")
            return False

        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.HEIGHT)
        self.cap.set(cv2.CAP_PROP_FPS, self.FPS)

        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("Не удалось получить тестовый кадр")
            return False

        self.get_logger().info(f"Камера готова: {frame.shape[1]}x{frame.shape[0]}")
        return True

    # ---------------------- TIMER CALLBACK ----------------------

    def timer_callback(self):
        if not self.cap or not self.cap.isOpened():
            return

        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("Не удалось получить кадр с камеры")
            return

        # 🔥 ЕДИНЫЙ timestamp для всего кадра
        stamp = self.get_clock().now().to_msg()

        height, width = frame.shape[:2]
        half_width = width // 2

        left_image = frame[:, :half_width, :]
        right_image = frame[:, half_width:, :]

        # Публикуем ОБЕ камеры (RViz2 сам отбросит лишнее)
        self.publish_image(left_image, self.left_image_pub, self.left_info_pub,
                        self.left_info_mgr, "left", stamp)
        
        self.publish_image(right_image, self.right_image_pub, self.right_info_pub,
                        self.right_info_mgr, "right", stamp)

    # ---------------------- IMAGE PUBLISH ----------------------

    def publish_image(self, cv_image, img_pub, info_pub, info_mgr, frame_name, stamp):
        # Публикуем CameraInfo
        info_msg = info_mgr.getCameraInfo()
        info_msg.header.stamp = stamp
        info_msg.header.frame_id = f"{frame_name}_camera_frame"
        info_pub.publish(info_msg)

        # Публикуем изображение (сжатое или сырое)
        if self.use_compression:
            # Сжатие JPEG
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]
            _, compressed_data = cv2.imencode('.jpg', cv_image, encode_param)
            
            compressed_msg = CompressedImage()
            compressed_msg.header.stamp = stamp
            compressed_msg.header.frame_id = f"{frame_name}_camera_frame"
            compressed_msg.format = "jpeg"
            compressed_msg.data = compressed_data.tobytes()
            
            img_pub.publish(compressed_msg)
        else:
            # Сырое изображение
            ros_image = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            ros_image.header.stamp = stamp
            ros_image.header.frame_id = f"{frame_name}_camera_frame"
            img_pub.publish(ros_image)


# ---------------------- MAIN ----------------------

def main(args=None):
    rclpy.init(args=args)
    node = StereoCameraNode()
    
    # Проверяем что нода создана успешно
    if node.cap is None:
        # Камера не инициализировалась, нода уже вызвала rclpy.shutdown()
        return
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Завершение работы по запросу пользователя")
    finally:
        if node.cap:
            node.cap.release()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
