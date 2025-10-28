#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from camera_info_manager import CameraInfoManager
import cv2

class StereoCameraNode(Node):
    def __init__(self):
        super().__init__('stereo_camera_node')

        # Параметры камеры
        self.camera_device = '/dev/video2'
        self.WIDTH = 2560
        self.HEIGHT = 720
        self.FPS = 30

        self.get_logger().info(f"Открываем камеру: {self.camera_device}")

        # --- CameraInfoManager для левой и правой камер ---
        self.left_info_mgr = CameraInfoManager(self, 'left', '')
        self.right_info_mgr = CameraInfoManager(self, 'right', '')

        # --- Публикаторы изображений и CameraInfo ---
        self.left_image_pub = self.create_publisher(Image, 'stereo/left/image_raw', 10)
        self.right_image_pub = self.create_publisher(Image, 'stereo/right/image_raw', 10)
        self.left_info_pub = self.create_publisher(CameraInfo, 'stereo/left/camera_info', 10)
        self.right_info_pub = self.create_publisher(CameraInfo, 'stereo/right/camera_info', 10)

        self.bridge = CvBridge()
        self.cap = None

        if self.init_camera_mjpg():
            self.timer = self.create_timer(1.0 / self.FPS, self.timer_callback)
            self.get_logger().info("Стерео камера успешно инициализирована")
        else:
            self.get_logger().error("Не удалось инициализировать камеру")

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

    def timer_callback(self):
        if not self.cap or not self.cap.isOpened():
            return

        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("Не удалось получить кадр с камеры")
            return

        height, width = frame.shape[:2]
        half_width = width // 2
        left_image = frame[:, :half_width, :]
        right_image = frame[:, half_width:, :]

        self.publish_image(left_image, self.left_image_pub, self.left_info_pub, self.left_info_mgr, "left")
        self.publish_image(right_image, self.right_image_pub, self.right_info_pub, self.right_info_mgr, "right")

    def publish_image(self, cv_image, img_pub, info_pub, info_mgr, frame_name):
        ros_image = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
        ros_image.header.stamp = self.get_clock().now().to_msg()
        ros_image.header.frame_id = f"{frame_name}_camera_frame"

        info_msg = info_mgr.getCameraInfo()
        info_msg.header = ros_image.header
        info_msg.width = cv_image.shape[1]
        info_msg.height = cv_image.shape[0]

        img_pub.publish(ros_image)
        info_pub.publish(info_msg)

def main(args=None):
    rclpy.init(args=args)
    node = StereoCameraNode()
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
