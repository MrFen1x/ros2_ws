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
        ...
        # Инициализация менеджеров
        self.left_info_mgr = CameraInfoManager(self, cname='left_camera', namespace='left_camera')
        self.right_info_mgr = CameraInfoManager(self, cname='right_camera', namespace='right_camera')


        # Создаём сервисы для совместимости с camera_calibration
        self.create_service(
            CameraInfo, 'left_camera/set_camera_info', self.handle_set_camera_info_left
        )
        self.create_service(
            CameraInfo, 'right_camera/set_camera_info', self.handle_set_camera_info_right
        )
        ...
    
    def handle_set_camera_info_left(self, request, response):
        """Обработчик сервиса установки калибровки для левой камеры"""
        if self.left_info_mgr.set_camera_info(request.camera_info):
            response.success = True
        else:
            response.success = False
        return response

    def handle_set_camera_info_right(self, request, response):
        """Обработчик сервиса установки калибровки для правой камеры"""
        if self.right_info_mgr.set_camera_info(request.camera_info):
            response.success = True
        else:
            response.success = False
        return response
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

    # Если калибровка есть, берем через CameraInfoManager
        try:
            info_msg = info_mgr.getCameraInfo()
            info_msg.header = ros_image.header
        except Exception:
        # Заглушка, чтобы нода не падала
            from sensor_msgs.msg import CameraInfo
            info_msg = CameraInfo()
            info_msg.header = ros_image.header
            info_msg.width = cv_image.shape[1]
            info_msg.height = cv_image.shape[0]
            info_msg.distortion_model = "plumb_bob"

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
