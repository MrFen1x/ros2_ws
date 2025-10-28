#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class StereoCameraNode(Node):
    def __init__(self):
        super().__init__('stereo_camera_node')
        

        
        self.camera_device ='/dev/video2'
        self.WIDTH = 2560
        self.HEIGHT = 720
        self.FPS = 30
        
        self.get_logger().info(f"Открываем камеру: {self.camera_device}")
        
        # Публикаторы
        self.left_publisher_ = self.create_publisher(Image, 'stereo/left/image_raw', 10)
        self.right_publisher_ = self.create_publisher(Image, 'stereo/right/image_raw', 10)
        
        self.bridge = CvBridge()
        self.cap = None
        
        # Инициализация камеры
        if self.init_camera_mjpg():
            self.timer = self.create_timer(1.0/self.FPS, self.timer_callback)
            self.get_logger().info("Стерео камера успешно инициализирована")
        else:
            self.get_logger().error("Не удалось инициализировать камеру")

    def init_camera_mjpg(self):
        """Инициализация камеры с MJPG форматом"""
        try:
            # Открываем камеру через V4L2 с явным указанием устройства
            self.cap = cv2.VideoCapture(self.camera_device, cv2.CAP_V4L2)
            
            if not self.cap.isOpened():
                self.get_logger().error(f"Не удалось открыть {self.camera_device}")
                return False
            
            # Устанавливаем формат MJPG и разрешение
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.WIDTH)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.HEIGHT)
            self.cap.set(cv2.CAP_PROP_FPS, self.FPS)
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 2)
            
            # Пробуем получить тестовый кадр
            for i in range(5):  # Несколько попыток
                ret, frame = self.cap.read()
                if ret and frame is not None:
                    self.get_logger().info(f"Получен тестовый кадр: {frame.shape[1]}x{frame.shape[0]}")
                    return True
                else:
                    self.get_logger().warn(f"Попытка {i+1}: не удалось получить кадр")
                    cv2.waitKey(100)
            
            return False
            
        except Exception as e:
            self.get_logger().error(f"Ошибка инициализации камеры: {e}")
            return False

    def timer_callback(self):
        """Обработка и публикация кадров"""
        if not self.cap or not self.cap.isOpened():
            return
            
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("Не удалось получить кадр с камеры")
            return

        try:
            height, width = frame.shape[:2]
            
            # Проверяем что кадр стерео (ширина должна быть в 2 раза больше высоты)
            if width != self.WIDTH or height != self.HEIGHT:
                self.get_logger().warn(f"Неожиданный размер кадра: {width}x{height}, ожидался {self.WIDTH}x{self.HEIGHT}")
                # Пробуем адаптироваться под фактический размер
                self.WIDTH = width
                self.HEIGHT = height
            
            half_width = width // 2
            
            # Разделяем стерео кадр на левое и правое изображения
            left_image = frame[:, :half_width, :]
            right_image = frame[:, half_width:, :]
            
            # Публикуем изображения
            self.publish_image(left_image, self.left_publisher_, "left")
            self.publish_image(right_image, self.right_publisher_, "right")
                     
        except Exception as e:
            self.get_logger().error(f"Ошибка обработки кадра: {e}")

    def publish_image(self, image, publisher, image_type):
        """Публикация изображения в ROS топик"""
        try:
            ros_image = self.bridge.cv2_to_imgmsg(image, "bgr8")
            ros_image.header.stamp = self.get_clock().now().to_msg()
            ros_image.header.frame_id = f"{image_type}_camera_frame"
            publisher.publish(ros_image)
        except Exception as e:
            self.get_logger().error(f"Ошибка публикации {image_type} изображения: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = StereoCameraNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Завершение работы по запросу пользователя")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()