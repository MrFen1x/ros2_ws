#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import RPi.GPIO as GPIO


class MotorDriverNode(Node):
    def __init__(self):
        super().__init__('motor_driver')

        # Расстояние между колесами (м)
        self.wheel_distance = 0.20

        # Настройка GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        # Пины левого мотора (M1)
        self.IN1 = 17
        self.IN2 = 27
        self.EN_LEFT = 13

        # Пины правого мотора (M2)
        self.IN3 = 23
        self.IN4 = 24
        self.EN_RIGHT = 12

        GPIO.setup(self.IN1, GPIO.OUT)
        GPIO.setup(self.IN2, GPIO.OUT)
        GPIO.setup(self.EN_LEFT, GPIO.OUT)

        GPIO.setup(self.IN3, GPIO.OUT)
        GPIO.setup(self.IN4, GPIO.OUT)
        GPIO.setup(self.EN_RIGHT, GPIO.OUT)

        # PWM на 1 кГц
        self.pwm_left = GPIO.PWM(self.EN_LEFT, 1000)
        self.pwm_right = GPIO.PWM(self.EN_RIGHT, 1000)

        self.pwm_left.start(0)
        self.pwm_right.start(0)

        # Подписка на /cmd_vel
        self.subscription = self.create_subscription(
            Twist,
            "/cmd_vel",
            self.cmd_callback,
            10
        )

        self.get_logger().info("Motor L298N driver started!")

    def set_motor(self, in1, in2, pwm_controller, speed):
        """
        speed: от -100 до 100 (процент PWM)
        """

        direction = GPIO.HIGH if speed > 0 else GPIO.LOW
        reverse = GPIO.LOW if speed > 0 else GPIO.HIGH

        # Направление
        GPIO.output(in1, direction)
        GPIO.output(in2, reverse)

        # Мощность PWM
        pwm_controller.ChangeDutyCycle(abs(speed))

    def cmd_callback(self, twist: Twist):
        linear = twist.linear.x      # m/s
        angular = twist.angular.z    # rad/s

        # Дифференциальная кинематика
        left_speed = linear - angular * self.wheel_distance / 2
        right_speed = linear + angular * self.wheel_distance / 2

        # Перевод скорости в PWM %
        left_pwm = max(min(left_speed * 100, 100), -100)
        right_pwm = max(min(right_speed * 100, 100), -100)

        self.set_motor(self.IN1, self.IN2, self.pwm_left, left_pwm)
        self.set_motor(self.IN3, self.IN4, self.pwm_right, right_pwm)

    def destroy_node(self):
        GPIO.cleanup()
        super().destroy_node()


def main(args=None):
        rclpy.init(args=args)
        node = MotorDriverNode()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
        main()
