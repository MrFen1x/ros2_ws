import rclpy
from std_msgs.msg import String
import json
import random

class Lab2Plugin:
    def get_logger(self):
        if hasattr(self, 'node') and self.node is not None and hasattr(self.node, 'get_logger'):
            try:
                return self.node.get_logger()
            except Exception:
                pass
        import rclpy.logging
        return rclpy.logging.get_logger("Lab2Plugin")

    def init(self, webots_node, properties):
        self.init_ok = False

        try:
            self.robot = webots_node.robot
            
            # Разбор аргументов из /proc/self/cmdline для получения реальных аргументов ROS2
            import sys
            import os
            ros_args = []
            if os.path.exists('/proc/self/cmdline'):
                try:
                    with open('/proc/self/cmdline', 'rb') as f:
                        cmdline_content = f.read()
                    raw_args = [arg.decode('utf-8') for arg in cmdline_content.split(b'\x00') if arg]
                    if '--ros-args' in raw_args:
                        idx = raw_args.index('--ros-args')
                        ros_args = raw_args[idx:]
                except Exception as e:
                    print(f"Error parsing /proc/self/cmdline: {e}", flush=True)
            
            # Инициализируем rclpy и создаем собственный узел, так как WebotsNode не предоставляет интерфейс ROS2 в Python
            if not rclpy.ok():
                rclpy.init(args=ros_args if ros_args else None)
            self.node = rclpy.create_node('lab2_plugin_supervisor_node')
            
            # Инициализация издателя метрик
            self.metrics_pub = self.node.create_publisher(String, '/lab2/metrics', 10)
            
            # Извлекаем параметр "scenario" с приоритетом:
            # 1. Параметры ROS2 узла (будут автоматически заполнены благодаря ros_args)
            # 2. Свойства (properties)
            # 3. Значение по умолчанию 'A'
            scenario_val = 'A'
            
            # Проверяем свойства плагина
            if properties and 'scenario' in properties:
                scenario_val = properties['scenario']
                
            # Проверяем / декларируем параметр ROS2
            try:
                self.node.declare_parameter('scenario', scenario_val)
                scenario_val = self.node.get_parameter('scenario').value
            except Exception as param_err:
                self.get_logger().warn(f"Failed to declare/get ROS2 parameter 'scenario': {param_err}")
                
            self.scenario = scenario_val
            self.get_logger().info(f"Starting Lab 2 Plugin - Scenario: {self.scenario}")
            
            # Инициализация моторов UR5e
            self.pose_pick_up = [3.14, -1.0, 1.5, -1.0, 1.57, 0.0]
            self.pose_pick_down = [3.14, -1.4, 2.0, -1.2, 1.57, 0.0]
            
            self.pose_drop_c1 = [0.0, -1.0, 1.5, -1.0, 1.57, 0.0]
            self.pose_drop_c2 = [-1.57, -1.0, 1.5, -1.0, 1.57, 0.0]
            self.pose_drop_telega = [-1.57, -0.7, 1.2, -1.0, 1.57, 0.0]
            
            self.motor_names = [
                'shoulder_pan_joint',
                'shoulder_lift_joint',
                'elbow_joint',
                'wrist_1_joint',
                'wrist_2_joint',
                'wrist_3_joint'
            ]
            self.motors = []
            for name in self.motor_names:
                motor = self.robot.getDevice(name)
                if motor:
                    self.get_logger().info(f"Found motor: {name}")
                    try:
                        motor.setVelocity(motor.getMaxVelocity())
                        motor.setAvailableTorque(motor.getMaxTorque())
                    except Exception as limit_err:
                        self.get_logger().warn(f"Could not set motor limits for {name}: {limit_err}")
                    motor.setPosition(0.0)
                    self.motors.append(motor)
                else:
                    self.get_logger().warn(f"Motor {name} not found!")
            self.get_logger().info(f"Initialized {len(self.motors)} motors out of {len(self.motor_names)}")
                    
            # Установка начальной позы
            for motor, angle in zip(self.motors, self.pose_pick_up):
                motor.setPosition(angle)
            
            self.state = 'PICK'
            self.step_start_time = self.robot.getTime()
            self.cycle_start_time = self.robot.getTime()
            
            self.objects_moved = 0
            self.conveyor_1_count = 0
            self.conveyor_2_count = 0
            self.telega_count = 0
            
            self.pickup_misses = 0
            self.drop_losses = 0
            self.total_errors = 0
            
            self.cycle_id = 0
            self.energy_proxy = 0.0
            self.last_cycle_time = 0.0
            self.cycle_time_sum = 0.0
            
            # Безопасное получение ноды BOX1
            self.box_node = None
            self.box_translation_field = None
            try:
                if hasattr(self.robot, 'getFromDef'):
                    self.box_node = self.robot.getFromDef("BOX1")
                    if self.box_node:
                        self.box_translation_field = self.box_node.getField("translation")
                        if self.box_translation_field:
                            self.box_translation_field.setSFVec3f([-0.5, 0.0, 0.05])
                        self.box_node.resetPhysics()
            except Exception as e:
                self.get_logger().warn(f"Could not get BOX1 node: {e}")
            
            # Определение начальной цели и длительности
            self.set_next_destination()
            self.calculate_durations()
            
            self.init_ok = True
            
        except Exception as e:
            import traceback
            error_str = f"CRITICAL ERROR in Lab2Plugin init: {e}\n{traceback.format_exc()}"
            print(error_str, flush=True)
            try:
                with open('/home/ilya/ros2_ws/init_error.log', 'w') as f:
                    f.write(error_str)
            except Exception as write_err:
                print(f"Failed to write to init_error.log: {write_err}", flush=True)
            
            try:
                self.get_logger().error(error_str)
            except Exception:
                pass
            raise e

    def set_next_destination(self):
        if self.scenario == 'A':
            self.target_conveyor = 1
        elif self.scenario == 'B':
            self.target_conveyor = 1 if (self.objects_moved % 2 == 0) else 2
        elif self.scenario == 'C':
            mod = self.objects_moved % 3
            self.target_conveyor = mod + 1
            
    def calculate_durations(self):
        # Base durations reflecting physical distance
        # Conveyor 1 (closest), Conveyor 2 (medium), Telega (furthest)
        self.t_pick = 1.2 + random.uniform(-0.1, 0.1)
        self.t_drop = 0.5 + random.uniform(-0.05, 0.05)
        
        if self.target_conveyor == 1:
            self.t_move = 1.5 + random.uniform(-0.15, 0.15)
            self.t_return = 1.0 + random.uniform(-0.1, 0.1)
            self.move_energy = 1.5
        elif self.target_conveyor == 2:
            self.t_move = 2.2 + random.uniform(-0.2, 0.2)
            self.t_return = 1.5 + random.uniform(-0.15, 0.15)
            self.move_energy = 2.5
        else: # Telega
            self.t_move = 3.0 + random.uniform(-0.3, 0.3)
            self.t_return = 2.0 + random.uniform(-0.2, 0.2)
            self.move_energy = 3.5

    def publish_metrics(self, current_time, status, error_type="none"):
        total_attempts = self.objects_moved + self.total_errors
        success_rate = (self.objects_moved / max(1, total_attempts)) * 100.0
        
        # Throughput: parts per minute based on total time elapsed
        throughput_ppm = (self.objects_moved / max(0.1, current_time)) * 60.0
        avg_cycle_time = self.cycle_time_sum / max(1, self.objects_moved)
        
        metrics = {
            'scenario': self.scenario,
            'time': current_time,
            'cycle_id': self.cycle_id,
            'destination': f"conveyor_{self.target_conveyor}" if self.target_conveyor < 3 else "telega",
            'cycle_duration': self.last_cycle_time,
            'status': status,
            'error_type': error_type,
            'objects_moved': self.objects_moved,
            'conveyor_1': self.conveyor_1_count,
            'conveyor_2': self.conveyor_2_count,
            'telega': self.telega_count,
            'errors': self.total_errors,
            'pickup_misses': self.pickup_misses,
            'drop_losses': self.drop_losses,
            'success_rate': success_rate,
            'throughput_ppm': throughput_ppm,
            'avg_cycle_time': avg_cycle_time,
            'energy_proxy': self.energy_proxy
        }
        
        msg = String()
        msg.data = json.dumps(metrics)
        self.metrics_pub.publish(msg)

    def interpolate_joints(self, start_pose, end_pose, ratio):
        ratio = max(0.0, min(1.0, ratio))
        t = ratio * ratio * (3.0 - 2.0 * ratio)
        return [s + (e - s) * t for s, e in zip(start_pose, end_pose)]

    def step(self):
        if not getattr(self, 'init_ok', False):
            return
        # Спин для выполнения коллбеков
        rclpy.spin_once(self.node, timeout_sec=0)
        
        current_time = self.robot.getTime()
        dt = current_time - self.step_start_time
        
        if not hasattr(self, 'is_pickup_miss'):
            self.is_pickup_miss = random.random() < 0.03
        if not hasattr(self, 'is_drop_loss'):
            self.is_drop_loss = random.random() < 0.02
            
        # Получаем целевую позу сброса и координаты
        if self.target_conveyor == 1:
            pose_drop = self.pose_drop_c1
            drop_pos = [1.2, 0.0, 0.3]
        elif self.target_conveyor == 2:
            pose_drop = self.pose_drop_c2
            drop_pos = [0.0, -1.2, 0.3]
        else: # Telega
            pose_drop = self.pose_drop_telega
            drop_pos = [0.0, -1.6, 0.3]
            
        pose = self.pose_pick_up
        
        if self.state == 'PICK':
            if dt < self.t_pick / 2:
                ratio = dt / (self.t_pick / 2)
                pose = self.interpolate_joints(self.pose_pick_up, self.pose_pick_down, ratio)
            else:
                ratio = (dt - self.t_pick / 2) / (self.t_pick / 2)
                pose = self.interpolate_joints(self.pose_pick_down, self.pose_pick_up, ratio)
                
            # Управление коробкой в фазе PICK (подъем)
            if self.box_translation_field:
                if dt < 0.05:
                    try:
                        self.box_translation_field.setSFVec3f([-0.5, 0.0, 0.05])
                        self.box_node.resetPhysics()
                    except Exception:
                        pass
                elif dt >= self.t_pick / 2 and not self.is_pickup_miss:
                    try:
                        ratio = (dt - self.t_pick / 2) / (self.t_pick / 2)
                        ratio = max(0.0, min(1.0, ratio))
                        self.box_translation_field.setSFVec3f([-0.5, 0.0, 0.05 + 0.10 * ratio])
                    except Exception:
                        pass
                        
            if dt >= self.t_pick:
                if self.is_pickup_miss:
                    self.pickup_misses += 1
                    self.total_errors += 1
                    self.energy_proxy += 0.5 + 1.0  # pick and return energy
                    self.last_cycle_time = (current_time - self.cycle_start_time)
                    self.cycle_id += 1
                    self.publish_metrics(current_time, 'FAILED_PICK', 'pickup_miss')
                    
                    self.state = 'RETURN'
                    self.step_start_time = current_time
                    self.calculate_durations()
                    self.is_pickup_miss = random.random() < 0.03
                    self.is_drop_loss = random.random() < 0.02
                else:
                    self.state = 'MOVE'
                    self.step_start_time = current_time
                    self.is_drop_loss = random.random() < 0.02
                    
        elif self.state == 'MOVE':
            ratio = dt / self.t_move
            pose = self.interpolate_joints(self.pose_pick_up, pose_drop, ratio)
            
            # Управление коробкой в фазе MOVE
            if self.box_translation_field:
                try:
                    ratio_clamped = max(0.0, min(1.0, ratio))
                    if self.is_drop_loss and ratio_clamped >= 0.5:
                        # Прекращаем двигать, пусть падает
                        pass
                    else:
                        x = -0.5 + (drop_pos[0] - (-0.5)) * ratio_clamped
                        y = 0.0 + (drop_pos[1] - 0.0) * ratio_clamped
                        # Параболическая траектория
                        z = 0.15 + (drop_pos[2] - 0.15) * ratio_clamped + 0.30 * (1.0 - (2.0 * ratio_clamped - 1.0) ** 2)
                        self.box_translation_field.setSFVec3f([x, y, z])
                except Exception:
                    pass
            
            if dt >= self.t_move:
                if self.is_drop_loss:
                    self.drop_losses += 1
                    self.total_errors += 1
                    self.energy_proxy += self.move_energy * 0.5 + self.move_energy
                    self.last_cycle_time = (current_time - self.cycle_start_time)
                    self.cycle_id += 1
                    self.publish_metrics(current_time, 'FAILED_MOVE', 'drop_loss')
                    
                    self.state = 'RETURN'
                    self.step_start_time = current_time
                    self.calculate_durations()
                    self.is_pickup_miss = random.random() < 0.03
                    self.is_drop_loss = random.random() < 0.02
                else:
                    self.state = 'DROP'
                    self.step_start_time = current_time
                    
        elif self.state == 'DROP':
            pose = pose_drop
            
            if dt >= self.t_drop:
                self.objects_moved += 1
                if self.target_conveyor == 1:
                    self.conveyor_1_count += 1
                elif self.target_conveyor == 2:
                    self.conveyor_2_count += 1
                else:
                    self.telega_count += 1
                
                self.energy_proxy += self.move_energy
                self.last_cycle_time = (current_time - self.cycle_start_time)
                self.cycle_time_sum += self.last_cycle_time
                self.cycle_id += 1
                
                self.publish_metrics(current_time, 'SUCCESS')
                
                self.state = 'RETURN'
                self.step_start_time = current_time
                
        elif self.state == 'RETURN':
            ratio = dt / self.t_return
            pose = self.interpolate_joints(pose_drop, self.pose_pick_up, ratio)
            
            if dt >= self.t_return:
                self.state = 'PICK'
                self.step_start_time = current_time
                self.cycle_start_time = current_time
                
                self.set_next_destination()
                self.calculate_durations()
                self.is_pickup_miss = random.random() < 0.03
                self.is_drop_loss = random.random() < 0.02
                
                if self.objects_moved >= 30:
                    self.get_logger().info(f"Scenario {self.scenario} complete!")
                    
        # Периодический лог для отладки
        if not hasattr(self, 'step_count'):
            self.step_count = 0
        self.step_count += 1
        if self.step_count % 100 == 0:
            self.get_logger().info(f"Step {self.step_count}: State={self.state}, Motors={len(self.motors)}, Pose={[round(a, 2) for a in pose]}")

        # Применяем вычисленную позу к моторам
        for motor, angle in zip(self.motors, pose):
            try:
                motor.setPosition(angle)
            except Exception as e:
                self.get_logger().error(f"Error setting motor position: {e}")
