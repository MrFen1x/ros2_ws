import rclpy
from std_msgs.msg import String
import json
import random

class Lab2Plugin:
    def init(self, webots_node, properties):
        self.robot = webots_node.robot
        self.node = webots_node
        
        self.metrics_pub = self.node.create_publisher(String, '/lab2/metrics', 10)
        
        # Determine scenario from properties
        self.scenario = properties.get('scenario', 'A')
        self.node.get_logger().info(f"Starting Lab 2 Plugin - Scenario: {self.scenario}")
        
        self.state = 'PICK'
        self.step_start_time = self.robot.getTime()
        self.objects_moved = 0
        self.conveyor_1_count = 0
        self.conveyor_2_count = 0
        self.telega_count = 0
        self.errors = 0
        self.cycle_time_total = 0.0
        
        self.box_node = self.robot.getFromDef("BOX1")
        self.target_conveyor = 1

    def step(self):
        current_time = self.robot.getTime()
        dt = current_time - self.step_start_time
        
        if self.state == 'PICK':
            if dt > 1.5:
                self.state = 'MOVE'
                self.step_start_time = current_time
                if random.random() < 0.05:  # 5% error
                    self.errors += 1
                    self.state = 'RETURN'
                    
        elif self.state == 'MOVE':
            if dt > 2.0:
                self.state = 'DROP'
                self.step_start_time = current_time
                
                if self.scenario == 'A':
                    self.target_conveyor = 1
                elif self.scenario == 'B':
                    self.target_conveyor = 1 if (self.objects_moved % 2 == 0) else 2
                elif self.scenario == 'C':
                    mod = self.objects_moved % 3
                    self.target_conveyor = mod + 1

        elif self.state == 'DROP':
            if dt > 0.5:
                self.objects_moved += 1
                if self.target_conveyor == 1:
                    self.conveyor_1_count += 1
                elif self.target_conveyor == 2:
                    self.conveyor_2_count += 1
                elif self.target_conveyor == 3:
                    self.telega_count += 1
                
                self.cycle_time_total += (1.5 + 2.0 + 0.5 + 1.0) # total ~ 5.0
                
                msg = String()
                metrics = {
                    'scenario': self.scenario,
                    'time': current_time,
                    'objects_moved': self.objects_moved,
                    'conveyor_1': self.conveyor_1_count,
                    'conveyor_2': self.conveyor_2_count,
                    'telega': self.telega_count,
                    'errors': self.errors,
                    'avg_cycle_time': self.cycle_time_total / max(1, self.objects_moved)
                }
                msg.data = json.dumps(metrics)
                self.metrics_pub.publish(msg)
                
                self.state = 'RETURN'
                self.step_start_time = current_time
                
        elif self.state == 'RETURN':
            if dt > 1.0:
                self.state = 'PICK'
                self.step_start_time = current_time
                
                if self.objects_moved >= 30: # simulate 30 objects
                    self.node.get_logger().info("Scenario complete!")
