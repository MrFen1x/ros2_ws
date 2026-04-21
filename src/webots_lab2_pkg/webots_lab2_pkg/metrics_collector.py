import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import csv
import os

class MetricsCollector(Node):
    def __init__(self):
        super().__init__('metrics_collector')
        
        self.declare_parameter('scenario', 'A')
        self.scenario = self.get_parameter('scenario').get_parameter_value().string_value
        
        self.subscription = self.create_subscription(
            String,
            '/lab2/metrics',
            self.metrics_callback,
            10
        )
        self.get_logger().info(f"Metrics Collector started for Scenario: {self.scenario}")
        
        # Open CSV file for writing
        self.csv_file = f'scenario_{self.scenario}_metrics.csv'
        self.file = open(self.csv_file, mode='w', newline='')
        self.writer = csv.writer(self.file)
        self.writer.writerow(['Time', 'Objects_Moved', 'Conveyor_1', 'Conveyor_2', 'Telega', 'Errors', 'Avg_Cycle_Time'])
        
        self.start_time = None

    def metrics_callback(self, msg):
        data = json.loads(msg.data)
        
        # Print periodically
        self.get_logger().info(f"Metrics: Moved={data['objects_moved']}, C1={data['conveyor_1']}, C2={data['conveyor_2']}, Telega={data['telega']}, Err={data['errors']}")
        
        self.writer.writerow([
            data['time'],
            data['objects_moved'],
            data['conveyor_1'],
            data['conveyor_2'],
            data['telega'],
            data['errors'],
            data['avg_cycle_time']
        ])
        self.file.flush()

    def destroy_node(self):
        self.file.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MetricsCollector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
