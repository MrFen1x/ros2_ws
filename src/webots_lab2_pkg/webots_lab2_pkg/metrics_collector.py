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
        self.writer.writerow([
            'Time', 'Cycle_ID', 'Destination', 'Duration', 'Status', 'Error_Type',
            'Objects_Moved', 'Conveyor_1', 'Conveyor_2', 'Telega', 'Errors',
            'Pickup_Misses', 'Drop_Losses', 'Success_Rate', 'Throughput_Ppm',
            'Avg_Cycle_Time', 'Energy_Proxy'
        ])
        
        self.start_time = None

    def metrics_callback(self, msg):
        data = json.loads(msg.data)
        
        self.get_logger().info(
            f"Metrics: Moved={data['objects_moved']}, C1={data['conveyor_1']}, "
            f"C2={data['conveyor_2']}, Telega={data['telega']}, Err={data['errors']}, "
            f"SR={data['success_rate']:.1f}%, PPM={data['throughput_ppm']:.1f}"
        )
        
        self.writer.writerow([
            data['time'],
            data['cycle_id'],
            data['destination'],
            data['cycle_duration'],
            data['status'],
            data['error_type'],
            data['objects_moved'],
            data['conveyor_1'],
            data['conveyor_2'],
            data['telega'],
            data['errors'],
            data['pickup_misses'],
            data['drop_losses'],
            data['success_rate'],
            data['throughput_ppm'],
            data['avg_cycle_time'],
            data['energy_proxy']
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
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
