import rclpy
from rclpy.node import Node
import argparse
import subprocess

class ServiceCallerNode(Node):
    def __init__(self):
        super().__init__('service_caller')

    def call_service(self, power):
        command = f"ros2 service call /CONVEYORPOWER conveyorbelt_msgs/srv/ConveyorBeltControl \"{{power: {power}}}\""
        subprocess.run(command, shell=True)
        self.get_logger().info("Service call executed successfully.")

def main():
    parser = argparse.ArgumentParser(description='ROS 2 Service Caller Node')
    parser.add_argument('power', type=int, help='Power value to send to the service')

    args = parser.parse_args()

    rclpy.init()
    node = ServiceCallerNode()
    node.call_service(args.power)
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
