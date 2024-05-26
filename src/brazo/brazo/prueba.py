import rclpy
from rclpy.node import Node
import subprocess

class TestNode(Node):
    def __init__(self):
        super().__init__('test_node')

    def send_goal_command(self):
        command = ['ros2', 'action', 'send_goal', '-f', '/MoveG', 'ros2_data/action/MoveG', '{goal: 0.01}']
        try:
            subprocess.run(command, check=True)
            self.get_logger().info('Comando enviado correctamente')
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f'Error al ejecutar el comando: {e}')

def main(args=None):
    rclpy.init(args=args)
    test_node = TestNode()
    test_node.send_goal_command()
    rclpy.spin(test_node)
    test_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
