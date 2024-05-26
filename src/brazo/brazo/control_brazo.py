import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

def main():
    rclpy.init()

    node = rclpy.create_node('pose_publisher')

    # Crear un publicador para el tópico "target_pose"
    publisher = node.create_publisher(PoseStamped, 'target_pose_topic', 10)

    # Crear una pose objetivo
    pose_msg = PoseStamped()
    pose_msg.header.frame_id = "base_link"  # Frame de referencia de la pose
    pose_msg.pose.position.x = 1.0  # Coordenada x
    pose_msg.pose.position.y = 1.0  # Coordenada y
    pose_msg.pose.position.z = 1.0  # Coordenada z
    pose_msg.pose.orientation.w = 0.7071 # Cuaternión de orientación (en este caso, sin rotación)
    pose_msg.pose.orientation.y = 0.7071
    # Publicar la pose objetivo
    node.get_logger().info('Publicando pose objetivo en el tópico "target_pose"')
    publisher.publish(pose_msg)

    # Pausa para permitir que el mensaje sea publicado
    time.sleep(1)  # Esperar 1 segundo

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
