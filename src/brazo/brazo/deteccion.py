import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import subprocess
import time
from std_msgs.msg import String

class ColorDetectionNode(Node):
    def __init__(self):
        super().__init__('color_detection_node')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',  # Reemplaza 'camera_topic' con el nombre correcto del topic de la cámara
            self.image_callback,
            10)
        
        self.object_subscription = self.create_subscription(
            String,
            'detected_object',
            self.object_callback,
            10)
        
        self.detected_object = None 
               
        self.bridge = CvBridge()

    def image_callback(self, msg):
        # Convertir el mensaje de ROS a una imagen OpenCV
        img_prev = self.bridge.imgmsg_to_cv2(msg)     
        cv_image = cv2.cvtColor(img_prev, cv2.COLOR_BGR2RGB)

        # Define the region of interest (ROI)
        height, width = cv_image.shape[:2]
        x_center, y_center = width // 2, height // 2
        roi_width, roi_height = 430, 300  # Tamaño del rectángulo de la ROI

        # Calcular las coordenadas de la ROI
        x1 = x_center - roi_width // 2
        y1 = y_center - roi_height // 2
        x2 = x_center + roi_width // 2
        y2 = y_center + roi_height // 2

        # Crear una máscara negra
        mask = np.zeros((height, width), dtype=np.uint8)

        # Hacer blanca la región de interés en la máscara
        mask[y1:y2, x1:x2] = 255

        # Aplicar la máscara a la imagen original
        masked_image = cv2.bitwise_and(cv_image, cv_image, mask=mask)


        # Detectar el color en la imagen
        color = self.detect_color(masked_image)


        if color is not None and self.detected_object is not None:
            self.get_logger().info(f"Color detectado: {color}")
            # Realizar acciones según el color detectado
            if color == 'rojo':
                time.sleep(5)
                if self.detected_object  == 'triangulo':
                    subprocess.call(['ros2', 'run', 'ros2_execution', 'ros2_execution.py', '--ros-args','-p', 'PROGRAM_FILENAME:=triangulo_rojo', '-p', 'ROBOT_MODEL:=irb120', '-p', 'EE_MODEL:=schunk'])
                elif self.detected_object  == 'luna':
                    subprocess.call(['ros2', 'run', 'ros2_execution', 'ros2_execution.py', '--ros-args','-p', 'PROGRAM_FILENAME:=luna_rojo', '-p', 'ROBOT_MODEL:=irb120', '-p', 'EE_MODEL:=schunk'])

            elif color == 'verde':
                time.sleep(5)
                if self.detected_object  == 'circulo':
                    subprocess.call(['ros2', 'run', 'ros2_execution', 'ros2_execution.py', '--ros-args','-p', 'PROGRAM_FILENAME:=cilindro_verde', '-p', 'ROBOT_MODEL:=irb120', '-p', 'EE_MODEL:=schunk'])
                elif self.detected_object  == 'estrella':
                    subprocess.call(['ros2', 'run', 'ros2_execution', 'ros2_execution.py', '--ros-args','-p', 'PROGRAM_FILENAME:=estrella_verde', '-p', 'ROBOT_MODEL:=irb120', '-p', 'EE_MODEL:=schunk'])
                elif self.detected_object  == 'luna':
                    subprocess.call(['ros2', 'run', 'ros2_execution', 'ros2_execution.py', '--ros-args','-p', 'PROGRAM_FILENAME:=luna_verde', '-p', 'ROBOT_MODEL:=irb120', '-p', 'EE_MODEL:=schunk'])

            elif color == 'azul':
                time.sleep(5)
                if self.detected_object  == 'estrella':
                    subprocess.call(['ros2', 'run', 'ros2_execution', 'ros2_execution.py', '--ros-args','-p', 'PROGRAM_FILENAME:=estrella_azul', '-p', 'ROBOT_MODEL:=irb120', '-p', 'EE_MODEL:=schunk'])
                elif self.detected_object  == 'circulo':
                    subprocess.call(['ros2', 'run', 'ros2_execution', 'ros2_execution.py', '--ros-args','-p', 'PROGRAM_FILENAME:=cilindro_azul', '-p', 'ROBOT_MODEL:=irb120', '-p', 'EE_MODEL:=schunk'])
                elif self.detected_object  == 'triangulo':
                    subprocess.call(['ros2', 'run', 'ros2_execution', 'ros2_execution.py', '--ros-args','-p', 'PROGRAM_FILENAME:=triangulo_azul', '-p', 'ROBOT_MODEL:=irb120', '-p', 'EE_MODEL:=schunk'])

            else:
                self.get_logger().info("no hay objeto")
                
            self.detected_object = None

            
    def object_callback(self, msg):
        self.detected_object = msg.data


    def detect_color(self, image):
        # Convertir la imagen de BGR a HSV (Hue, Saturation, Value)
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Definir los rangos de los colores en HSV
        lower_red = np.array([0, 100, 100])
        upper_red = np.array([10, 255, 255])

        lower_green = np.array([40, 100, 100])
        upper_green = np.array([80, 255, 255])

        lower_blue = np.array([100, 100, 100])
        upper_blue = np.array([140, 255, 255])

        # Filtrar los colores en el rango definido
        mask_red = cv2.inRange(hsv_image, lower_red, upper_red)
        mask_green = cv2.inRange(hsv_image, lower_green, upper_green)
        mask_blue = cv2.inRange(hsv_image, lower_blue, upper_blue)

        # Encontrar contornos de los objetos enmascarados
        contours_red, _ = cv2.findContours(mask_red, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours_green, _ = cv2.findContours(mask_green, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours_blue, _ = cv2.findContours(mask_blue, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # Determinar el color dominante
        color = None
        if len(contours_red) > 0:
            color = "rojo"
        elif len(contours_green) > 0:
            color = "verde"
        elif len(contours_blue) > 0:
            color = "azul"

        return color

def main(args=None):
    rclpy.init(args=args)
    color_detection_node = ColorDetectionNode()
    rclpy.spin(color_detection_node)
    color_detection_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
