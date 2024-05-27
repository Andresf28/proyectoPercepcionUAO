import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

class YoloDetector(Node):
    def __init__(self):
        super().__init__('yolo_detector')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.listener_callback,
            10)
        self.publisher_ = self.create_publisher(String, 'detected_object', 10)
        self.bridge = CvBridge()
        self.model = YOLO('/home/andres/percepcion_ws/src/brazo/pesos.pt')

    def listener_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg)
        results = self.model(cv_image)
        
        detected_classes = set()
        for result in results:
            for detection in result:
                class_id = detection[-1]
                if class_id == 0:
                    detected_classes.add('circulo')
                elif class_id == 1:
                    detected_classes.add('triangulo')
                elif class_id == 2:
                    detected_classes.add('estrella')
                elif class_id == 3:
                    detected_classes.add('luna')

        for detected_class in detected_classes:
            self.publisher_.publish(String(data=detected_class))

def main(args=None):
    rclpy.init(args=args)
    yolo_detector = YoloDetector()
    rclpy.spin(yolo_detector)
    yolo_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
