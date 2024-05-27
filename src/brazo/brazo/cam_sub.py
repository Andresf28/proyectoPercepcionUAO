import rclpy  # Python library for ROS 2
from rclpy.node import Node  # Handles the creation of nodes
from sensor_msgs.msg import Image  # Image is the message type
from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images
import cv2  # OpenCV library
import numpy as np  # Library for numerical operations

class ImageSubscriber(Node):
    """
    Create an ImageSubscriber class, which is a subclass of the Node class.
    """
    def __init__(self):
        """
        Class constructor to set up the node
        """
        # Initiate the Node class's constructor and give it a name
        super().__init__('image_subscriber')

        # Create the subscriber. This subscriber will receive an Image
        # from the video_frames topic. The queue size is 10 messages.
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.listener_callback, 30)
        self.subscription  # prevent unused variable warning

        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()

    def listener_callback(self, data):
        """
        Callback function.
        """
        # Display the message on the console
        self.get_logger().info('Receiving video frame')

        # Convert ROS Image message to OpenCV image
        current_frame = self.br.imgmsg_to_cv2(data, desired_encoding='bgr8')

        # Define the region of interest (ROI)
        height, width = current_frame.shape[:2]
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
        masked_image = cv2.bitwise_and(current_frame, current_frame, mask=mask)

        # Convertir la imagen enmascarada a RGB (opcional)
        masked_image_rgb = cv2.cvtColor(masked_image, cv2.COLOR_BGR2RGB)

        # Display image
        cv2.imshow("camera", masked_image)
        cv2.waitKey(1)

def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create the node
    image_subscriber = ImageSubscriber()

    # Spin the node so the callback function is called.
    rclpy.spin(image_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    image_subscriber.destroy_node()

    # Shutdown the ROS client library for Python
    rclpy.shutdown()

if __name__ == '__main__':
    main()
