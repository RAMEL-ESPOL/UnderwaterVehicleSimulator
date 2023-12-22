import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class BallDetector(Node):
    def __init__(self):
        super().__init__('ball_detector')
        self.subscription = self.create_subscription(
            Image,
            '/model/auv_max/camera',
            self.image_callback,
            10)
        self.bridge = CvBridge()
        self.publisher = self.create_publisher(Image, '/detected_balls', 10)

    def image_callback(self, msg):
        # Convertir mensaje ROS Image a imagen para OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # Convertir a escala de grises
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (15, 15), 0)

        # Usar HoughCircles para detectar circulos
        circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, 50, param1=100, param2=30, minRadius=10, maxRadius=100)

        if circles is not None:
            circles = np.uint16(np.around(circles))
            for i in circles[0, :]:
                # Dibujar un circulo exterior en la imagen
                cv2.circle(frame, (i[0], i[1]), i[2], (0, 255, 0), 2)
                # Dibujar un circulo en el centro del circulo exterior
                cv2.circle(frame, (i[0], i[1]), 2, (0, 0, 255), 3)

            self.get_logger().info('Bolas {} detectadas'.format(len(circles[0, :])))

        # Convertir la imagen modificada a un mensaje ROS Image y publicarlo
        image_message = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.publisher.publish(image_message)

def main(args=None):
    rclpy.init(args=args)
    ball_detector = BallDetector()
    rclpy.spin(ball_detector)
    ball_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
