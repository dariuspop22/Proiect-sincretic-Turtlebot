import rclpy
from rclpy.node import Node
import cv2
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class CaptureSub(Node):
    
    def __init__(self):
        super().__init__('capture')
        self.get_logger().info('Node started')

        # Subscriber pentru imagine
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.callback,
            10)
        self.subscription

        # Publisher pentru rotația robotului
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.bridge = CvBridge()
        self.rotating = True  # Variabilă pentru starea de rotație
        self.ball_detected = False
        self.cx = 0  # Coordonata centrului mingii pe axa X
        self.width = 0  # Lățimea imaginii
        self.area = 0  # Aria mingii detectate

    def callback(self, data):
        try:
            # Convertim imaginea din ROS în format OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"Error converting image: {e}")
            return

        # Convertim imaginea în format HSV pentru detecția culorii
        hsv_frame = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Intervalul culorii mingii (roșu, de exemplu)
        low_hsv = (0, 100, 100)
        high_hsv = (10, 255, 255)

        # Creăm o mască pentru detecția mingii
        mask = cv2.inRange(hsv_frame, low_hsv, high_hsv)
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # Resetăm starea de detecție
        self.ball_detected = False
        self.cx = 0
        self.area = 0

        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 200:  # Detecție doar pentru obiecte mari
                self.ball_detected = True
                x, y, w, h = cv2.boundingRect(contour)
                self.cx = x + (w // 2)  # Coordonata centrului mingii pe axa X
                self.width = cv_image.shape[1]  # Lățimea imaginii
                self.area = area  # Salvează aria pentru a evalua distanța
                cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                self.get_logger().info(f"Ball detected at X: {self.cx}, Area: {self.area}")
                break

        # Afișăm imaginea procesată
        cv2.imshow("Processed Image", cv_image)
        cv2.waitKey(3)

        # Controlăm mișcarea robotului
        self.control_robot()

    def control_robot(self):
        twist = Twist()

        if self.ball_detected:
            error = self.cx - (self.width // 2)  # Calculăm eroarea față de centrul imaginii
            self.get_logger().info(f"Centering ball, error: {error}")

            if abs(error) > 20:  # Dacă mingea nu este centrată
                twist.angular.z = -0.002 * error  # Rotește pentru a centra mingea
                twist.linear.x = 0.0
                self.get_logger().info("Adjusting position...")
            elif self.area < 90000:  # Dacă mingea este centrată, dar este departe
                twist.angular.z = 0.0
                twist.linear.x = 0.1  # Deplasează robotul înainte
                self.get_logger().info("Moving towards the ball...")
            else:  # Mingea este centrată și aproape
                twist.angular.z = 0.0
                twist.linear.x = 0.0
                self.get_logger().info("Ball reached! Stopping movement.")
                self.rotating = False
        elif self.rotating:  # Dacă mingea nu este detectată
            twist.angular.z = 0.2  # Rotește încet
            twist.linear.x = 0.0
            self.get_logger().info("Rotating, searching for ball...")

        # Publicăm comanda de mișcare
        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = CaptureSub()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

