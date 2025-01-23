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

        # Variabile pentru detecția mingii
        self.ball_detected = False
        self.cx = 0  # Coordonata centrului mingii pe axa X
        self.width = 0  # Lățimea imaginii
        self.area = 0  # Aria mingii detectate

        # Variabile pentru detectarea porții
        self.goal_detected = False
        self.goal_center_x = 0
        self.goal_area = 0

        # Stare robot
        self.state = "SEARCH_BALL"  # Starea inițială

    def callback(self, data):
        try:
            # Convertim imaginea din ROS în format OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"Error converting image: {e}")
            return

        # Convertim imaginea în format HSV pentru detecția culorilor
        hsv_frame = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Intervalul culorii mingii (roșu)
        low_hsv_ball = (0, 100, 100)
        high_hsv_ball = (10, 255, 255)

        # Mască pentru detecția mingii
        mask_ball = cv2.inRange(hsv_frame, low_hsv_ball, high_hsv_ball)
        contours_ball, _ = cv2.findContours(mask_ball, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # Resetăm starea de detecție pentru minge
        self.ball_detected = False
        self.cx = 0
        self.area = 0

        for contour in contours_ball:
            area = cv2.contourArea(contour)
            if area > 200:  # Detecție doar pentru obiecte mari
                self.ball_detected = True
                x, y, w, h = cv2.boundingRect(contour)
                self.cx = x + (w // 2)
                self.width = cv_image.shape[1]
                self.area = area
                cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                self.get_logger().info(f"Ball detected at X: {self.cx}, Area: {self.area}")
                break

        # Intervalele pentru poarta albastră și galbenă
        low_hsv_blue = (100, 150, 50)
        high_hsv_blue = (140, 255, 255)

        low_hsv_yellow = (20, 100, 100)
        high_hsv_yellow = (30, 255, 255)

        # Detectăm poarta albastră
        mask_blue = cv2.inRange(hsv_frame, low_hsv_blue, high_hsv_blue)
        contours_blue, _ = cv2.findContours(mask_blue, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # Detectăm poarta galbenă
        mask_yellow = cv2.inRange(hsv_frame, low_hsv_yellow, high_hsv_yellow)
        contours_yellow, _ = cv2.findContours(mask_yellow, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # Prelucrare morfologică pentru ambele măști
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        mask_blue = cv2.morphologyEx(mask_blue, cv2.MORPH_CLOSE, kernel)
        mask_yellow = cv2.morphologyEx(mask_yellow, cv2.MORPH_CLOSE, kernel)

        # Combinăm rezultatele
        self.goal_detected = False
        self.goal_center_x = 0
        self.goal_area = 0

        for contours, color in [(contours_blue, 'blue'), (contours_yellow, 'yellow')]:
            for contour in contours:
                area = cv2.contourArea(contour)
                if area > 1000:  # Prag pentru dimensiunea porții
                    x, y, w, h = cv2.boundingRect(contour)
                    aspect_ratio = w / h  # Raport lățime/înălțime

                    if 0.8 < aspect_ratio < 2.0 and h > 50:  # Dimensiuni rezonabile pentru poartă
                        self.goal_detected = True
                        self.goal_center_x = x + (w // 2)
                        self.goal_area = area
                        cv2.rectangle(cv_image, (x, y), (x + w, y + h), (255, 0, 0), 2)
                        self.get_logger().info(f"Goal detected ({color}) at X: {self.goal_center_x}, Area: {self.goal_area}")
                        break
            if self.goal_detected:
                break

        # Afișăm imaginea procesată
        cv2.imshow("Processed Image", cv_image)
        cv2.waitKey(3)

        # Controlăm mișcarea robotului
        self.control_robot()

    def control_robot(self):
        twist = Twist()

        if self.state == "SEARCH_BALL":
            if self.ball_detected:
                self.state = "APPROACH_BALL"
                self.get_logger().info("Ball detected! Switching to APPROACH_BALL.")
            else:
                twist.angular.z = 0.2
                self.get_logger().info("Rotating, searching for ball...")

        elif self.state == "APPROACH_BALL":
            if self.ball_detected:
                error = self.cx - (self.width // 2)
                if abs(error) > 20:
                    twist.angular.z = -0.002 * error
                    twist.linear.x = 0.0
                elif self.area < 70000:
                    twist.angular.z = 0.0
                    twist.linear.x = 0.05
                else:
                    twist.angular.z = 0.0
                    twist.linear.x = 0.0
                    self.state = "SEARCH_GOAL"
                    self.get_logger().info("Ball reached! Switching to SEARCH_GOAL.")
            else:
                self.state = "SEARCH_BALL"
                self.get_logger().info("Lost ball, switching to SEARCH_BALL.")

        elif self.state == "SEARCH_GOAL":
            if self.goal_detected:
                self.state = "ALIGN_GOAL"
                self.get_logger().info("Goal detected! Switching to ALIGN_GOAL.")
            else:
                twist.angular.z = 0.2
                self.get_logger().info("Rotating, searching for goal...")

        elif self.state == "ALIGN_GOAL":
            error_goal = self.goal_center_x - (self.width // 2)
            if abs(error_goal) > 20:
                twist.angular.z = -0.002 * error_goal
                twist.linear.x = 0.0
            else:
                twist.angular.z = 0.0
                twist.linear.x = 0.0
                self.state = "PUSH_BALL"
                self.get_logger().info("Goal aligned! Ready for next action.")
                
        elif self.state == "PUSH_BALL":
            if self.ball_detected:
                error = self.cx - (self.width // 2)
                if abs(error) > 20:  # Ajustare pentru aliniere cu mingea
                    twist.angular.z = -0.001 * error
                    twist.linear.x = 0.0
                    self.get_logger().info("Adjusting alignment while pushing ball.")
                else:  # Deplasează robotul înainte pentru a împinge mingea
                    twist.angular.z = 0.0
                    twist.linear.x = 0.05
                    self.get_logger().info("Pushing ball towards goal.")
        else:
            twist.angular.z = 0.0
            twist.linear.x = 0.0
            self.state = "SEARCH_BALL"
            self.get_logger().info("Ball lost while pushing. Switching to SEARCH_BALL.")

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
