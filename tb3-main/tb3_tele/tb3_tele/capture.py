import rclpy
from rclpy.node import Node
import sys
import cv2
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class CaptureSub(Node):
    
    def __init__(self):
        super().__init__('capture')
        self.get_logger().info('start')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.callback,
            10)
        self.subscription
        self.bridge = CvBridge()         
        
    def callback(self,data):
        self.get_logger().info('frame')
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data)
        except CvBridgeError as e:
            print(e)    
        edges = cv2.Canny(cv_image,100,200)

        cv2.imshow("Image window", edges)
        cv2.waitKey(3)

        # try:
        #     self.image_pub.publish(self.bridge.cv2_to_imgmsg(edges, "8UC1"))
        # except CvBridgeError as e:
        # print(e)

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = CaptureSub()
    # minimal_publisher.operate()
    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
