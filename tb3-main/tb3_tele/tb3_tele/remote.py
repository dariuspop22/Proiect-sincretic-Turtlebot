import rclpy
from rclpy.node import Node
import sys
from geometry_msgs.msg import Twist

class TelePublisher(Node):
    
    def __init__(self):
        super().__init__('tele_publisher')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.linear=0
        # self.get_logger().info('start')
        #timer_period = 0.5  # seconds
        #self.timer = self.create_timer(timer_period, self.timer_callback)
        #self.i = 0

    def operate(self):
        self.get_logger().info('start')
        while(1):
            self.get_logger().info('teleop')
            key = sys.stdin.read(1)
            if key == 'w':
                self.linear += 0.01

            twist = Twist()
            twist.linear.x = self.linear
            self.publisher_.publish(twist)

    # def timer_callback(self):
    #     msg = String()
    #     msg.data = 'Hello World: %d' % self.i
    #     self.publisher_.publish(msg)
    #     self.get_logger().info('Publishing: "%s"' % msg.data)
    #     self.i += 1


def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = TelePublisher()
    minimal_publisher.operate()
    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()