import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class Controller(Node):
    def __init__(self):
        super().__init__('controller')

        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        radius = 1.0
        speed = 1.5
        twist = Twist()
        ang_speed = speed / radius
        twist.linear.x = speed
        twist.angular.z = ang_speed
        self.publisher.publish(twist)

def main():
    rclpy.init()
    node = Controller()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()