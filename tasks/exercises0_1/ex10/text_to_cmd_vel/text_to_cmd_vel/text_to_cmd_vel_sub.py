import sys

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist

def main():
    rclpy.init()

    node = rclpy.create_node('t_text_to_cmd_vel')
    publisher = node.create_publisher(Twist, '/turtle1/cmd_vel', 10)
    cmd = sys.argv[1] if len(sys.argv) > 1 else ''

    msg = Twist()

    if cmd == 'turn_right':
      msg.angular.z = -1.5
    elif cmd == 'turn_left':
      msg.angular.z = 1.5
    elif cmd == 'move_forward':
      msg.linear.x = 1.0
    elif cmd == 'move_backward':
      msg.linear.x = -1.0
    else:
      pass

    # publisher.publish(msg)

    node.get_logger().debug(f'linear: ({msg.linear.x:.2f}, {msg.linear.y:.2f}, {msg.linear.z:.2f}), '
                            f'angular: ({msg.angular.x:.2f}, {msg.angular.y:.2f}, {msg.angular.z:.2f})')
    node.get_logger().info(f'Sending command: {cmd}')

    try:
      rclpy.spin_once(node, timeout_sec=1)
      publisher.publish(msg)
    finally:
      node.destroy_node()
      rclpy.shutdown()

if __name__ == '__main__':
    main()
