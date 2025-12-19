import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CircleController(Node):
    def __init__(self):
        super().__init__('circle_controller')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('linear_x', 0.2)
        self.declare_parameter('angular_z', 0.2)
        self.declare_parameter('rate_hz', 10.0)
        try:
            self.declare_parameter('use_sim_time', True)
        except Exception:
            pass

        topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        linear_x = float(self.get_parameter('linear_x').value)
        angular_z = float(self.get_parameter('angular_z').value)
        rate_hz = float(self.get_parameter('rate_hz').value)

        self._msg = Twist()
        self._msg.linear.x = linear_x
        self._msg.angular.z = angular_z

        self._pub = self.create_publisher(Twist, topic, 10)
        self._timer = self.create_timer(1.0 / max(rate_hz, 1e-3), self._tick)

    def _tick(self):
        self._pub.publish(self._msg)


def main():
    rclpy.init()
    node = CircleController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()