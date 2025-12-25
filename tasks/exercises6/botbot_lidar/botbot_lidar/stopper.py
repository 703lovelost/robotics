import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy

class ObstacleStopper(Node):
    def __init__(self):
        super().__init__('obstacle_stopper')

        qos_policy = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            qos_policy)

        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.safe_distance = 1.0
        self.speed = 0.5

    def listener_callback(self, msg):
        valid_ranges = [r for r in msg.ranges if r > 0.05 and r < 10.0]

        twist = Twist()

        if len(valid_ranges) == 0:
            min_dist = 0.0 
        else:
            min_dist = min(valid_ranges)

        if min_dist < self.safe_distance and min_dist > 0.05:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.get_logger().warning(f'Препятствие! Дистанция: {min_dist:.2f} м. СТОП!')
        else:
            twist.linear.x = self.speed
            twist.angular.z = 0.0

        self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleStopper()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop_msg = Twist()
        node.publisher_.publish(stop_msg)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()