#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn

class Spawner(Node):
    def __init__(self):
        super().__init__('spawn_turtle2')
        self.cli = self.create_client(Spawn, '/spawn')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /spawn service...')
        req = Spawn.Request()
        req.x = 5.0
        req.y = 5.0
        req.theta = 0.0
        req.name = 'turtle2'
        self.future = self.cli.call_async(req)
        self.future.add_done_callback(self.done)

    def done(self, fut):
        try:
            resp = fut.result()
            self.get_logger().info(f"Spawned: {resp.name}")
        except Exception as e:
            self.get_logger().error(f"Spawn failed: {e}")
        rclpy.shutdown()

def main():
    rclpy.init()
    node = Spawner()
    rclpy.spin(node)

if __name__ == '__main__':
    main()

