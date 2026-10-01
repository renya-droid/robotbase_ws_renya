#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class IdleTwistPublisher(Node):
    """Keep a zero-velocity fallback available to twist_mux."""

    def __init__(self):
        super().__init__('idle_twist_publisher')
        rate = self.declare_parameter('publish_rate', 10.0).value
        self.publisher = self.create_publisher(Twist, 'cmd_vel_idle', 10)
        self.timer = self.create_timer(1.0 / max(float(rate), 1.0), self.publish_zero)

    def publish_zero(self):
        self.publisher.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = IdleTwistPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
