#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TwistTester(Node):
    def __init__(self):
        super().__init__('twist_tester')
        self.pub = self.create_publisher(Twist, '/twist_nexus', 10)
        self.timer = self.create_timer(0.1, self.publish_twist)  # 10 Hz

        self.vx = 0.0   # m/s
        self.vy = 0.0   # m/s
        self.wz = 0.0   # rad/s

    def publish_twist(self):
        msg = Twist()
        msg.linear.x = self.vx
        msg.linear.y = self.vy
        msg.angular.z = self.wz
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TwistTester()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
