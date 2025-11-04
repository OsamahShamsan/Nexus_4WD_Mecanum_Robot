#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import signal
import sys


class TwistTester(Node):
    def __init__(self):
        super().__init__('twist_tester')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.publish_twist)  # 10 Hz

        # Default test velocity
        self.vx = 0.12   # m/s
        self.vy = 0.0    # m/s
        self.wz = 0.0    # rad/s

    def publish_twist(self):
        msg = Twist()
        msg.linear.x = self.vx
        msg.linear.y = self.vy
        msg.angular.z = self.wz
        self.pub.publish(msg)

    def stop_and_exit(self):
        """Send zero velocity to stop the robot."""
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.angular.z = 0.0
        self.pub.publish(msg)
        self.get_logger().info("Robot stopped — published 0,0,0 to /cmd_vel")


# === Main entry point ===
def main(args=None):
    rclpy.init(args=args)
    node = TwistTester()

    # Custom SIGINT handler
    def signal_handler(sig, frame):
        node.get_logger().info("SIGINT received. Stopping robot and shutting down...")
        node.stop_and_exit()
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

    # Register the handler so Ctrl+C triggers stop_and_exit()
    signal.signal(signal.SIGINT, signal_handler)

    try:
        rclpy.spin(node)
    except Exception as e:
        node.get_logger().error(f"Unexpected exception: {e}")
        node.stop_and_exit()
    finally:
        node.stop_and_exit()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
