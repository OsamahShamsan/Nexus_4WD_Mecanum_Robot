#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class MoveRobotNode(Node):
    def __init__(self):
        super().__init__('move_robot_node')

        # Declare and get parameters
        self.declare_parameter('vx', 0.2)
        self.declare_parameter('vy', 0.0)
        self.declare_parameter('w', 0.0)
        self.declare_parameter('t', 2.0)
        self.declare_parameter('M', 'l')  # l=line, r=rotate, s=square

        self.vx = self.get_parameter('vx').value
        self.vy = self.get_parameter('vy').value
        self.w = self.get_parameter('w').value
        self.t = self.get_parameter('t').value
        self.mode = self.get_parameter('M').value.lower()

        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.start_time = time.time()
        self.step = 0

        if self.mode == 's':
            self.timer = self.create_timer(0.1, self.square_callback)
        elif self.mode == 'r':
            self.timer = self.create_timer(0.1, self.rotate_callback)
        elif self.mode == 'l':
            self.timer = self.create_timer(0.1, self.line_callback)
        else:
            self.get_logger().warn(f"Unknown mode '{self.mode}', defaulting to line.")
            self.timer = self.create_timer(0.1, self.line_callback)

        self.get_logger().info(f"Started move_robot_node in mode '{self.mode}'.")

    def square_callback(self):
        now = time.time()
        if self.step >= 4:
            self.stop_and_exit()
            return
        if now - self.start_time < self.t:
            twist = Twist()
            twist.linear.x = self.vx
            twist.linear.y = self.vy
            self.publisher_.publish(twist)
        elif now - self.start_time < self.t + 1.0:
            twist = Twist()
            twist.angular.z = self.w
            self.publisher_.publish(twist)
        else:
            self.step += 1
            self.start_time = now

    def line_callback(self):
        if time.time() - self.start_time < self.t:
            twist = Twist()
            twist.linear.x = self.vx
            twist.linear.y = self.vy
            twist.angular.z = self.w
            self.publisher_.publish(twist)
        else:
            self.stop_and_exit()

    def rotate_callback(self):
        if time.time() - self.start_time < self.t:
            twist = Twist()
            twist.angular.z = self.w
            self.publisher_.publish(twist)
        else:
            self.stop_and_exit()

    def stop_and_exit(self):
        self.send_stop_twist()
        self.get_logger().info("Motion complete. Stopping robot.")
        self.destroy_timer(self.timer)
        self.destroy_node()

    def send_stop_twist(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.angular.z = 0.0
        self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = MoveRobotNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Ctrl+C detected. Sending stop twist.")
        node.send_stop_twist()
        node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
