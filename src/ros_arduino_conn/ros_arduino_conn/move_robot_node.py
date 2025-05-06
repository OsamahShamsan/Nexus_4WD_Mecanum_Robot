#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class MoveRobotNode(Node):
    def __init__(self):
        super().__init__('move_robot_node')

        self.declare_parameters('', [
            ('vx', 0.0), ('vy', 0.0), ('w', 0.0),
            ('t_straight', 1.0), ('t_rotate', 1.0), ('M', 'l')
        ])

        self.vx = self.clamp(self.get_parameter('vx').value)
        self.vy = self.clamp(self.get_parameter('vy').value)
        self.w  = self.clamp(self.get_parameter('w').value)
        self.t_straight = self.get_parameter('t_straight').value
        self.t_rotate   = self.get_parameter('t_rotate').value
        self.mode = self.get_parameter('M').value.lower()
        
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.start_time = time.time()
        self.timer_freq = 0.03
        self.ramp_steps = 0.1
        self.current_vx = self.current_vy = self.current_w = 0.0

        valid_modes = {'s': self.square_callback, 'r': self.rotate_callback, 'l': self.line_callback}
        if self.mode not in valid_modes:
            self.get_logger().error(f"Invalid mode '{self.mode}'. Must be one of: {list(valid_modes.keys())}")
            rclpy.shutdown()
            return
        self.timer = self.create_timer(self.timer_freq, valid_modes[self.mode])
        self.get_logger().info(f"Started in mode '{self.mode}'.")

    def clamp(self, val, min_val=-1.0, max_val=1.0):
        return max(min(val, max_val), min_val)

    def ramp(self, current, target):
        diff = target - current
        return target if abs(diff) <= self.ramp_steps else current + self.ramp_steps * (diff / abs(diff))

    def ramp_and_publish(self, target_vx=0.0, target_vy=0.0, target_w=0.0):
        twist = Twist()
        twist.linear.x = self.ramp(twist.linear.x, target_vx)
        twist.linear.y = self.ramp(twist.linear.y, target_vy)
        twist.angular.z  = self.ramp(twist.angular.z,  target_w)
        self.publisher_.publish(twist)

    def ramp_and_publish(self, target_vx=0.0, target_vy=0.0, target_w=0.0):
        self.current_vx = self.ramp(getattr(self, 'current_vx', 0.0), self.clamp(target_vx))
        self.current_vy = self.ramp(getattr(self, 'current_vy', 0.0), self.clamp(target_vy))
        self.current_w  = self.ramp(getattr(self, 'current_w',  0.0), self.clamp(target_w))

        twist = Twist()
        twist.linear.x = self.current_vx
        twist.linear.y = self.current_vy
        twist.angular.z = self.current_w
        self.publisher_.publish(twist)

    def line_callback(self):
        if time.time() - self.start_time < self.t_straight:
            self.ramp_and_publish(self.vx, self.vy, 0.0) 
        else:
            self.stop_and_exit()
            return

    def rotate_callback(self):
        if time.time() - self.start_time < self.t_straight:
            self.ramp_and_publish(0.0, 0.0, self.w)
        else:
            self.stop_and_exit()
            return

    def square_callback(self):
        if not hasattr(self.square_callback, "step"):
            self.square_callback.step = 0

        now = time.time()
        elapsed = now - self.start_time

        if self.square_callback.step >= 4:
            self.square_callback.step = 0
            self.stop_and_exit()
            return

        if elapsed < self.t_straight:
            self.ramp_and_publish(self.vx, self.vy, 0.0)
        elif elapsed < self.t_straight + self.t_rotate:
            self.ramp_and_publish(0.0, 0.0, self.w)
        else:
            self.square_callback.step += 1
            self.start_time = now
            self.ramp_and_publish(0.0, 0.0, 0.0)


    def stop_and_exit(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.angular.z = 0.0
        self.publisher_.publish(twist)
        self.get_logger().info("Motion complete. Stopping robot.")
        self.destroy_timer(self.timer)

def main(args=None):
    rclpy.init(args=args)
    node = MoveRobotNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
