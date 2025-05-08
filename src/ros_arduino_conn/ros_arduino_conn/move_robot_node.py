#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class MoveRobotNode(Node):
    def __init__(self):
        super().__init__('move_robot_node')

        # Parameters that can be passed by the user:
        # vx = forward speed , vy = sideway speed (positive vy = move sideway left), w = rotation around z-axis (omega_z),
        # t_straight = time to move the robot in straight lines (forward, backward or diagonal), t_rotate = time to rotate the robot (see square_callback function)
        # M = is the mode in which the robot will move (l = line, s = square and r = rotation on place)      
        self.declare_parameters('', [
            ('vx', 0.0), ('vy', 0.0), ('w', 0.0),
            ('t_straight', 1.0), ('t_rotate', 1.0), ('M', 'l')
        ])

        self.vx = self.clamp(self.get_parameter('vx').value)             # Get the param vx and set it within the allowed range [-1,1] => to not allow input of extreme values 
        self.vy = self.clamp(self.get_parameter('vy').value)             # Get the param vy and set it within the allowed range [-1,1] => to not allow input of extreme values
        self.w  = self.clamp(self.get_parameter('w').value)              # Get the param w  and set it within the allowed range [-1,1] =>  to not allow input of extreme values
        self.t_straight = self.get_parameter('t_straight').value         # Get the param t_straight
        self.t_rotate   = self.get_parameter('t_rotate').value           # Get the param t_rotate
        self.mode = self.get_parameter('M').value.lower()                # Get the param M (Mode of movement) and convert uppercases chars to lowercases   

        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)    # Publisher to publish the needed speed to cmd_vel
        self.start_time = time.time()                                    # 
        self.timer_freq = 0.03                                           # The frequency in which the mode functions should send a new speed command to cmd_vel topic
        self.ramp_steps = 0.1                                            # The steps in which each cmd_vel command should increment => to not cause a rapid change in speed e.g. from -1 directly to 1 (safety for motors)
        self.current_vx = self.current_vy = self.current_w = 0.0         # The speeds that will be sent to the cmd_vel

        # Assign s, r, and l to their mode_callback 
        valid_modes = {'s': self.square_callback, 'r': self.rotate_callback, 'l': self.line_callback}

        # If user input does not match one of the valid_modes => reject throw and error 
        if self.mode not in valid_modes:
            self.get_logger().error(f"Invalid mode '{self.mode}'. Must be one of: {list(valid_modes.keys())}")
            rclpy.shutdown()
            return
        
        # Start a timer and assign the callback_func based on the user input mode
        self.timer = self.create_timer(self.timer_freq, valid_modes[self.mode])

        # Debug message
        self.get_logger().info(f"Started in mode '{self.mode}'.")

    # Function to not allow values below -1 and over 1
    def clamp(self, val, min_val=-1.0, max_val=1.0):
        return max(min(val, max_val), min_val)

    # Function to increase or decrease the target value with a fixed ramp_steps 
    def ramp(self, current, target):
        diff = target - current
        return target if abs(diff) <= self.ramp_steps else current + self.ramp_steps * (diff / abs(diff))

    # Function to ramp the values and publish them to cmd_vel 
    def ramp_and_publish(self, target_vx=0.0, target_vy=0.0, target_w=0.0):
        self.current_vx = self.ramp(self.current_vx, target_vx)
        self.current_vy = self.ramp(self.current_vy, target_vy)
        self.current_w  = self.ramp(self.current_w,  target_w)

        twist = Twist()
        twist.linear.x = self.current_vx
        twist.linear.y = self.current_vy
        twist.angular.z = self.current_w
        self.publisher_.publish(twist)

    # Callback function to move in straight lines
    def line_callback(self):
        if time.time() - self.start_time < self.t_straight:
            self.ramp_and_publish(self.vx, self.vy, 0.0) 
        else:
            self.stop_and_exit()
            return
        
    # Callback function to rotate in place
    def rotate_callback(self):
        if time.time() - self.start_time < self.t_straight:
            self.ramp_and_publish(0.0, 0.0, self.w)
        else:
            self.stop_and_exit()
            return

    # Callback function to move in a square shape
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

    # Stop the robot and destroy the timer
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
