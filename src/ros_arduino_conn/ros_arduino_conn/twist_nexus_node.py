#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray

WHEELSPAN = 300  # mm

class TwistNexusNode(Node):
    def __init__(self):
        super().__init__('twist_nexus_node')

        self.declare_parameter('linear_scale', 500.0)
        self.declare_parameter('angular_scale', 1.0)
        self.declare_parameter('linear_ramp_step', 20.0)
        self.declare_parameter('angular_ramp_step', 0.1)

        self.linear_scale = self.get_parameter('linear_scale').value
        self.angular_scale = self.get_parameter('angular_scale').value
        self.linear_ramp_step = self.get_parameter('linear_ramp_step').value
        self.angular_ramp_step = self.get_parameter('angular_ramp_step').value

        #self.publisher = self.create_publisher(Int32MultiArray, '/twist_nexus', 10)
        self.publisher = self.create_publisher(Int32MultiArray, '/encoder_wheels', 10)

        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        self.timer = self.create_timer(0.01, self.loop)  # Main loop 50 Hz

        self.target_vx = 0.0
        self.target_vy = 0.0
        self.target_omega = 0.0

        self.current_vx = 0.0
        self.current_vy = 0.0
        self.current_omega = 0.0

    def cmd_vel_callback(self, msg):
        self.target_vx = msg.linear.x * self.linear_scale
        self.target_vy = msg.linear.y * self.linear_scale
        self.target_omega = msg.angular.z * self.angular_scale

    # Function to not allow values below -1 and over 1
    def clamp(self, val, min_val, max_val):
        return max(min(val, max_val), min_val)
    
    def ramp(self, current, target, step):
        diff = target - current
        if abs(diff) <= step:
            return target
        return current + step * (diff / abs(diff))

    def loop(self):
        publishers = self.subscription.get_publisher_count()
        msg = Int32MultiArray()
        
        if publishers == 0:
            #self.get_logger().info(f"1 Number of publishers on {self.input_topic}: {publishers}")
            self.current_vx =    self.ramp(self.current_vx, 0, self.linear_ramp_step)
            self.current_vy =    self.ramp(self.current_vy, 0, self.linear_ramp_step)
            self.current_omega = self.ramp(self.current_omega, 0, self.angular_ramp_step)
        else:
            self.current_vx =    self.clamp(self.ramp(self.current_vx, self.target_vx, self.linear_ramp_step), -500, 500)
            self.current_vy =    self.clamp(self.ramp(self.current_vy, self.target_vy, self.linear_ramp_step), -500, 500)
            self.current_omega = self.clamp(self.ramp(self.current_omega, self.target_omega, self.angular_ramp_step), -1, 1)

        r = WHEELSPAN
        v1 = int( (self.current_vx - self.current_vy - (r * self.current_omega)))
        v2 = int(-(self.current_vx + self.current_vy + (r * self.current_omega)))
        v3 = int( (self.current_vx + self.current_vy - (r * self.current_omega)))
        v4 = int(-(self.current_vx - self.current_vy + (r * self.current_omega)))

        msg.data = [v1, v2, v3, v4]

        self.publisher.publish(msg)
        #self.get_logger().info(f"2 Number of publishers on {self.input_topic}: {publishers}")

        

def main(args=None):
    rclpy.init(args=args)
    node = TwistNexusNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
