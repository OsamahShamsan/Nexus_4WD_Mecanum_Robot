#!/usr/bin/env python3
"""
move_with_coordinates.py

ROS 2 node that drives the robot to a target (x, y) in the /odom frame.

- Subscribes: /odom (nav_msgs/msg/Odometry)
- Publishes: /cmd_vel (geometry_msgs/msg/Twist)

Behavior:
1) Rotate to face the goal.
2) Drive straight towards it.
3) When the distance < position_tolerance, send zero cmd_vel and stay stopped.
4) On Ctrl+C, also send a zero cmd_vel for safety.

Usage example:
  ros2 run nexus_control move_with_coordinates --ros-args \
    -p goal_x:=1.0 -p goal_y:=0.0
"""

import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


def quaternion_to_yaw(q):
    """Convert a geometry_msgs/msg/Quaternion into yaw angle (rad)."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle):
    """Wrap angle to [-pi, pi]."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def clamp(value, min_value, max_value):
    return max(min(value, max_value), min_value)


class GoToGoalNode(Node):
    def __init__(self):
        super().__init__('go_to_goal')

        # ---- Parameters ----
        self.declare_parameter('goal_x', 0.0)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('k_linear', 0.8)
        self.declare_parameter('k_angular', 1.5)
        self.declare_parameter('max_linear_speed', 0.3)   # m/s
        self.declare_parameter('max_angular_speed', 1.0)  # rad/s
        self.declare_parameter('position_tolerance', 0.05)  # m
        self.declare_parameter('yaw_tolerance', 0.05)       # rad (~3 deg)

        self.goal_x = float(self.get_parameter('goal_x').value)
        self.goal_y = float(self.get_parameter('goal_y').value)
        self.k_linear = float(self.get_parameter('k_linear').value)
        self.k_angular = float(self.get_parameter('k_angular').value)
        self.max_linear_speed = float(self.get_parameter('max_linear_speed').value)
        self.max_angular_speed = float(self.get_parameter('max_angular_speed').value)
        self.position_tolerance = float(self.get_parameter('position_tolerance').value)
        self.yaw_tolerance = float(self.get_parameter('yaw_tolerance').value)

        self.get_logger().info(
            f"Go-to-goal: goal_x={self.goal_x:.3f}, goal_y={self.goal_y:.3f} (odom frame)"
        )

        # ---- State ----
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.odom_ready = False
        self.goal_reached = False

        # ---- ROS interfaces ----
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # Control loop at 20 Hz
        self.control_timer = self.create_timer(0.05, self.control_loop)

    def odom_callback(self, msg: Odometry):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        self.odom_ready = True

    def control_loop(self):
        # Always be ready to stop
        stop_twist = Twist()

        # If no odom yet, keep robot stopped
        if not self.odom_ready:
            self.cmd_pub.publish(stop_twist)
            return

        # If we already reached the goal, keep sending zeros
        if self.goal_reached:
            self.cmd_pub.publish(stop_twist)
            return

        dx = self.goal_x - self.current_x
        dy = self.goal_y - self.current_y
        distance = math.sqrt(dx * dx + dy * dy)

        # Close enough to the goal
        if distance < self.position_tolerance:
            self.goal_reached = True
            self.get_logger().info(
                f"Goal reached (dist={distance:.3f} m). Stopping robot."
            )
            self.cmd_pub.publish(stop_twist)
            return

        # Desired yaw (world frame) pointing toward goal
        target_yaw = math.atan2(dy, dx)
        yaw_error = normalize_angle(target_yaw - self.current_yaw)

        twist = Twist()

        # 1) If yaw error is big, rotate in place first
        if abs(yaw_error) > self.yaw_tolerance:
            wz = self.k_angular * yaw_error
            wz = clamp(wz, -self.max_angular_speed, self.max_angular_speed)
            twist.angular.z = wz
            twist.linear.x = 0.0
        else:
            # 2) Yaw is roughly correct -> drive forward towards the goal
            vx = self.k_linear * distance
            vx = clamp(vx, -self.max_linear_speed, self.max_linear_speed)
            twist.linear.x = vx

            # Small heading correction while driving
            wz = self.k_angular * yaw_error
            wz = clamp(wz, -self.max_angular_speed, self.max_angular_speed)
            twist.angular.z = wz

        self.cmd_pub.publish(twist)

    def stop_robot(self):
        """Publish a single zero cmd_vel for safety."""
        stop = Twist()
        self.cmd_pub.publish(stop)
        self.get_logger().info("Sent zero /cmd_vel (stop_robot).")


def main(args=None):
    rclpy.init(args=args)
    node = GoToGoalNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # On Ctrl+C, send zero command for safety
        node.get_logger().info("KeyboardInterrupt received, stopping robot.")
        node.stop_robot()
    finally:
        # Extra safety: send zero again just before shutdown
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
