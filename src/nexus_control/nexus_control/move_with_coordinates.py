#!/usr/bin/env python3
"""
move_with_coordinates.py

Single motion node with multiple modes, using /odom feedback and publishing /cmd_vel.

Modes (parameter: motion_type):

  "go_to_goal"  -> drive to (goal_x, goal_y) in odom frame
                  (vx + optional rotation; simple controller)
  "move_x"      -> move a given distance (m) with pure vx
                   vy = 0, wz = 0
  "move_y"      -> move a given distance (m) with pure vy (strafe)
                   vx = 0, wz = 0
  "rotate"      -> rotate a given angle (deg) in place with pure wz
                   vx = 0, vy = 0

All modes:
  - Subscribes: /odom (nav_msgs/msg/Odometry)
  - Publishes: /cmd_vel (geometry_msgs/msg/Twist)
  - Stops when goal reached and keeps sending zeros
  - On Ctrl+C (SIGINT) sends zero /cmd_vel and shuts down safely

Examples:

  # 1) Go to (x=1.0, y=0.0) using /odom
  ros2 run nexus_control move_with_coordinates --ros-args \
    -p motion_type:="go_to_goal" -p goal_x:=1.0 -p goal_y:=0.0

  # 2) Move +1 m forward with pure vx (vy=0, wz=0)
  ros2 run nexus_control move_with_coordinates --ros-args \
    -p motion_type:="move_x" -p distance:=1.0

  # 3) Strafe +1 m in Y with pure vy (vx=0, wz=0)
  ros2 run nexus_control move_with_coordinates --ros-args \
    -p motion_type:="move_y" -p distance:=1.0

  # 4) Rotate +360 deg (vx=0, vy=0, wz != 0)
  ros2 run nexus_control move_with_coordinates --ros-args \
    -p motion_type:="rotate" -p angle_deg:=360.0
"""

import math
import signal

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


def quaternion_to_yaw(q):
    """Convert geometry_msgs/msg/Quaternion into yaw angle (rad)."""
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


def sign(x: float) -> float:
    if x > 0.0:
        return 1.0
    if x < 0.0:
        return -1.0
    return 0.0


def clamp(value, min_value, max_value):
    return max(min(value, max_value), min_value)


class MultiMotionNode(Node):
    def __init__(self):
        super().__init__('multi_motion_node')

        # ---- Common mode selection ----
        self.declare_parameter('motion_type', 'go_to_goal')
        self.motion_type = self.get_parameter('motion_type').value

        # ---- Go-to-goal parameters (odom frame) ----
        self.declare_parameter('goal_x', 0.0)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('k_linear', 0.8)
        self.declare_parameter('k_angular', 1.5)
        self.declare_parameter('max_linear_speed', 0.3)   # m/s (vx)
        self.declare_parameter('max_side_speed', 0.3)      # m/s (vy) if used
        self.declare_parameter('max_angular_speed', 1.0)   # rad/s (wz)
        self.declare_parameter('position_tolerance', 0.05) # m
        self.declare_parameter('yaw_tolerance', 0.05)      # rad
        self.declare_parameter('brake_distance', 0.15)    # m, stop this far before target for X/Y


        self.goal_x = float(self.get_parameter('goal_x').value)
        self.goal_y = float(self.get_parameter('goal_y').value)
        self.k_linear = float(self.get_parameter('k_linear').value)
        self.k_angular = float(self.get_parameter('k_angular').value)
        self.max_linear_speed = float(self.get_parameter('max_linear_speed').value)
        self.max_side_speed = float(self.get_parameter('max_side_speed').value)
        self.max_angular_speed = float(self.get_parameter('max_angular_speed').value)
        self.position_tolerance = float(self.get_parameter('position_tolerance').value)
        self.yaw_tolerance = float(self.get_parameter('yaw_tolerance').value)
        self.brake_distance = float(self.get_parameter('brake_distance').value)
        
        # ---- Axis-test parameters (pure vx / vy / wz) ----
        self.declare_parameter('distance', 1.0)           # m, for move_x/move_y
        self.declare_parameter('angle_deg', 360.0)        # deg, for rotate
        self.declare_parameter('speed_lin', 0.2)          # m/s for move_x
        self.declare_parameter('speed_strafe', 0.2)       # m/s for move_y
        self.declare_parameter('speed_rot', 0.5)          # rad/s for rotate
        self.declare_parameter('dist_tolerance', 0.01)    # m
        self.declare_parameter('angle_tolerance_deg', 5.0) # deg

        self.distance = float(self.get_parameter('distance').value)
        self.angle_deg = float(self.get_parameter('angle_deg').value)
        self.speed_lin = float(self.get_parameter('speed_lin').value)
        self.speed_strafe = float(self.get_parameter('speed_strafe').value)
        self.speed_rot = float(self.get_parameter('speed_rot').value)
        self.dist_tolerance = float(self.get_parameter('dist_tolerance').value)
        self.angle_tolerance_deg = float(self.get_parameter('angle_tolerance_deg').value)
        self.angle_tolerance_rad = math.radians(self.angle_tolerance_deg)
        self.target_angle_rad = math.radians(self.angle_deg)

        # ---- State ----
        self.odom_received = False
        self.goal_reached = False

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

        self.start_x = None
        self.start_y = None
        self.start_yaw = None

        self.last_yaw = None
        self.cum_yaw = 0.0  # for rotation

        # ---- ROS interfaces ----
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )
        self.timer = self.create_timer(0.05, self.control_loop)  # 20 Hz

        self.get_logger().info(
            f"MultiMotionNode started. motion_type='{self.motion_type}'."
        )

    # ---------------- ODOM CALLBACK ----------------

    def odom_callback(self, msg: Odometry):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        self.current_yaw = yaw

        if not self.odom_received:
            # First odom: store start pose
            self.start_x = self.current_x
            self.start_y = self.current_y
            self.start_yaw = self.current_yaw
            self.last_yaw = self.current_yaw
            self.cum_yaw = 0.0
            self.odom_received = True

            self.get_logger().info(
                f"Initial pose: x={self.start_x:.3f}, y={self.start_y:.3f}, yaw={self.start_yaw:.3f} rad"
            )
        else:
            # Track cumulative yaw only needed for rotation
            if self.motion_type == 'rotate':
                dyaw = normalize_angle(self.current_yaw - self.last_yaw)
                self.cum_yaw += dyaw
                self.last_yaw = self.current_yaw

    # ---------------- CONTROL LOOP ----------------

    def control_loop(self):
        stop_twist = Twist()

        if not self.odom_received:
            # No odom yet -> keep stopped
            self.cmd_pub.publish(stop_twist)
            return

        if self.goal_reached:
            # Goal reached -> keep sending zero for safety
            self.cmd_pub.publish(stop_twist)
            return

        twist = Twist()

        # Dispatch by motion_type
        if self.motion_type == 'go_to_goal':
            self.control_go_to_goal(twist)
        elif self.motion_type == 'move_x':
            self.control_move_x(twist)
        elif self.motion_type == 'move_y':
            self.control_move_y(twist)
        elif self.motion_type == 'rotate':
            self.control_rotate(twist)
        else:
            self.get_logger().warn(
                f"Unknown motion_type '{self.motion_type}', stopping."
            )
            self.goal_reached = True
            twist = stop_twist

        self.cmd_pub.publish(twist)

    # ---------- MODE: go_to_goal (simple controller) ----------

    def control_go_to_goal(self, twist: Twist):
        dx = self.goal_x - self.current_x
        dy = self.goal_y - self.current_y
        distance = math.sqrt(dx * dx + dy * dy)

        if distance < self.position_tolerance:
            self.get_logger().info(
                f"[go_to_goal] Goal reached (dist={distance:.3f} m). Stopping."
            )
            self.goal_reached = True
            return

        # Heading towards goal (world frame)
        target_yaw = math.atan2(dy, dx)
        yaw_error = normalize_angle(target_yaw - self.current_yaw)

        # Strategy: rotate to reduce yaw error, but keep motion simple (vx + small wz)
        # We do NOT use vy here, to keep movements cleaner.
        if abs(yaw_error) > self.yaw_tolerance:
            # Rotate in place
            vz = self.k_angular * yaw_error
            vz = clamp(vz, -self.max_angular_speed, self.max_angular_speed)
            twist.angular.z = vz
            twist.linear.x = 0.0
        else:
            # Drive forward with small heading correction
            vx = self.k_linear * distance
            vx = clamp(vx, -self.max_linear_speed, self.max_linear_speed)
            twist.linear.x = vx

            vz = self.k_angular * yaw_error
            vz = clamp(vz, -self.max_angular_speed, self.max_angular_speed)
            twist.angular.z = vz

        # vy remains 0 for this mode
        twist.linear.y = 0.0

    # ---------- MODE: move_x (pure vx, stop early with brake_distance) ----------

    def control_move_x(self, twist: Twist):
        if self.start_x is None:
            return

        # Signed distance along X axis
        dx = self.current_x - self.start_x
        target = abs(self.distance)
        direction = sign(self.distance)

        distance_done = dx * direction  # progress in commanded direction

        # How far before the target we stop commanding (to let it coast)
        stop_margin = min(abs(self.brake_distance), target)

        # If we've already reached "target - stop_margin", stop sending commands
        if distance_done >= target - stop_margin:
            self.get_logger().info(
                f"[move_x] Early stop for braking: Δx={dx:.3f} m, "
                f"target={self.distance:.3f} m, brake_distance={self.brake_distance:.3f} m."
            )
            self.goal_reached = True
            return

        # PURE vx: vy = 0, wz = 0
        twist.linear.x = self.speed_lin * direction
        twist.linear.y = 0.0
        twist.angular.z = 0.0

    # ---------- MODE: move_y (pure vy / strafe, stop early with brake_distance) ----------

    def control_move_y(self, twist: Twist):
        if self.start_y is None:
            return

        # Signed distance along Y axis
        dy = self.current_y - self.start_y
        target = abs(self.distance)
        direction = sign(self.distance)

        distance_done = dy * direction  # progress in commanded direction

        stop_margin = min(abs(self.brake_distance), target)

        if distance_done >= target - stop_margin:
            self.get_logger().info(
                f"[move_y] Early stop for braking: Δy={dy:.3f} m, "
                f"target={self.distance:.3f} m, brake_distance={self.brake_distance:.3f} m."
            )
            self.goal_reached = True
            return

        # PURE vy: vx = 0, wz = 0
        twist.linear.x = 0.0
        twist.linear.y = self.speed_strafe * direction
        twist.angular.z = 0.0


     # ---------- MODE: rotate (pure wz, with slow-down near target) ----------

    def control_rotate(self, twist: Twist):
        # Total target angle (rad, absolute)
        target = abs(self.target_angle_rad)
        direction = sign(self.target_angle_rad)

        # How much we've rotated already (abs)
        angle_done = abs(self.cum_yaw)
        remaining = target - angle_done

        # Close enough -> stop
        if remaining <= self.angle_tolerance_rad:
            self.get_logger().info(
                f"[rotate] Target reached: rotated ~{math.degrees(self.cum_yaw):.1f} deg "
                f"(requested {self.angle_deg:.1f} deg). Stopping."
            )
            self.goal_reached = True
            return

        # Slow down when we get close to the target.
        # Example: start slowing down in the last 45 degrees.
        slowdown_angle = math.radians(45.0)

        # Scale speed between 0.2 and 1.0 depending on how far we still have to go.
        scale = remaining / slowdown_angle
        if scale > 1.0:
            scale = 1.0
        if scale < 0.2:
            scale = 0.2

        wz = self.speed_rot * direction * scale

        # PURE wz: vx = 0, vy = 0
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.angular.z = wz

    # ---------- Safety stop ----------

    def stop_robot(self):
        """Publish a zero cmd_vel once, for safety."""
        stop = Twist()
        self.cmd_pub.publish(stop)
        self.get_logger().info("Sent zero /cmd_vel (stop_robot).")


def main(args=None):
    rclpy.init(args=args)
    node = MultiMotionNode()

    # Handle Ctrl+C with explicit signal handler
    def sigint_handler(sig, frame):
        node.get_logger().info("SIGINT received, stopping robot via signal handler.")
        node.stop_robot()
        rclpy.shutdown()

    signal.signal(signal.SIGINT, sigint_handler)

    try:
        rclpy.spin(node)
    finally:
        node.stop_robot()
        node.destroy_node()


if __name__ == '__main__':
    main()
