#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import signal, sys, math


class TwistTester(Node):
    def __init__(self, mode="velocity"):
        super().__init__('twist_tester')

        # --- Publishers/Subscribers ---
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.timer = self.create_timer(0.1, self.update)  # 10 Hz

        # --- Mode: "velocity" or "pose" ---
        self.mode = mode

        # --- Default constant speed (for velocity mode) ---
        self.vx = 0.33   # m/s
        self.vy = 0.0
        self.wz = 0.0    # rad/s

        # --- Target position/orientation (for pose mode) ---
        self.x_target = 1.0    # meters
        self.y_target = 0.0
        self.theta_target = 0.0  # radians

        # --- Internal odom state ---
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.odom_received = False

        # --- Thresholds to stop ---
        self.tolerance_pos = 0.02    # 2 cm
        self.tolerance_ang = 0.05    # 3°

        self.get_logger().info(f"TwistTester started in '{self.mode}' mode")

    # ------------------------------------------------------------------
    def odom_callback(self, msg: Odometry):
        """Receive robot pose from /odom."""
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.theta = math.atan2(siny_cosp, cosy_cosp)

        self.odom_received = True

    # ------------------------------------------------------------------
    def update(self):
        """Main update loop."""
        if self.mode == "velocity":
            self.publish_constant_velocity()
        elif self.mode == "pose":
            self.publish_velocity_to_target()

    # ------------------------------------------------------------------
    def publish_constant_velocity(self):
        msg = Twist()
        msg.linear.x = self.vx
        msg.linear.y = self.vy
        msg.angular.z = self.wz
        self.pub.publish(msg)

    # ------------------------------------------------------------------
    def publish_velocity_to_target(self):
        """Compute simple geometric velocity commands toward (x_target, y_target, θ_target)."""
        if not self.odom_received:
            self.get_logger().warn_throttle(5.0, "Waiting for /odom messages...")
            return

        dx = self.x_target - self.x
        dy = self.y_target - self.y
        dist = math.hypot(dx, dy)

        if dist < self.tolerance_pos:
            # stop if close enough
            self.get_logger().info("Reached target position.")
            self.stop_and_exit()
            self.mode = "idle"
            return

        # Compute heading direction to target
        angle_to_target = math.atan2(dy, dx)
        angle_error = self._normalize_angle(angle_to_target - self.theta)
        theta_error = self._normalize_angle(self.theta_target - self.theta)

        # --- Velocity commands (simple geometric approach, no PID) ---
        # Forward speed proportional to remaining distance (capped)
        v = min(dist, 0.3)   # [m/s]
        # Rotate gently toward target
        w = max(min(angle_error, 0.5), -0.5)  # [rad/s] limited

        # If near final target, correct heading
        if dist < 0.1:
            v = 0.0
            w = theta_error

        msg = Twist()
        msg.linear.x = v
        msg.linear.y = 0.0
        msg.angular.z = w
        self.pub.publish(msg)

    # ------------------------------------------------------------------
    def _normalize_angle(self, a):
        while a > math.pi:
            a -= 2.0 * math.pi
        while a < -math.pi:
            a += 2.0 * math.pi
        return a

    # ------------------------------------------------------------------
    def stop_and_exit(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.angular.z = 0.0
        self.pub.publish(msg)
        self.get_logger().info("Robot stopped — published 0,0,0 to /cmd_vel.")


# ======================================================================
def main(args=None):
    rclpy.init(args=args)

    mode = "velocity"
    if len(sys.argv) > 1 and sys.argv[1] in ("pose", "position", "xy"):
        mode = "pose"

    node = TwistTester(mode)

    def signal_handler(sig, frame):
        node.get_logger().info("SIGINT received — stopping robot...")
        node.stop_and_exit()
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)

    try:
        rclpy.spin(node)
    finally:
        node.stop_and_exit()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
