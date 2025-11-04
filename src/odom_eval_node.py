#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import csv
import math
import threading
import time


# ========= EDIT THESE MOTIONS IF YOU WANT =========
#
# Coordinates are in meters, yaw in degrees
MOTIONS = {
    1: {
        "name": "straight_1m",
        "points": [
            (0.0, 0.0,   0.0,   "start"),
            (1.0, 0.0,   0.0,   "x1_y0"),
            (0.0, 0.0, 180.0,   "back_0_0"),
        ],
    },
    2: {
        "name": "straight_3m",
        "points": [
            (0.0, 0.0,   0.0,   "start"),
            (3.0, 0.0,   0.0,   "x3_y0"),
            (0.0, 0.0, 180.0,   "back_0_0"),
        ],
    },
    3: {
        "name": "rect_3x1",
        "points": [
            (0.0, 0.0,   0.0,   "start"),
            (3.0, 0.0,   0.0,   "x3_y0"),
            (3.0, 1.0,  90.0,   "x3_y1"),
            (0.0, 1.0, 180.0,   "x0_y1"),
            (0.0, 0.0, -90.0,   "back_0_0"),  # same as 270 deg
        ],
    },
}
# ==================================================

CSV_PATH = "odom_eval_results.csv"


class OdomEvalNode(Node):
    def __init__(self):
        super().__init__("odom_eval_node")
        self.latest_odom = None
        self.latest_filt = None

        self.create_subscription(Odometry, "/odom", self.odom_cb, 10)
        self.create_subscription(Odometry, "/odometry/filtered", self.filt_cb, 10)

    def odom_cb(self, msg):
        self.latest_odom = msg

    def filt_cb(self, msg):
        self.latest_filt = msg


def spin_thread(node):
    rclpy.spin(node)


def quat_to_yaw_deg(q):
    # yaw from quaternion, in degrees
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return math.degrees(yaw)


def wrap_deg(angle):
    # wrap to [-180, 180]
    while angle > 180.0:
        angle -= 360.0
    while angle < -180.0:
        angle += 360.0
    return angle


def main():
    rclpy.init()
    node = OdomEvalNode()

    th = threading.Thread(target=spin_thread, args=(node,), daemon=True)
    th.start()

    print("\n=== Odom evaluation node ===")
    print("Available motions:")
    for mid, m in MOTIONS.items():
        print(f"  {mid}: {m['name']} ({len(m['points'])} points)")

    motion_id = int(input("Select motion_id (number): ").strip())
    if motion_id not in MOTIONS:
        print("Invalid motion_id.")
        return

    trial_id = int(input("Enter trial_id (e.g. 1,2,3...): ").strip())

    motion = MOTIONS[motion_id]
    motion_name = motion["name"]
    points = motion["points"]

    print(f"\nSelected motion: {motion_name}")
    print("Ground-truth points:")
    for i, (x_gt, y_gt, yaw_gt, label) in enumerate(points):
        print(f"  sample {i}: ({x_gt:.2f}, {y_gt:.2f}), yaw={yaw_gt:.1f} deg, label={label}")

    print("\nWaiting for /odom and /odometry/filtered...")
    while node.latest_odom is None or node.latest_filt is None:
        time.sleep(0.1)
    print("OK, receiving messages.")

    # Prepare CSV
    file_exists = False
    try:
        with open(CSV_PATH, "r"):
            file_exists = True
    except FileNotFoundError:
        file_exists = False

    f = open(CSV_PATH, "a", newline="")
    writer = csv.writer(f)

    if not file_exists:
        writer.writerow([
            "trial_id", "motion_id", "motion_name",
            "sample_idx", "sample_label", "t_sec",
            "x_gt", "y_gt", "yaw_gt_deg",
            "x_odom", "y_odom", "yaw_odom_deg",
            "dx_odom", "dy_odom", "epos_odom", "dyaw_odom_deg",
            "x_filt", "y_filt", "yaw_filt_deg",
            "dx_filt", "dy_filt", "epos_filt", "dyaw_filt_deg",
        ])

    # For covariance stats (odom)
    dx_odom_list = []
    dy_odom_list = []

    print("\nFor EACH point:")
    print("  1) Drive robot to floor mark.")
    print("  2) Align orientation.")
    print("  3) Let it stand still.")
    print("  4) Press ENTER here to log.\n")

    for i, (x_gt, y_gt, yaw_gt_deg, label) in enumerate(points):
        input(f"Sample {i} ({label}) at ({x_gt},{y_gt}), yaw={yaw_gt_deg} deg -> press ENTER to log...")

        odom = node.latest_odom
        filt = node.latest_filt

        # time (sec)
        t = odom.header.stamp.sec + odom.header.stamp.nanosec * 1e-9

        # odom pose
        xo = odom.pose.pose.position.x
        yo = odom.pose.pose.position.y
        yaw_o_deg = quat_to_yaw_deg(odom.pose.pose.orientation)

        # filtered pose
        xf = filt.pose.pose.position.x
        yf = filt.pose.pose.position.y
        yaw_f_deg = quat_to_yaw_deg(filt.pose.pose.orientation)

        # errors odom
        dx_o = xo - x_gt
        dy_o = yo - y_gt
        epos_o = math.sqrt(dx_o * dx_o + dy_o * dy_o)
        dyaw_o = wrap_deg(yaw_o_deg - yaw_gt_deg)

        # errors filtered
        dx_f = xf - x_gt
        dy_f = yf - y_gt
        epos_f = math.sqrt(dx_f * dx_f + dy_f * dy_f)
        dyaw_f = wrap_deg(yaw_f_deg - yaw_gt_deg)

        dx_odom_list.append(dx_o)
        dy_odom_list.append(dy_o)

        writer.writerow([
            trial_id, motion_id, motion_name,
            i, label, f"{t:.6f}",
            x_gt, y_gt, yaw_gt_deg,
            xo, yo, yaw_o_deg,
            dx_o, dy_o, epos_o, dyaw_o,
            xf, yf, yaw_f_deg,
            dx_f, dy_f, epos_f, dyaw_f,
        ])

        print(f"  logged: gt=({x_gt:.3f},{y_gt:.3f}) "
              f"odom=({xo:.3f},{yo:.3f}) epos_odom={epos_o:.3f} "
              f"filt=({xf:.3f},{yf:.3f}) epos_filt={epos_f:.3f}")

    f.close()

    # --- simple covariance for odom (dx, dy) ---
    n = len(dx_odom_list)
    if n >= 2:
        mean_dx = sum(dx_odom_list) / n
        mean_dy = sum(dy_odom_list) / n
        sum_xx = sum((dx - mean_dx) ** 2 for dx in dx_odom_list)
        sum_yy = sum((dy - mean_dy) ** 2 for dy in dy_odom_list)
        sum_xy = sum((dx - mean_dx) * (dy - mean_dy)
                     for dx, dy in zip(dx_odom_list, dy_odom_list))
        var_x = sum_xx / (n - 1)
        var_y = sum_yy / (n - 1)
        cov_xy = sum_xy / (n - 1)

        print("\nOdom position covariance (dx, dy) [m^2]:")
        print(f"[ {var_x:.6f}   {cov_xy:.6f} ]")
        print(f"[ {cov_xy:.6f}   {var_y:.6f} ]")
    else:
        print("\nNot enough samples for covariance.")

    rclpy.shutdown()
    print(f"\nDone. Results appended to {CSV_PATH}")


if __name__ == "__main__":
    main()
