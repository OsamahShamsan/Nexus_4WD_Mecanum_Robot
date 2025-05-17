#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Float32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
from tf_transformations import quaternion_from_euler, euler_from_quaternion
from tf2_ros import TransformBroadcaster
import math
from std_srvs.srv import Empty

class NexusOdomNode(Node):
    def __init__(self):
        super().__init__('nexus_odom_node')

        # Parameters
        self.declare_parameter('wheelspan', 300.0)
        self.wheelspan = self.get_parameter('wheelspan').value

        # Robot pose
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Time tracking
        self.last_time = self.get_clock().now().nanoseconds / 1e9

        # ROS interfaces
        self.subscription = self.create_subscription( Int32MultiArray, '/encoder_wheels', self.encoder_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.odom_deg_pub = self.create_publisher( Float32MultiArray, '/odom_degree', 10)

        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

    def apply_deadzone(self, value, threshold):
        if abs(value) < threshold:
            return 0.0
        return value

    def encoder_callback(self, msg):
        if len(msg.data) != 4:
            self.get_logger().warn("Received malformed encoder data")
            return

        threshold = 15  # in mm/s

        v1_mmps, v2_mmps, v3_mmps, v4_mmps = msg.data
        
        v1_mmps = self.apply_deadzone(v1_mmps, threshold)
        v2_mmps = self.apply_deadzone(v2_mmps, threshold)
        v3_mmps = self.apply_deadzone(v3_mmps, threshold)
        v4_mmps = self.apply_deadzone(v4_mmps, threshold)

        #self.get_logger().info(f"v1_mmps: {v1_mmps:.3f} m/s, v2_mmps: {v2_mmps:.3f} m/s, v3_mmps: {v3_mmps:.3f} m/s, v4_mmps: {v4_mmps:.3f} m/s,")

        v1 = v1_mmps / 1000.0
        v2 = v2_mmps / 1000.0
        v3 = v3_mmps / 1000.0
        v4 = v4_mmps / 1000.0

        # Momentane Mecanum kinematics
        vx =    ( v1 + v3 - v4 - v2) /  4.0
        vy =    (-v1 + v3 + v4 - v2) /  4.0
        omega = (-v1 - v3 - v4 - v2) / (4.0 * self.wheelspan)

        now = self.get_clock().now().nanoseconds / 1e9
        dt = now - self.last_time
        self.last_time = now

        # Integrate pose and Normalize angle to [-pi, pi].
        self.theta += (omega * dt)
        self.theta_normalized= (self.theta + math.pi) % (2 * math.pi) - math.pi

        # Compute degrees version and Make sure it's [0, 360[
        theta_degree = (math.degrees(self.theta_normalized) + 180) % 360 - 180

        #theta_degree = math.fmod(theta_degree, 360.0)  # Normalize to [-360, 360)
        #if theta_degree < 0.0:
        #    theta_degree += 360.0  # Shift to [0, 360)


        #self.theta_normalized = (self.theta + math.pi) % (2 * math.pi) - math.pi
        self.x += ((vx * math.cos(self.theta_normalized) - vy * math.sin(self.theta_normalized)) * dt)    # in meters
        self.y += ((vx * math.sin(self.theta_normalized) + vy * math.cos(self.theta_normalized)) * dt)    # in meters
        
        

        # Create the message
        odom_deg_msg =  Float32MultiArray()
        odom_deg_msg.data = [self.x, self.y, theta_degree]

        # Publish it for debugging
        self.odom_deg_pub.publish(odom_deg_msg)

        # Quaternion for rotation
        q = quaternion_from_euler(0, 0, self.theta_normalized)

        # Publish odometry message
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = omega

        self.odom_pub.publish(odom)

        # Broadcast TF transform
        tf_msg = TransformStamped()
        tf_msg.header.stamp = odom.header.stamp
        tf_msg.header.frame_id = "odom"
        tf_msg.child_frame_id = "base_link"

        tf_msg.transform.translation.x = self.x
        tf_msg.transform.translation.y = self.y
        tf_msg.transform.translation.z = 0.0
        tf_msg.transform.rotation = odom.pose.pose.orientation

        self.tf_broadcaster.sendTransform(tf_msg)

        # Create reset service
        self.reset_odom_srv = self.create_service(Empty, 'reset_odom', self.reset_odom_callback)

    def reset_odom_callback(self, request, response):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.get_logger().info("Odometry has been reset.")
        return response
    
def main(args=None):
    rclpy.init(args=args)
    node = NexusOdomNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
