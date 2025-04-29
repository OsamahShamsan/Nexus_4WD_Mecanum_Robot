# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist
# from nav_msgs.msg import Odometry
# from std_msgs.msg import String
# import time
# import serial

# class CmdVelToSerialNode(Node):
#     def __init__(self):
#         super().__init__('cmd_vel_to_serial')
        
#         self.frequency_ = 0.01
        
#         self.odom = Odometry()
        
#         # Declare parameters
#         self.declare_parameter('cmd_topic', '/cmd_vel')
#         self.declare_parameter('twist_output', '/scaled_cmd_vel')
#         self.declare_parameter('odom_output', '/robot/odom')
#         self.declare_parameter('serial_port', '/dev/ttyUSB0')
#         self.declare_parameter('baud_rate', 115200)
#         self.declare_parameter('linear_scale', 500)
#         self.declare_parameter('angular_scale', 1)

#         # Get parameter values
#         self.cmd_topic = self.get_parameter('cmd_topic').value
#         self.twist_output = self.get_parameter('twist_output').value
#         self.odom_output = self.get_parameter('odom_output').value
#         self.serial_port = self.get_parameter('serial_port').value
#         self.baud_rate = self.get_parameter('baud_rate').value
#         self.linear_scale = self.get_parameter('linear_scale').value
#         self.angular_scale = self.get_parameter('angular_scale').value

#         self.timer_ = self.create_timer(self.frequency_, self.timerCallback)
#         self.twist_pub_ = self.create_publisher(Twist, self.twist_output, 10)
#         #self.pub_ = self.create_publisher(String, self.odom_output, 10)
        
#         # Initialize serial connection
#         try:
#             self.serial_conn = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
#             #time.sleep(2)  # Give Arduino time to reset
#             self.get_logger().info(f"Connected to Arduino on {self.serial_port} at {self.baud_rate} baud.")
#         except serial.SerialException as e:
#             self.get_logger().error(f"Failed to connect to serial port: {e}")
#             exit(1)
        
#         # Subscribe to input topic
#         self.subscription = self.create_subscription(Twist, self.cmd_topic, self.cmd_vel_callback, 10)
    
#     def timerCallback(self):
#         """
#         if rclpy.ok() and self.serial_conn.is_open:
#             data = self.serial_conn.readline()
#             if not data:  # Skip if no data
#                 return
#             try:
#                 decoded_data = data.decode("utf-8").strip()
#                 msg = String()
#                 msg.data = decoded_data
#                 self.pub_.publish(msg)
#             except UnicodeDecodeError:
#                 self.get_logger().warn("Failed to decode serial data.")  
#         """
        
#     def cmd_vel_callback(self, msg):
#         # Scale the Twist message
#         scaled_msg = Twist()
#         scaled_msg.linear.x = msg.linear.x * self.linear_scale
#         scaled_msg.linear.y = msg.linear.y * self.linear_scale
#         scaled_msg.angular.z = msg.angular.z * self.angular_scale

#         # Serialize the Twist message and send it to Arduino
#         serial_message = f"{scaled_msg.linear.x},{scaled_msg.linear.y},{scaled_msg.angular.z}\n"
#         self.serial_conn.write(serial_message.encode('utf-8'))
#         self.get_logger().info(f"Sent to Arduino: {serial_message.strip()}")

#         # Publish the scaled Twist message
#         self.twist_pub_.publish(scaled_msg)
#         self.get_logger().info(
#             f"Published scaled Twist: linear=({scaled_msg.linear.x}, {scaled_msg.linear.y}, {scaled_msg.linear.z}), "
#             f"angular=({scaled_msg.angular.x}, {scaled_msg.angular.y}, {scaled_msg.angular.z})"
#         )
#         time.sleep(0.05)  # This is equivalent to a "delay" 


# def main():
#     rclpy.init()
#     node = CmdVelToSerialNode()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         node.get_logger().info("Node stopped cleanly.")
#     finally:
#         if hasattr(node, 'serial_conn') and node.serial_conn.is_open:
#             node.serial_conn.close()
#             node.get_logger().info("Serial connection closed.")
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()
