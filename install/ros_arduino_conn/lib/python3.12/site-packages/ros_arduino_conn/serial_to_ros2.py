import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import serial


class SerialToROS2Node(Node):
    def __init__(self):
        super().__init__('serial_to_ros2')
        self.pub_ = self.create_publisher(String, '/robot/odom', 10)
        self.frequency_ = 0.01
        self.get_logger().info("Publishing at %d Hz" % self.frequency_)
        
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value

        # Initialize Odometry message
        self.odom = Odometry()

        self.timer_ = self.create_timer(self.frequency_, self.timerCallback)
        
        # Connect to the serial port
        try:
            self.arduino_ = serial.Serial(self.serial_port, self.baud_rate)
            self.get_logger().info(f"Connected to Arduino on {self.serial_port} at {self.baud_rate} baud.")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to serial port: {e}")
            exit(1)

    def timerCallback(self):
        if rclpy.ok() and self.arduino_.is_open:
            data = self.arduino_.readline()
            try:
                data.decode("utf-8")
            except:
                return
            msg = String()
            msg.data = str(data)
            self.pub_.publish(msg)
            


def main():
    rclpy.init()
    
    node = SerialToROS2Node()
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
