import rclpy
from rclpy.node import Node
import serial
import time

class SerialReaderNode(Node):
    def __init__(self):
        super().__init__('serial_reader_node')

        # Initialize the serial connection
        self.serial_port = '/dev/ttyUSB0'  # Adjust based on your setup
        self.baud_rate = 115200
        self.ser = serial.Serial(self.serial_port, self.baud_rate)

        # Timer to periodically read from serial
        self.timer = self.create_timer(0.1, self.read_serial)

    def read_serial(self):
        try:
            if self.ser.in_waiting > 0:  # Check if data is available
                line = self.ser.readline().decode('utf-8').strip()  # Read and decode the line
                self.get_logger().info(f"Raw data: {line}")
                
                # Split the line into float values
                data = line.split(',')
                if len(data) == 3:
                    var1, var2, var3 = map(float, data)
                    self.get_logger().info(f"Received: var1={var1}, var2={var2}, var3={var3}")
                else:
                    self.get_logger().warning("Unexpected data format")
                time.sleep(0.005)  # This is equivalent to a "delay" 

        except Exception as e:
            self.get_logger().error(f"Error reading serial: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = SerialReaderNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
