#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
import time

#define width 150      // mm
#define length 153     // mm
#define total (width + length)
#define R 50           // mm

class SerialConnNode(Node):
    def __init__(self):
        super().__init__('serial_conn_node')
        self.ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
        self.timer = self.create_timer(1.0, self.loop)

    def loop(self):
        self.ser.write(b"Hello from Raspberry Pi!\n")
        #line = self.ser.readline().decode('utf-8').rstrip()
        line = str(self.ser.readline())
        #line = self.ser.readline().decode('utf-8', errors='ignore').strip()
        self.get_logger().info(f"Received: {line}")

def main(args=None):
    rclpy.init(args=args)
    node = SerialConnNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
