#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
import time
from std_msgs.msg import Float32MultiArray


class SerialConnNode(Node):
    def __init__(self):
        super().__init__('serial_conn_node')

        # Serial connection to Arduino
        self.ser = serial.Serial('/dev/ttyUSB0', 250000, timeout=1)
        self.get_logger().info("Waiting for Arduino to boot...")
        time.sleep(0.5)
        self.get_logger().info("Starting communication.")

        # Initial wheel values
        self.v = [0.0, 0.0, 0.0, 0.0]

        # ROS interfaces
        self.subscription = self.create_subscription(
            Float32MultiArray, '/twist_nexus', self.twist_nexus_callback, 10
        )

        self.encoder_pub = self.create_publisher(
            Float32MultiArray, '/encoder_wheels', 10
        )

        # Main loop at 20 Hz
        self.timer = self.create_timer(0.02, self.loop)

    def twist_nexus_callback(self, msg):
        if len(msg.data) == 4:
            self.v = msg.data

    def loop(self):
        v1, v2, v3, v4 = self.v

        # Send to Arduino
        serial_message = f"{v1:.2f},{v2:.2f},{v3:.2f},{v4:.2f}\n"
        self.ser.write(serial_message.encode('utf-8'))

        # Handle Arduino feedback
        while self.ser.in_waiting:
            try:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if not line.startswith("msg"):
                    continue

                twist_data, encoder_data = line.replace("msg", "").split("|")
                t1, t2, t3, t4 = map(int, twist_data.split(','))
                e1, e2, e3, e4 = map(int, encoder_data.split(','))

                self.get_logger().info(
                    f"[TWIST SENT] {t1:d}, {t2:d}, {t3:}, {t4:d} | "
                    f"[ENCODER] {e1:d}, {e2:d}, {e3:d}, {e4:d}"
                )

                # Publish encoder values
                encoder_msg = Float32MultiArray()
                encoder_msg.data = [e1, e2, e3, e4]
                self.encoder_pub.publish(encoder_msg)

            except Exception as e:
                self.get_logger().warn(f"Serial parse error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = SerialConnNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
