#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
import time
from std_msgs.msg import Int32MultiArray, String

class NexusSerialConnNode(Node):
    def __init__(self):
        super().__init__('nexus_serial_conn_node')

        # Serial connection to Arduino
        self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)

        # Debug log topic
        self.serial_debug_pub = self.create_publisher(String, '/serial_debug', 10)

        self._log_debug("Waiting for Arduino to boot...")
        time.sleep(0.5)
        self._log_debug("Starting communication.")

        # Initial wheel values
        self.v = [0, 0, 0, 0]

        # ROS interfaces
        self.subscription = self.create_subscription(
            Int32MultiArray, '/twist_nexus', self.twist_nexus_callback, 10
        )

        self.encoder_pub = self.create_publisher(
            Int32MultiArray, '/encoder_wheels', 10
        )

        # Main loop at 20 Hz
        self.timer = self.create_timer(0.01, self.loop)

    def _log_debug(self, message: str):
        msg = String()
        msg.data = message
        self.serial_debug_pub.publish(msg)

    def twist_nexus_callback(self, msg):
        if len(msg.data) == 4:
            self.v = msg.data

    def compute_checksum(self, payload: str) -> int:
        checksum = 0
        for c in payload:
            checksum ^= ord(c)
        return checksum

    def generate_twist_message(self, v1, v2, v3, v4) -> str:
        payload = f"{v1},{v2},{v3},{v4}"
        checksum = self.compute_checksum(payload)
        return f"@{payload}*{checksum}#\n"

    def parse_encoder_message(self, line: str):
        if not (line.startswith('%') and line.endswith('!')):
            self._log_debug(f"[INVALID FRAME] {line}")
            return None

        content = line[1:-1]  # strip % and !
        if '*' not in content:
            self._log_debug(f"[NO CHECKSUM] {line}")
            return None

        payload, checksum_str = content.rsplit('*', 1)
        try:
            received_checksum = int(checksum_str)
        except ValueError:
            self._log_debug(f"[BAD CHECKSUM FORMAT] {checksum_str}")
            return None

        computed_checksum = self.compute_checksum(payload)
        if received_checksum != computed_checksum:
            self._log_debug(f"[CHECKSUM MISMATCH] Computed {computed_checksum} != Received {received_checksum}")
            return None

        if "|" not in payload:
            self._log_debug(f"[MALFORMED PAYLOAD] {payload}")
            return None

        twist_data, encoder_data = payload.split("|")
        try:
            t1, t2, t3, t4 = map(int, twist_data.split(','))
            e1, e2, e3, e4 = map(int, encoder_data.split(','))
        except ValueError:
            self._log_debug(f"[BAD DATA FORMAT] {payload}")
            return None

        return (t1, t2, t3, t4, e1, e2, e3, e4)

    def loop(self):
        # 1. Send twist command
        v1, v2, v3, v4 = self.v
        serial_message = self.generate_twist_message(v1, v2, v3, v4)
        self.ser.write(serial_message.encode('utf-8'))

        # 2. Read feedback from Arduino
        while self.ser.in_waiting:
            try:
                line = self.ser.readline().decode('utf-8', errors='replace').strip()

                if "�" in line:
                    self._log_debug(f"[CORRUPT UTF-8] {line}")
                    continue

                parsed = self.parse_encoder_message(line)
                if parsed is None:
                    continue

                t1, t2, t3, t4, e1, e2, e3, e4 = parsed

                self._log_debug(
                    f"[TWIST SENT] {t1}, {t2}, {t3}, {t4} | [ENCODER] {e1}, {e2}, {e3}, {e4}"
                )

                encoder_msg = Int32MultiArray()
                encoder_msg.data = [e1, e2, e3, e4]
                self.encoder_pub.publish(encoder_msg)

            except Exception as e:
                self._log_debug(f"[PARSE ERROR] {e}")

def main(args=None):
    rclpy.init(args=args)
    node = NexusSerialConnNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
