#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
import time
from std_msgs.msg import Int32MultiArray, String

class NexusSerialConnNode(Node):
    def __init__(self):
        super().__init__('nexus_serial_conn_node')

        # Initialize serial connection with settings matching Arduino:
        # - Baudrate: 115200
        # - Data bits: 8
        # - Parity: None
        # - Stop bits: 1
        # - Flow control: None
        # - Timeout: 1 second
        self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)   # These must exactly match Serial.begin(...) settings on Arduino

        # Ensure the port is open
        if not self.ser.is_open:
            self.ser.open()

        # Clear any old data before starting communication
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

        # Log useful serial configuration info
        self.get_logger().info("Serial port opened successfully.")
        self.get_logger().info(f"Port: {self.ser.name}")
        self.get_logger().info(f"Baudrate: {self.ser.baudrate}, Data bits: {self.ser.bytesize}, Parity: {self.ser.parity}, Stop bits: {self.ser.stopbits}")
        self.get_logger().info("Serial buffers flushed and ready.")

        # ROS interfaces        
        self.serial_debug_pub = self.create_publisher(String, '/serial_debug', 10)          # Debug log topic
        self.subscription = self.create_subscription( Int32MultiArray, '/twist_nexus', self.twist_nexus_callback, 10 )
        self.encoder_pub = self.create_publisher( Int32MultiArray, '/encoder_wheels', 10 )
        
        self._log_debug("Waiting for Arduino to boot...")
        time.sleep(0.5)
        self._log_debug("Starting communication.")

        # Initial wheel values
        self.v = [0, 0, 0, 0]
        self.counter = 0
        self.success_rate = 100  # Success Rate optional default 

        self.timer = self.create_timer(0.01, self.loop)         # Main loop at 20 Hz


    def twist_nexus_callback(self, msg):
        if len(msg.data) == 4:
            self.v = msg.data

    def _log_debug(self, message: str):
        msg = String()
        msg.data = message
        self.serial_debug_pub.publish(msg)
        

    def compute_crc8(self, payload: str) -> int:
        crc = 0x00
        for byte in payload.encode():
            crc ^= byte
            for _ in range(8):
                crc = ((crc << 1) ^ 0x07) if (crc & 0x80) else (crc << 1)
                crc &= 0xFF
        return crc

    def build_twist_msg(self, v1_UL, v2_UR, v3_LL, v4_LR):
        # ---------------------<T:counter,v1_UL,v2_UR,v3_LL,v4_LR*CRC>---------------------------       
        # ---------------------<T:  42   ,120  ,115  ,-100 ,-105 *CRC>--------------------------- 
        payload = f"{self.counter},{v1_UL},{v2_UR},{v3_LL},{v4_LR}"
        crc = self.compute_crc8(payload)
        msg = f"<T:{payload}*{crc}>\n"
        return msg


    def parse_encoder_message(self, line: str):
        if not (line.startswith('<E:') and line.endswith('>')):
            return None

        content = line[3:-1]
        if '*' not in content or '|' not in content:
            self._log_debug(f"[INVALID FORMAT] {line}")
            return None

        try:
            payload, crc_str = content.rsplit('*', 1)
            received_crc = int(crc_str)
        except ValueError:
            self._log_debug(f"[BAD CRC FORMAT] {line}")
            return None

        if self.compute_crc8(payload) != received_crc:
            self._log_debug(f"[CRC MISMATCH] {payload}")
            return None

        data_part, sr_str = payload.split('|')
        try:
            id_str, enc_v1_UL, enc_v2_UR, enc_v3_LL, enc_v4_LR = data_part.split(',')
            msg_counter = int(id_str)
            enc_vals = list(map(int, [enc_v1_UL, enc_v2_UR, enc_v3_LL, enc_v4_LR]))
            Success_Rate = int(sr_str)
            return (msg_counter, enc_vals, Success_Rate)
        except Exception as e:
            self._log_debug(f"[PARSE ERROR] {e}")
            return None


    def loop(self):
        v1_UL, v2_UR, v3_LL, v4_LR = self.v
        tx_msg = self.build_twist_msg(v1_UL, v2_UR, v3_LL, v4_LR)
        self.ser.write(tx_msg.encode('utf-8'))

        #self.get_logger().info(f"[TX: {tx_msg}]")

        self.counter = (self.counter + 1) % 256

        while self.ser.in_waiting:
            try:
                line = self.ser.readline().decode('utf-8', errors='replace').strip()
                self.get_logger().info(f"[RX: {line}]")
                if "�" in line or not line:
                    self.get_logger().info(f"[CORRUPT] {line}")
                    continue

                parsed = self.parse_encoder_message(line)
                if parsed is None:
                    continue

                rx_counter, encoders, success_rate = parsed
                payload = f"{rx_counter},{','.join(map(str, encoders))}|{success_rate}"
                    
                # Print both messages
                #self.get_logger().info(f"[TX: {tx_msg}]  |  [RX: {line}]")

                # Publish encoders
                encoder_msg = Int32MultiArray()
                encoder_msg.data = encoders
                self.encoder_pub.publish(encoder_msg)

            except Exception as e:
                self.get_logger().info(f"[LOOP ERROR] {e}")


def main(args=None):
    rclpy.init(args=args)
    node = NexusSerialConnNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
