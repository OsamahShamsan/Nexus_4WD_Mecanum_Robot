#!/usr/bin/env python3

# The documntation in this node and constants are based on:
# 1- NEXUS ROBOT User Manual (https://openhacks.com/uploadsproductos/file-1342627200__1_.pdf)
# 2- Omni4WD and MotorWheel library (https://github.com/lupusorina/nexus-robots/tree/master/documentation-libraries/lib/MotorWheel)

# Constants for the robot
# These values are based on the robot's physical dimensions and motor specifications
# WHEELSPAN = 300         # mm
# MAX_SPEEDRPM = 8000     # RPM
# REDUCTION_RATIO = 64    # 
# CIRMM = 314             # mm
# MAX_SPEED = 500.0       # mm/s (for safety use  500 mm/s)
# MIN_SPEED = -500.0      # mm/s (for safety use -500 mm/s)

#  Mecanum4WD Top View (See position of Power Switch for reference)
#  From the top view of the robot you can't see the arduino board.
#  
#  
#    			     Front MOTORS_FB
#                  ------------------
#                 |                  |
#    wheel1_UL // |		             | \\ wheel4_UR	
#                  ------------------
#                  ------------------
#                 |                  |          
#  (Power Switch) |                  |
#                 |     Top View     |
#                 |                  |
#                 |  (Arduino board  |        
#    wheel2_LL \\ |    not visible)  | // wheel3_LR
#                 |                  |  
#                  ------------------
#    			     Back MOTORS_BF
                                          

#  Mecanum4WD Bottom View (See position of Power Switch for reference)
#  Here you can see the arduino board.
#  
#  
#  			       Front MOTORS_FB
#                ------------------
#               |                  |
#  wheel4_UR \\ |		           | // wheel1_UL	
#                ------------------
#                ------------------
#               |                  |          
#               |                  | (Power Switch)
#               |   Bottom View    |
#               |                  |
#               |  (Arduino board  |        
#  wheel3_LR // |     visible)     | \\ wheel2_LL
#               |                  |  
#                ------------------
#  			       Back MOTORS_BF
 
#  Initialize serial communication with:
# - Baudrate: 115200  (For Arduino 328: Baudrates between 300 and 2800000 were tested and they worked. 
#                      Baudrates that did not work in the tests 100, 200, 2850000 and 2900000).
# - Data bits: 8
# - Parity: None
# - Stop bits: 1
# - Flow control: None (default for Arduino Serial)


# To find the max speed of the robot (see MotorWheel.h):
# MAX_SPEEDRPM 8000 => see MotorWheel.h
# REDUCTION_RATIO 64
# CIRMM 314
# =>  Output speed = 8000 / 64 = 125 RPM       
# =>  125 RPM × 314 mm = 39,250 mm/min
# =>  39,250 / 60 ≈ 654 mm/s
# =>  max speed for each wheel is around 654 mm/s (for safety 600 mm/s)


#  Kinematic equations for "my" 4WD mecanum robot:
#  v1_UL = -vy + vx - w*(r)     # in my case v1_UL = 4_UR in the library
#  v2_LL = -vy - vx - w*(r)     # in my case v2_LL = 3_LR in the library
#  v3_LR = +vy + vx - w*(r)     # in my case v3_LR = 1_UL in the library
#  v4_UR = +vy - vx - w*(r)     # in my case v4_UR = 2_LL in the library
#  
#  Kinematic equations for the 4WD mecanum robot from the Omni4WD library (See UNCOMMENTED section in Omni4WD::setCarMove):
#  v_UL=Vty+Vtx-w(a+b);
#  v_LL=Vty-Vtx-w(a+b);
#  v_LR=Vty+Vtx+w(a+b);
#  v_UR=Vty-Vtx+w(a+b);

# Motors and Encoders are connected to the Arduino board as follows:
# MotorWheel(_pinPWM, _pinDir, _pinIRQ, _pinIRQB, _isr, ratio=REDUCTION_RATIO, cirMM=CIRMM);
# Motor PWM:Pin3, DIR:Pin2, Encoder A:Pin4, B:Pin5
# irqISR(irq1, isr1);
# MotorWheel wheel1_UL(3, 2, 4, 5, &irq1);
# irqISR(irq2, isr2);
# MotorWheel wheel2_LL(11, 12, 14, 15, &irq2);
# irqISR(irq3, isr3);
# MotorWheel wheel3_LR(9, 8, 16, 17, &irq3);
# irqISR(irq4, isr4);
# MotorWheel wheel4_UR(10, 7, 18, 19, &irq4);

# Using the class Omni4WD(MotorWheel* wheelUL,MotorWheel* wheelLL, MotorWheel* wheelLR,MotorWheel* wheelUR,unsigned int wheelspan=WHEELSPAN);
# Omni4WD Omni(&wheel1_UL, &wheel2_LL, &wheel3_LR, &wheel4_UR);

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
import time

class TwistNexusNode(Node):
    def __init__(self):
        super().__init__('nexus_twist_node')

        # ----------------------------------------------------------
        # Parameters
        # ----------------------------------------------------------
        self.declare_parameter('input_topic', '/cmd_vel_joystick')   # or '/cmd_vel'

        # Unit system (0 = m/s, 1 = mm/s)
        self.declare_parameter('use_millimetre', 0)

        # Base scaling (for m/s and rad/s)
        self.declare_parameter('linear_scale_mps', 0.66)             # maps [-1,1] → [-0.66,0.66] m/s
        self.declare_parameter('angular_scale_radps', 2.2)           # maps [-1,1] → [-2.2,2.2] rad/s

        # Control loop behavior
        self.declare_parameter('watchdog_timeout_s', 0.30)           # stop if no cmd for this long
        self.declare_parameter('v_step_mps', 0.01)                   # velocity ramp step [m/s]
        self.declare_parameter('wz_step_radps', 0.10)                # angular ramp step [rad/s]

        # ----------------------------------------------------------
        # Read parameters
        # ----------------------------------------------------------
        self.input_topic = self.get_parameter('input_topic').value
        self.use_mm = int(self.get_parameter('use_millimetre').value)
        self.lin_scale = float(self.get_parameter('linear_scale_mps').value)
        self.ang_scale = float(self.get_parameter('angular_scale_radps').value)
        self.timeout_s = float(self.get_parameter('watchdog_timeout_s').value)
        self.v_step = float(self.get_parameter('v_step_mps').value)
        self.wz_step = float(self.get_parameter('wz_step_radps').value)

        # ----------------------------------------------------------
        # Unit conversion (automatic scaling)
        # ----------------------------------------------------------
        if self.use_mm:
            self.get_logger().info("Using millimetre units [mm/s]")
            self.lin_scale *= 1000.0
            self.v_step *= 1000.0
        else:
            self.get_logger().info("Using metre units [m/s]")

        # ----------------------------------------------------------
        # Publishers & Subscribers
        # ----------------------------------------------------------
        self.pub_cmd = self.create_publisher(Float32MultiArray, '/twist_nexus', 10)
        self.pub_cfg = self.create_publisher(Float32MultiArray, '/nexus_ctrl/config', 1)
        self.sub = self.create_subscription(Twist, self.input_topic, self.on_twist, 10)

        # ----------------------------------------------------------
        # Timers
        # ----------------------------------------------------------
        self.create_timer(0.01, self.tick_100hz)  # publish at 100 Hz
        self.create_timer(1.0, self.send_config)  # resend config each second

        # ----------------------------------------------------------
        # State
        # ----------------------------------------------------------
        self.last_twist_time = 0.0
        self.last_cmd = [0.0, 0.0, 0.0]  # [vx, vy, wz] in m/s or mm/s, depending on mode

        self.get_logger().info(f"Listening to: {self.input_topic}")
        self.get_logger().info(f"Linear scale: {self.lin_scale:.3f}, Angular scale: {self.ang_scale:.3f}")

    # ----------------------------------------------------------
    # Callbacks
    # ----------------------------------------------------------
    def on_twist(self, msg: Twist):
        # Scale joystick inputs to physical velocities
        vx = float(msg.linear.x)  * self.lin_scale
        vy = float(msg.linear.y)  * self.lin_scale
        wz = float(msg.angular.z) * self.ang_scale
        self.last_cmd = [vx, vy, wz]
        self.last_twist_time = time.monotonic()

    def send_config(self):
        # Send ramp step sizes to MCU
        m = Float32MultiArray()
        m.data = [float(self.v_step), float(self.wz_step)]
        self.pub_cfg.publish(m)

    def tick_100hz(self):
        # Safety: stop if timeout or no active input
        have_input = self.subscription_publisher_count() > 0
        timed_out = (time.monotonic() - self.last_twist_time) > self.timeout_s

        if not have_input or timed_out:
            data = [0.0, 0.0, 0.0]
        else:
            data = self.last_cmd

        m = Float32MultiArray()
        m.data = data
        self.pub_cmd.publish(m)

    def subscription_publisher_count(self) -> int:
        try:
            return self.sub.get_publisher_count()
        except Exception:
            return 1  # fallback if method not supported


def main():
    rclpy.init()
    rclpy.spin(TwistNexusNode())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
