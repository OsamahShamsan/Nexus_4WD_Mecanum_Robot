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


import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray

class TwistNexusNode(Node):
    def __init__(self):
        super().__init__('nexus_twist_node')

        # Constants
        self.WHEELSPAN = 300         
        self.MAX_SPEEDRPM = 8000     
        self.REDUCTION_RATIO = 64    
        self.CIRMM = 314             
        self.MAX_SPEED = 500.0
        self.MIN_SPEED = -500.0

        self.declare_parameter('linear_scale', self.MAX_SPEED) 
        self.declare_parameter('angular_scale', 1.0)
        self.declare_parameter('linear_ramp_step', 20.0)
        self.declare_parameter('angular_ramp_step', 0.1)

        self.linear_scale = self.get_parameter('linear_scale').value
        self.angular_scale = self.get_parameter('angular_scale').value
        self.linear_ramp_step = self.get_parameter('linear_ramp_step').value
        self.angular_ramp_step = self.get_parameter('angular_ramp_step').value

        self.publisher = self.create_publisher(Int32MultiArray, '/twist_nexus', 10)
        #self.publisher = self.create_publisher(Int32MultiArray, '/encoder_wheels', 10)

        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        self.timer = self.create_timer(0.01, self.loop)  # Main loop 50 Hz

        self.target_vx = 0.0
        self.target_vy = 0.0
        self.target_omega = 0.0

        self.vx = 0.0
        self.vy = 0.0
        self.w = 0.0

    def cmd_vel_callback(self, msg):
        self.target_vx = msg.linear.x * self.linear_scale
        self.target_vy = msg.linear.y * self.linear_scale
        self.target_omega = msg.angular.z * self.angular_scale

    # Function to not allow values below -1 and over 1
    def clamp(self, val, min_val, max_val):
        return max(min(val, max_val), min_val)
    
    def ramp(self, current, target, step):
        diff = target - current
        if abs(diff) <= step:
            return target
        return current + step * (diff / abs(diff))

    def loop(self):
        publishers = self.subscription.get_publisher_count()
        msg = Int32MultiArray()
        
        if publishers == 0:
            #self.get_logger().info(f"1 Number of publishers on {self.input_topic}: {publishers}")
            self.vx = self.ramp(self.vx, 0, self.linear_ramp_step)
            self.vy = self.ramp(self.vy, 0, self.linear_ramp_step)
            self.w  = self.ramp(self.w, 0, self.angular_ramp_step)
        else:
            self.vx = self.clamp(self.ramp(self.vx, self.target_vx, self.linear_ramp_step), -500, 500)
            self.vy = self.clamp(self.ramp(self.vy, self.target_vy, self.linear_ramp_step), -500, 500)
            self.w  = self.clamp(self.ramp(self.w, self.target_omega, self.angular_ramp_step), -1, 1)

        r = self.WHEELSPAN

        # Since we are already getting vx and vy we dont need to multiply v*sin(rad) and v*cos(rad) because v*sin(rad) = vy and v*cos(rad) = vx
        v1_UL = int( ( - self.vy + self.vx - (r * self.w)))
        v2_LL = int( ( - self.vy - self.vx - (r * self.w)))
        v3_LR = int( ( + self.vy + self.vx - (r * self.w)))
        v4_UR = int( ( + self.vy - self.vx - (r * self.w)))
        
        msg.data = [v1_UL, v2_LL, v3_LR, v4_UR]

        self.publisher.publish(msg)
        #self.get_logger().info(f"2 Number of publishers on {self.input_topic}: {publishers}")

        

def main(args=None):
    rclpy.init(args=args)
    node = TwistNexusNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
