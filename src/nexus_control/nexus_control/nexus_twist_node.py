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


# Import required ROS 2 and message modules
import rclpy  # Core ROS 2 Python library
from rclpy.node import Node  # Base class for creating ROS 2 nodes
from geometry_msgs.msg import Twist  # Message type for velocity commands (cmd_vel)
from std_msgs.msg import Int32MultiArray  # Message type for publishing wheel velocities

# Define a custom node class that translates Twist messages into wheel velocities
class TwistNexusNode(Node):
    def __init__(self):
        # Initialize the node with a unique name
        super().__init__('nexus_twist_node')

        # Constants representing the physical characteristics of the robot
        self.WHEELSPAN = 300          # Distance between wheels (in mm)
        self.MAX_SPEEDRPM = 8000      # Max motor RPM (from datasheet)
        self.REDUCTION_RATIO = 64     # Gear reduction ratio
        self.CIRMM = 314              # Wheel circumference in mm
        self.MAX_SPEED = 500.0        # Max safe speed (mm/s)
        self.MIN_SPEED = -500.0       # Min safe speed (mm/s)

        # Declare configurable parameters with default values
        self.declare_parameter('linear_scale', self.MAX_SPEED)     # Scale factor for linear velocity
        self.declare_parameter('angular_scale', 1.0)               # Scale factor for angular velocity
        self.declare_parameter('linear_ramp_step', 20.0)           # Max change per loop for linear vel
        self.declare_parameter('angular_ramp_step', 0.1)           # Max change per loop for angular vel

        # Get parameter values
        self.linear_scale = self.get_parameter('linear_scale').value
        self.angular_scale = self.get_parameter('angular_scale').value
        self.linear_ramp_step = self.get_parameter('linear_ramp_step').value
        self.angular_ramp_step = self.get_parameter('angular_ramp_step').value

        # Publisher that sends motor commands (4 wheel speeds)
        self.publisher = self.create_publisher(Int32MultiArray, '/twist_nexus', 10)

        # Subscription to the cmd_vel topic to receive robot velocity commands
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Timer for the control loop (called every 10ms → 100Hz)
        self.timer = self.create_timer(0.01, self.loop)

        # Target velocities from Twist messages
        self.target_vx = 0.0  # Target linear velocity in x
        self.target_vy = 0.0  # Target linear velocity in y
        self.target_omega = 0.0  # Target angular velocity around z

        # Current (ramped) velocities used to prevent abrupt changes
        self.vx = 0.0
        self.vy = 0.0
        self.w = 0.0

    # Callback function triggered when a Twist message is received
    def cmd_vel_callback(self, msg):
        # Scale received linear and angular velocities
        self.target_vx = msg.linear.x * self.linear_scale
        self.target_vy = msg.linear.y * self.linear_scale
        self.target_omega = msg.angular.z * self.angular_scale

    # Clamp a value to a given range
    def clamp(self, val, min_val, max_val):
        return max(min(val, max_val), min_val)
    
    # Ramp the current value toward the target by at most 'step' units
    def ramp(self, current, target, step):
        diff = target - current
        if abs(diff) <= step:
            return target
        return current + step * (diff / abs(diff))  # Increment in the direction of the target

    # Main control loop called periodically by the timer
    def loop(self):
        # Get the number of publishers on the /cmd_vel topic
        publishers = self.subscription.get_publisher_count()

        # Create a message for publishing motor speeds
        msg = Int32MultiArray()
        
        if publishers == 0:
            # If no /cmd_vel input, gradually ramp to zero (safe stop)
            self.vx = self.ramp(self.vx, 0, self.linear_ramp_step)
            self.vy = self.ramp(self.vy, 0, self.linear_ramp_step)
            self.w  = self.ramp(self.w, 0, self.angular_ramp_step)
        else:
            # Ramp toward the target velocities and clamp within safe limits
            self.vx = self.clamp(self.ramp(self.vx, self.target_vx, self.linear_ramp_step), -500, 500)
            self.vy = self.clamp(self.ramp(self.vy, self.target_vy, self.linear_ramp_step), -500, 500)
            self.w  = self.clamp(self.ramp(self.w, self.target_omega, self.angular_ramp_step), -1, 1)

        # Use wheel kinematics to calculate individual wheel speeds
        r = self.WHEELSPAN  # distance factor (can be interpreted as (a + b) in kinematics)

        # Mecanum kinematic equations (adjusted to match wheel orientation)
        v1_UL = int(-self.vy + self.vx - (r * self.w))  # Upper Left
        v2_LL = int(-self.vy - self.vx - (r * self.w))  # Lower Left
        v3_LR = int(+self.vy + self.vx - (r * self.w))  # Lower Right
        v4_UR = int(+self.vy - self.vx - (r * self.w))  # Upper Right
        
        # Pack wheel speeds into message and publish
        msg.data = [v1_UL, v2_LL, v3_LR, v4_UR]
        self.publisher.publish(msg)

# Main function that initializes and runs the node
def main(args=None):
    rclpy.init(args=args)        # Initialize ROS 2 communication
    node = TwistNexusNode()      # Create an instance of the node
    rclpy.spin(node)             # Keep the node running
    node.destroy_node()          # Cleanup when done
    rclpy.shutdown()             # Shutdown ROS 2 client

# Entry point if this file is run directly
if __name__ == '__main__':
    main()
