#!/usr/bin/env python3

# Import ROS 2 Python client library, message type for velocity commands and time module
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import signal
import time
import sys

class NexusMoveNode(Node):
    def __init__(self):
        # Initialize the node with the name 'nexus_move_node'
        super().__init__('nexus_move_node')

        # To access and modify the global variables within the class methods 

        # Parameters that can be passed by the user: 
        # vx         = forward speed (+vx => forward), vy = sideway speed (+vy => sideway l), w = rotation around z-axis (+w => counter clockwise),
        # t_straight = moving time in straight lines (forward, backward, sideways or diagonals), t_rotate = rotation time (see square_callback function)
        # M          = Mode of movement (l = line, s = square and r = rotation on place)
        self.declare_parameters('', [
            ('vx', 0.0), ('vy', 0.0), ('w', 0.0),
            ('t_straight', 1.0), ('t_rotate', 1.0), ('M', 'l')
        ])
        self.vx         = self.clamp(self.get_parameter('vx').value)     # Get the param vx and set it within the allowed range [-1,1] => to not allow input of extreme values
        self.vy         = self.clamp(self.get_parameter('vy').value)     # Get the param vy and set it within the allowed range [-1,1] => to not allow input of extreme values
        self.w          = self.clamp(self.get_parameter('w').value)      # Get the param w  and set it within the allowed range [-1,1] => to not allow input of extreme values
        self.t_straight = self.get_parameter('t_straight').value         # Get the param t_straight
        self.t_rotate   = self.get_parameter('t_rotate').value           # Get the param t_rotate
        self.mode       = self.get_parameter('M').value.lower()          # Get the param M (Mode of movement) and convert uppercases chars to lowercases

        self.publisher_ = self.create_publisher(Twist, '/mecanum4WD_cmd_vel', 10)    # Publisher to publish the needed speed to cmd_vel
        self.timer_freq = 0.03                                           # The frequency in Hz in which the mode functions should send a new speed command to cmd_vel topic
        self.ramp_steps = 0.1                                            # The steps in which each cmd_vel command should increment => to not cause a rapid change in speed e.g. from -1 directly to 1 (safety for motors)
        self.current_vx = self.current_vy = self.current_w = 0.0         # The speeds that will be sent to the cmd_vel
        #self.step = 0

        # Map movement modes to their corresponding callback functions
        valid_modes = {'s': self.square_callback, 'r': self.rotate_callback, 'l': self.line_callback}

        # Validate the selected mode by user; shutdown if invalid
        if self.mode not in valid_modes: 
            self.get_logger().error(f"Invalid mode '{self.mode}'. Must be one of: {list(valid_modes.keys())}")
            rclpy.shutdown()
            return
        
        ## Start the timer and assign the appropriate callback based on the selected mode
        self.timer = self.create_timer(self.timer_freq, valid_modes[self.mode])

        # Debug message
        self.get_logger().info(f"Started in mode '{self.mode}'.")

    # Function to clamp a value between min_val and max_val (default: -1.0 to 1.0)
    def clamp(self, val, min_val=-1.0, max_val=1.0): 
        return max(min(val, max_val), min_val)

    # Function to smoothly ramp the current value toward the target value by at most ramp_steps per call
    def ramp(self, current, target):
        diff = target - current
        return target if abs(diff) <= self.ramp_steps else current + self.ramp_steps * (diff / abs(diff))

    # Function to ramp all velocity components and publish the resulting Twist message
    def ramp_and_publish(self, target_vx=0.0, target_vy=0.0, target_w=0.0): 
        self.current_vx = self.ramp(self.current_vx, target_vx)
        self.current_vy = self.ramp(self.current_vy, target_vy)
        self.current_w  = self.ramp(self.current_w,  target_w)

        twist           = Twist()
        twist.linear.x  = self.current_vx
        twist.linear.y  = self.current_vy
        twist.angular.z = self.current_w
        try:
            self.publisher_.publish(twist)
            self.get_logger().info(f"vx: {twist.linear.x:.2f}, vy:{twist.linear.y:.2f}: and w:{twist.angular.z:.2f}")
        except Exception as e:
            self.get_logger().error(f"Failed to publish stop message: {e}")

    # Callback for moving in a straight line for t_straight seconds
    def line_callback(self):
        # Check if the start_time attribute exists; if not, initialize it
        # This is used to track the elapsed time since the start of the movement
        if not hasattr(self, 'start_time'): 
            self.start_time = time.time()
            self.get_logger().info(f"Initialized start_time: {self.start_time}")
        
        elapsed = time.time() - self.start_time
        self.get_logger().info(f"Elapsed: {elapsed:.2f} / {self.t_straight}")
        

        # Check if the elapsed time is less than t_straight seconds otherwise stop the robot and exit
        if elapsed < self.t_straight:   
            self.ramp_and_publish(self.vx, self.vy, 0.0)
        else: 
            self.stop_and_exit()
            return
        
    # Callback for rotating in place for t_rotate seconds (rotation around z-axis)
    def rotate_callback(self):
        if not hasattr(self, 'start_time'): 
            self.start_time = time.time()

        elapsed = time.time() - self.start_time
        self.get_logger().info(f"Elapsed: {elapsed:.2f} / {self.t_straight}")

        if elapsed < self.t_rotate: 
            self.ramp_and_publish(0.0, 0.0, self.w)
        else: 
            self.stop_and_exit()
            return

    # Callback for moving in a square pattern: move straight, then rotate, repeat 4 times
    def square_callback(self): 
        # Static attribute to track the current steps (side of the square)
        if  not hasattr(self, "step"): 
                self.step = 0
                self.start_time = time.time()

        elapsed = time.time() - self.start_time
        self.get_logger().info(f"Elapsed: {elapsed:.2f} / {self.t_straight}, Step: {self.step} / 4")
        # After 4 sides, stop the robot
        if self.step >= 4:
           self.stop_and_exit()
           return

        # Move straight for t_straight seconds
        if time.time() - self.start_time < self.t_straight: 
            self.ramp_and_publish(self.vx, self.vy, 0.0)
        # Rotate for t_rotate seconds
        elif time.time() - self.start_time < self.t_straight + self.t_rotate: 
            self.ramp_and_publish(0.0, 0.0, self.w)
        # After finishing both, increment step and reset timer
        else: 
            self.step += 1
            self.start_time            = time.time()
            self.ramp_and_publish(0.0, 0.0, 0.0)

    # Stop the robot, publish zero velocities, and destroy the timer
    def stop_and_exit(self): 
        twist           = Twist()
        twist.linear.x  = 0.0
        twist.linear.y  = 0.0
        twist.angular.z = 0.0
        self.publisher_.publish(twist)
        self.get_logger().info("Motion complete. Stopping robot.")
        self.destroy_timer(self.timer)
        
        # Gracefully shut down everything (optional if you're done)
        self.get_logger().info("Shutting down node...")
        self.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

# Main entry point for the node
def main(args=None):
    rclpy.init(args=args)           # Initialize ROS 2 Python client
    node = NexusMoveNode()          # Create the node

    # Custom SIGINT handler (to able sending 0,0,0 to cmd_vel topic)
    # This function will be called when the node receives a SIGINT signal (e.g., when Ctrl+C is entered in terminal)
    # It stops the robot first and then shuts down the node and sys gracefully
    def signal_handler(sig, frame):
        node.get_logger().info("SIGINT received. Stopping robot and shutting down...")
        node.stop_and_exit()
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

    # Register the signal handler for SIGINT (Ctrl+C)
    signal.signal(signal.SIGINT, signal_handler)

    rclpy.spin(node)            # Keep the node alive and processing callbacks


# Run the main function if this script is executed directly
if __name__ == '__main__':
    main()
