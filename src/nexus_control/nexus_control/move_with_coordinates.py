import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
import math

class MoveWithCoordinates(Node):
    def __init__(self):
        super().__init__('move_with_coordinates')

        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('yaw', 0.0)

        self.x = self.get_parameter('x').value
        self.y = self.get_parameter('y').value
        self.yaw = self.get_parameter('yaw').value

        self.navigator = BasicNavigator()
        self.navigator.waitUntilNav2Active()

        self.send_goal(self.x, self.y, self.yaw)

    def send_goal(self, x, y, theta_deg):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.navigator.get_clock().now().to_msg()

        goal.pose.position.x = x
        goal.pose.position.y = y

        yaw = math.radians(theta_deg)
        goal.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.orientation.w = math.cos(yaw / 2.0)

        self.get_logger().info(f"Sending goal to ({x}, {y}, {theta_deg}°)")
        self.navigator.goToPose(goal)

        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            self.get_logger().info(f"Feedback: {feedback}")

        result = self.navigator.getResult()
        if result == self.navigator.TaskResult.SUCCEEDED:
            self.get_logger().info("Goal succeeded!")
        else:
            self.get_logger().warn("Goal failed.")

def main(args=None):
    rclpy.init(args=args)
    node = MoveWithCoordinates()
    rclpy.spin_once(node, timeout_sec=0)  # Optional spin if you want logs/services to work
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
