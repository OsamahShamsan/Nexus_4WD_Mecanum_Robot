import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
import math

class SendGoal(Node):
    def __init__(self):
        super().__init__('send_goal')
        self.navigator = BasicNavigator()
        self.navigator.waitUntilNav2Active()
        self.send_goal()

    def send_goal(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.navigator.get_clock().now().to_msg()
        goal.pose.position.x = 1.0
        goal.pose.position.y = 0.5
        yaw = math.radians(90)
        goal.pose.orientation.z = math.sin(yaw / 2)
        goal.pose.orientation.w = math.cos(yaw / 2)

        self.get_logger().info('Sending goal...')
        self.navigator.goToPose(goal)

        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            self.get_logger().info(f'Feedback: {feedback}')

        result = self.navigator.getResult()
        self.get_logger().info(f'Result: {result}')

def main(args=None):
    rclpy.init(args=args)
    node = SendGoal()
    rclpy.spin_once(node, timeout_sec=0)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
