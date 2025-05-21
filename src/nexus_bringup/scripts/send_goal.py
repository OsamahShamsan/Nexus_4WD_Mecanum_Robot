from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
import rclpy
import math

def main():
    rclpy.init()
    navigator = BasicNavigator()
    navigator.waitUntilNav2Active()

    goal = PoseStamped()
    goal.header.frame_id = 'map'
    goal.header.stamp = navigator.get_clock().now().to_msg()
    goal.pose.position.x = 1.0
    goal.pose.position.y = 0.5
    yaw = math.radians(90)
    goal.pose.orientation.z = math.sin(yaw / 2)
    goal.pose.orientation.w = math.cos(yaw / 2)

    navigator.goToPose(goal)

    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        print(feedback)

    print("Done")

if __name__ == '__main__':
    main()
