from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        ## Joy Node
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
        ),
        
        ## Teleop Twist Joy Node
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            output='screen',
            parameters=[
                '/home/mecroka/bumperbot_ws/src/ros_arduino_conn/config/ps4-holonomic.yaml'
            ],
        ),

        # Twist Nexus Node
        Node(
            package='ros_arduino_conn',
            executable='twist_nexus_node',
            name='twist_nexus_node',
            #parameters=[
            #    '/home/mecroka/bumperbot_ws/src/ros_arduino_conn/config/params.yaml'
            #],
            output='screen',
        ),

        # Serial Conn Node
        #Node(
        #    package='ros_arduino_conn',
        #    executable='serial_conn_node',
        #    name='serial_conn_node',
        #    #parameters=[
        #    #    '/home/mecroka/bumperbot_ws/src/ros_arduino_conn/config/params.yaml'
        #    #],
        #    output='screen',
        #),

        # Odom Nexus Node
        Node(
            package='ros_arduino_conn',
            executable='odom_nexus_node',
            name='odom_nexus_node',
            #parameters=[
            #    '/home/mecroka/bumperbot_ws/src/ros_arduino_conn/config/params.yaml'
            #],
            output='screen',
        ),
    ])
