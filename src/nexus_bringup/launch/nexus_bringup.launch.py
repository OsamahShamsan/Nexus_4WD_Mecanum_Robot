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
                '/home/mecroka/nexus_4wd_mecanum_ws/src/nexus_bringup/config/ps4-holonomic.yaml'
            ],
        ),

        # Nexus Twist Node
        Node(
            package='nexus_control',
            executable='nexus_twist_node',
            name='nexus_twist_node',
            #parameters=[
            #    '/home/mecroka/nexus_4wd_mecanum_ws/src/nexus_bringup/config/params.yaml'
            #],
            output='screen',
        ),

        # Nexus Serial Conn Node
        Node(
            package='nexus_serial_conn',
            executable='nexus_serial_conn_node',
            name='nexus_serial_conn_node',
            #parameters=[
            #    '/home/mecroka/nexus_4wd_mecanum_ws/src/nexus_bringup/config/params.yaml'
            #],
            output='screen',
        ),

        # Odom Nexus Node
        Node(
            package='nexus_odom',
            executable='nexus_odom_node',
            name='nexus_odom_node',
            #parameters=[
            #    '/home/mecroka/nexus_4wd_mecanum_ws/src/nexus_bringup/config/params.yaml'
            #],
            output='screen',
        ),
    ])
