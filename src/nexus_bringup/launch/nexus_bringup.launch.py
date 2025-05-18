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
                '/home/mecroka/nexus_4wd_mecanum_ws/src/nexus_control/config/ps4-holonomic.yaml'
                
            ],
            ## Remap the output of teleop twist node to joystick_cmd_vel so that it can prioritized over nexus_twist_node
            remappings=[
                ('/cmd_vel', '/joystick_cmd_vel')
            ]
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
        
        # Twist Mux Node
        Node(
            package='twist_mux',
            executable='twist_mux',
            name='twist_mux',
            parameters=['/home/mecroka/nexus_4wd_mecanum_ws/src/nexus_bringup/config/twist_mux.yaml'],
            remappings=[
                ('/cmd_vel_out', '/cmd_vel')  
            ]
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
