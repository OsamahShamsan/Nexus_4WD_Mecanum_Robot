from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    #nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    nexus_bringup_dir = get_package_share_directory('nexus_bringup')
    nexus_control_dir = get_package_share_directory('nexus_control')
    #nexus_serial_conn_dir = get_package_share_directory('nexus_serial_conn')
    #nexus_odom_dir = get_package_share_directory('nexus_odom')

    # Paths to config files
    #nav2_launch_file = os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
    #map_file = os.path.join(nexus_bringup_dir, 'maps', 'warehouse_speed.yaml')
    #nav2_params_file = os.path.join(nexus_bringup_dir, 'config', 'nav2_params.yaml')
    twist_mux_params_file = os.path.join(nexus_bringup_dir, 'config', 'twist_mux.yaml')
    ps4_holonomic_file = os.path.join(nexus_control_dir, 'config', 'ps4-holonomic.yaml')
    nexus_ctrl_yaml = os.path.join(nexus_control_dir, 'config', 'nexus_4wd_mecanum_controllers.yaml')
    #serial_conn_file = os.path.join(nexus_serial_conn_dir, 'config', 'serial_conn.yaml')
    #odom_params_file = os.path.join(nexus_odom_dir, 'config', 'odom_params.yaml')

    for path in [twist_mux_params_file,
             ps4_holonomic_file, nexus_ctrl_yaml]:
        if not os.path.exists(path):
            raise FileNotFoundError(f"[LAUNCH ERROR] Required config file missing: {path}")
    
    #return LaunchDescription([
        # Joy Node
        #Node(
        #    package='joy',
        #    executable='joy_node',
        #    name='joy_node',
        #    output='screen',
        #),

        # Teleop Twist Joy Node
        #Node(
        #    package='teleop_twist_joy',
        #    executable='teleop_node',
        #    name='teleop_twist_joy_node',
        #    output='screen',
        #    parameters=[ps4_holonomic_file],
            #remappings=[('/cmd_vel', '/cmd_vel_joystick')]
        #),

        # Nexus Twist Node
        #Node(
        #    package='nexus_control',
        #    executable='nexus_twist_node',
        #    name='nexus_twist_node',
        #    parameters=[nexus_ctrl_yaml],
        #    output='screen',
        #),

        # Twist Mux Node
        #Node(
        #    package='twist_mux',
        #    executable='twist_mux',
        #    name='twist_mux',
        #    parameters=[twist_mux_params_file],
        #    remappings=[('/cmd_vel_out', '/cmd_vel')]
        #)

        # Serial Connection Node
        #Node(
         #   package='nexus_serial_conn',
         #   executable='nexus_serial_conn_node',
          #  name='nexus_serial_conn_node',
            #parameters=[serial_conn_file],
         #   output='screen',
        #),

        # Odometry Node
        #Node(
        #    package='nexus_odom',
        #    executable='nexus_odom_node',
        #    name='nexus_odom_node',
        #    #parameters=[odom_params_file],
        #    output='screen',
        #),

        # Nav2 bringup (via IncludeLaunchDescription)
        #IncludeLaunchDescription(
        #    PythonLaunchDescriptionSource(nav2_launch_file),
        #    launch_arguments={
        #        'use_sim_time': 'false',
        #        'map': map_file,
        #        'params_file': nav2_params_file
        #    }.items()
        #)
    #])
