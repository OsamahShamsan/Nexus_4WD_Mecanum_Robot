from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    nexus_bringup_dir = get_package_share_directory('nexus_bringup')
    nexus_control_dir = get_package_share_directory('nexus_control')

    # Paths to config files
    twist_mux_params_file = os.path.join(nexus_bringup_dir, 'config', 'twist_mux.yaml')
    ps4_holonomic_file = os.path.join(nexus_control_dir, 'config', 'ps4-holonomic.yaml')
    nexus_ctrl_yaml = os.path.join(nexus_control_dir, 'config', 'nexus_4wd_mecanum_controllers.yaml')

    for path in [twist_mux_params_file,
             ps4_holonomic_file, nexus_ctrl_yaml]:
        if not os.path.exists(path):
            raise FileNotFoundError(f"[LAUNCH ERROR] Required config file missing: {path}")
    
    return LaunchDescription([
        # Joy Node
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
        ),

        # Teleop Twist Joy Node
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            output='screen',
            parameters=[ps4_holonomic_file],
        ),
    ])
