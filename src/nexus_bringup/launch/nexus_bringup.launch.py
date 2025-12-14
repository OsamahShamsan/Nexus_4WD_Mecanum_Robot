from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    description_dir = get_package_share_directory('nexus_description')
    bringup_dir = get_package_share_directory('nexus_bringup')
    sllidar_dir = get_package_share_directory('sllidar_ros2')

    urdf_path = os.path.join(
        description_dir,
        'urdf',
        'nexus_4wd_mecanum.urdf.xacro'
    )

    robot_description = ParameterValue(
        Command(['xacro ', urdf_path]),
        value_type=str
    )

    # -------------------------------
    # Robot State Publisher (static TF)
    # -------------------------------
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False
        }],
    )

    # -------------------------------
    # LiDAR (vendor launch)
    # -------------------------------
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sllidar_dir, 'launch', 'sllidar_a3_launch.py')
        ),
        launch_arguments={
            'serial_port': '/dev/ttyUSB0',
            'serial_baudrate': '256000'
        }.items(),
    )

    # -------------------------------
    # Static Transform: base_link -> laser
    # (LiDAR frame connection to robot base)
    # -------------------------------
    static_tf_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_laser_tf',
        arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'laser'],
        output='screen'
    )

    # -------------------------------
    # SLAM Toolbox (online async)
    # Uses external odometry from STM32 node via /odom topic
    # -------------------------------
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('slam_toolbox'),
                'launch',
                'online_async_launch.py'
            )
        ),
        launch_arguments={
            'slam_params_file': os.path.join(
                bringup_dir,
                'config',
                'mapper_params_online_async.yaml'
            ),
            'use_sim_time': 'false'
        }.items(),
    )

    return LaunchDescription([
        robot_state_pub,
        static_tf_laser,
        lidar_launch,
        slam_toolbox_launch,
    ])
