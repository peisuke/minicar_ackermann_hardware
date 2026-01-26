#!/usr/bin/env python3
"""
Launch file for Ackermann steering robot (TT-02) hardware bringup.

Launches:
- Robot State Publisher
- Controller Manager with hardware interface
- Joint State Broadcaster
- Ackermann Steering Controller
- RPLiDAR C1 node
- Static TF for laser frame
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    # Launch arguments
    robot_ns_arg = DeclareLaunchArgument(
        'robot_ns',
        default_value='real_robot',
        description='Robot namespace'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )

    rplidar_serial_port_arg = DeclareLaunchArgument(
        'rplidar_serial_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for RPLiDAR'
    )

    rplidar_inverted_arg = DeclareLaunchArgument(
        'rplidar_inverted',
        default_value='false',
        description='Invert LiDAR scan direction (left-right flip)'
    )

    # Launch configurations
    robot_ns = LaunchConfiguration('robot_ns')
    use_sim_time = LaunchConfiguration('use_sim_time')
    rplidar_serial_port = LaunchConfiguration('rplidar_serial_port')
    rplidar_inverted = LaunchConfiguration('rplidar_inverted')

    # URDF/XACRO file for hardware
    xacro_file = PathJoinSubstitution([
        FindPackageShare('minicar_ackermann_hardware'),
        'config', 'minicar_ackermann_real.xacro'
    ])

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' robot_ns:=', robot_ns]),
        value_type=str
    )

    # Hardware config file
    hardware_config = PathJoinSubstitution([
        FindPackageShare('minicar_ackermann_hardware'),
        'config', 'ackermann_controller.yaml'
    ])

    return LaunchDescription([
        robot_ns_arg,
        use_sim_time_arg,
        rplidar_serial_port_arg,
        rplidar_inverted_arg,

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace=robot_ns,
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': use_sim_time
            }]
        ),

        # Controller Manager (hardware interface)
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            namespace=robot_ns,
            output='screen',
            parameters=[
                hardware_config,
                {
                    'use_sim_time': use_sim_time,
                    'robot_description': robot_description
                }
            ]
        ),

        # Joint State Broadcaster
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "joint_state_broadcaster",
                "--controller-manager", ["/", robot_ns, "/controller_manager"]
            ],
            output="screen",
            parameters=[{'use_sim_time': use_sim_time}]
        ),

        # Ackermann Steering Controller
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                'ackermann_steering_controller',
                '--controller-manager', ["/", robot_ns, "/controller_manager"]
            ],
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),

        # RPLiDAR C1
        Node(
            package='rplidar_ros2',
            executable='rplidar_node',
            name='rplidar_node',
            namespace=robot_ns,
            output='screen',
            parameters=[{
                'serial_port': rplidar_serial_port,
                'serial_baudrate': 460800,  # C1 baud rate
                'frame_id': 'laser',
                'inverted': rplidar_inverted,
                'angle_compensate': True,
                'scan_frequency': 10.0,
                'use_sim_time': use_sim_time
            }]
        ),

        # Static TF: base_link -> laser
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser_tf',
            namespace=robot_ns,
            output='screen',
            arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'laser'],
            parameters=[{'use_sim_time': use_sim_time}]
        ),
    ])
