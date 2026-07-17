#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # World file
    world_file_name = "my_world.sdf"
    world_path = os.path.join(get_package_share_directory('robot_model'), 'worlds', world_file_name)

    # Robot model
    default_model = 'robot.urdf.xacro'

    # Launch Configurations
    mode_cfg = LaunchConfiguration('mode')
    
    # Automatically set use_sim_time to true if mode is 'sim', otherwise false
    use_sim_time_cfg = PythonExpression(["'true' if '", mode_cfg, "' == 'sim' else 'false'"])

    # Get the robot description from the xacro file
    robot_desc = Command([
        'xacro ',
        PathJoinSubstitution([
            FindPackageShare('robot_model'),
            'robot_description',
            LaunchConfiguration('robot_model')
        ])
    ])

    # Path to RViz config file (if it exists)
    rviz_config_path = os.path.join(get_package_share_directory('robot_model'), 'config', 'robot.rviz')

    # Condition to check if we are in simulation mode
    is_sim = IfCondition(PythonExpression(["'", mode_cfg, "' == 'sim'"]))

    return LaunchDescription([
        # --- LAUNCH ARGUMENTS ---
        DeclareLaunchArgument(
            'robot_model',
            default_value=default_model,
            description='URDF/Xacro file name to load'
        ),
        DeclareLaunchArgument(
            'mode',
            default_value='sim',
            choices=['sim', 'real'],
            description='Choose between sim (Gazebo) and real (Hardware) mode'
        ),

        # --- NODES FOR BOTH SIM & REAL ---
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time_cfg,
                'robot_description': robot_desc
            }],
        ),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time_cfg}],
        ),

        Node(
            package='ekf_slam',
            executable='odometry',
            name='odometry',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time_cfg}],
        ),

        Node(
            package='ekf_slam',
            executable='landmark_extraction',
            name='landmark_extraction',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time_cfg}],
        ),

        Node(
            package='ekf_slam',
            executable='ekf_slam',
            name='ekf_slam',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time_cfg}],
        ),

        # --- RVIZ ---
        TimerAction(
            period=5.0,  # Wait for robot_state_publisher to publish
            actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', rviz_config_path],
                parameters=[{'use_sim_time': use_sim_time_cfg}],
            )
        ]),

        # --- GAZEBO SIMULATION ---
        # 1. Start Gazebo Sim
        TimerAction(
            period=2.0,
            condition=is_sim,
            actions=[
                ExecuteProcess(
                    cmd=['gz', 'sim', world_path],
                    output='screen'
                )
            ]
        ),
        
        # 2. Spawn the robot into Gazebo
        TimerAction(
            period=5.0,
            condition=is_sim,
            actions=[
                ExecuteProcess(
                    cmd=[
                        'ros2', 'run', 'ros_gz_sim', 'create',
                        '-topic', 'robot_description',
                        '-name', 'my_robot',
                        '-x', '-5.0',
                        '-y', '-5.0'
                    ],
                    output='screen'
                )
            ]
        ),
        
        # 3. Start ros_gz_bridge
        TimerAction(
            period=6.0,
            condition=is_sim,
            actions=[
                Node(
                    package='ros_gz_bridge',
                    executable='parameter_bridge',
                    arguments=[
                        '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                        '/robot1/imu/data@sensor_msgs/msg/Imu@gz.msgs.IMU',
                        '/robot1/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                        '/robot1/scan/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
                        '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
                        '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                        '/robot1/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                        f'/world/simple_room/model/my_robot/joint_state@sensor_msgs/msg/JointState@gz.msgs.Model'
                    ],
                    output='screen',
                    remappings=[
                        (f'/world/simple_room/model/my_robot/joint_state', '/joint_states_raw'),
                        ('/imu', '/robot1/imu/data'),
                    ]
                )
            ]
        )
    ])