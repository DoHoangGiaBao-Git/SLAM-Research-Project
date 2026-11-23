import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Package name
    package_name = 'pose_graph_slam'

    pkg_share = get_package_share_directory(package_name)
    rviz_config_path = os.path.join(pkg_share, 'rviz2', 'config.rviz')

    # Set use_sim_time for all nodes
    node_params = [{'use_sim_time': False}]

    # Run Pose Graph SLAM
    pose_graph_slam_node = Node(
        package=package_name,
        executable='pose_graph_slam_node',
        name='pose_graph_slam_node',
        output='screen',
        parameters=node_params,
    )

    # Occupancy Grid Map
    occupancy_grid_map_node = Node(
        package=package_name,
        executable='occupancy_grid_map_node',
        name='occupancy_grid_map_node',
        output='screen',
        parameters=node_params,
    )

    # RViz2 Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen',
        parameters=node_params,
    )

    return LaunchDescription([
        pose_graph_slam_node,
        occupancy_grid_map_node,
        rviz_node,
    ])