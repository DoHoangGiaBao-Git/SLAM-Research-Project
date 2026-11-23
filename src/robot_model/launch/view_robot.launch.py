from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare('robot_model')
    urdf_file = PathJoinSubstitution([package_share, 'urdf', 'robot_model.urdf'])
    rviz_config = PathJoinSubstitution(
        [package_share, 'rviz', 'robot_model.rviz'])
    robot_description = ParameterValue(Command(['cat ', urdf_file]), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument(
            'serial_port',
            default_value=(
                '/dev/serial/by-id/'
                'usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0'),
            description='RPLIDAR serial device path'),
        DeclareLaunchArgument('laser_x', default_value='0.0'),
        DeclareLaunchArgument('laser_y', default_value='0.0'),
        DeclareLaunchArgument(
            'laser_z',
            default_value='0.10',
            description='Approximate scan-plane height above base_link, in meters'),
        DeclareLaunchArgument('laser_yaw', default_value='0.0'),
        Node(
            package='robot_model',
            executable='robot_feedback_node',
            name='robot_feedback_node',
            output='screen',
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen',
        ),
        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            name='rplidar',
            parameters=[{
                'channel_type': 'serial',
                'serial_port': LaunchConfiguration('serial_port'),
                'serial_baudrate': 115200,
                'frame_id': 'laser',
                'inverted': False,
                'angle_compensate': True,
                'topic_name': '/scan',
            }],
            output='screen',
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='laser_frame_publisher',
            arguments=[
                '--x', LaunchConfiguration('laser_x'),
                '--y', LaunchConfiguration('laser_y'),
                '--z', LaunchConfiguration('laser_z'),
                '--yaw', LaunchConfiguration('laser_yaw'),
                '--frame-id', 'base_link',
                '--child-frame-id', 'laser',
            ],
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
        ),
    ])
