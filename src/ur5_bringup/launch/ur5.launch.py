"""UR5 driver only (Universal_Robots_ROS2_Driver)."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('ur_type', default_value='ur5',
                              description='ur5 (CB3) or ur5e (e-Series)'),
        DeclareLaunchArgument('robot_ip', default_value='192.168.1.102'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('ur_robot_driver'), 'launch', 'ur_control.launch.py'])),
            launch_arguments={
                'ur_type': LaunchConfiguration('ur_type'),
                'robot_ip': LaunchConfiguration('robot_ip'),
                'launch_rviz': 'false',
            }.items()),
    ])
