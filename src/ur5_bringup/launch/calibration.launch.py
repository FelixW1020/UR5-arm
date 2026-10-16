"""Hand-eye calibration session.

  ros2 launch ur5_bringup calibration.launch.py robot_ip:=192.168.1.102 mount:=eye_to_hand
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup = FindPackageShare('ur5_bringup')
    config = PathJoinSubstitution([bringup, 'config', 'calibration.yaml'])
    arg = LaunchConfiguration

    return LaunchDescription([
        DeclareLaunchArgument('launch_robot', default_value='true'),
        DeclareLaunchArgument('launch_camera', default_value='true'),
        DeclareLaunchArgument('ur_type', default_value='ur5'),
        DeclareLaunchArgument('robot_ip', default_value='192.168.1.102'),
        DeclareLaunchArgument('mount', default_value='eye_to_hand',
                              description='eye_in_hand (camera on wrist) or eye_to_hand (fixed camera)'),
        DeclareLaunchArgument('use_motion', default_value='false',
                              description='start handeye_motion for automatic pose replay'),
        DeclareLaunchArgument('rviz', default_value='true'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([bringup, 'launch', 'ur5.launch.py'])),
            launch_arguments={'ur_type': arg('ur_type'), 'robot_ip': arg('robot_ip')}.items(),
            condition=IfCondition(arg('launch_robot'))),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([bringup, 'launch', 'camera.launch.py'])),
            condition=IfCondition(arg('launch_camera'))),

        Node(package='calib_target_detector', executable='target_detector',
             name='target_detector', parameters=[config], output='screen'),
        Node(package='handeye_calibration', executable='calibration_node',
             name='handeye_calibration', parameters=[config, {'mount': arg('mount')}],
             output='screen'),
        Node(package='handeye_motion', executable='motion_node', name='handeye_motion',
             parameters=[config], output='screen', condition=IfCondition(arg('use_motion'))),
        Node(package='rviz2', executable='rviz2', name='rviz2',
             arguments=['-d', PathJoinSubstitution([bringup, 'rviz', 'calibration.rviz'])],
             condition=IfCondition(arg('rviz'))),
    ])
