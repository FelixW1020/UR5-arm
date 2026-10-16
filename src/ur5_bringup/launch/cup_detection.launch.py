"""Run-time stack: calibrated camera TF + YOLO + 3D localization.

  ros2 launch ur5_bringup cup_detection.launch.py robot_ip:=192.168.1.102
  ros2 service call /object_localization/locate handeye_interfaces/srv/LocateObject \
      "{class_name: cup, timeout_s: 2.0}"
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
    config = PathJoinSubstitution([bringup, 'config', 'detection.yaml'])
    arg = LaunchConfiguration

    return LaunchDescription([
        DeclareLaunchArgument('launch_robot', default_value='true'),
        DeclareLaunchArgument('launch_camera', default_value='true'),
        DeclareLaunchArgument('ur_type', default_value='ur5'),
        DeclareLaunchArgument('robot_ip', default_value='192.168.1.102'),
        DeclareLaunchArgument('calibration_file', default_value='~/.ros/handeye/ur5_handeye.yaml'),
        DeclareLaunchArgument('model', default_value='yolo11n.pt'),
        DeclareLaunchArgument('target_class', default_value='cup'),
        DeclareLaunchArgument('device', default_value='auto', description="auto, cpu or cuda:0"),
        DeclareLaunchArgument('rviz', default_value='true'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([bringup, 'launch', 'ur5.launch.py'])),
            launch_arguments={'ur_type': arg('ur_type'), 'robot_ip': arg('robot_ip')}.items(),
            condition=IfCondition(arg('launch_robot'))),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([bringup, 'launch', 'camera.launch.py'])),
            condition=IfCondition(arg('launch_camera'))),

        Node(package='handeye_publisher', executable='publisher', name='handeye_publisher',
             parameters=[config, {'calibration_file': arg('calibration_file')}], output='screen'),
        Node(package='yolo_detector', executable='yolo_node', name='yolo_detector',
             parameters=[config, {'model': arg('model'), 'classes': arg('target_class'),
                                  'device': arg('device')}],
             output='screen'),
        Node(package='object_localization', executable='localization_node',
             name='object_localization',
             parameters=[config, {'target_class': arg('target_class')}], output='screen'),
        Node(package='rviz2', executable='rviz2', name='rviz2',
             arguments=['-d', PathJoinSubstitution([bringup, 'rviz', 'detection.rviz'])],
             condition=IfCondition(arg('rviz'))),
    ])
