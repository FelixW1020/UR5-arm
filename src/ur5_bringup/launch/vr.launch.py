"""Robot <-> Meta Quest registration and the ROS side of the game link.

  ros2 launch ur5_bringup vr.launch.py robot_ip:=<UR_IP> \
      wrist_cam_serial:=<SN> table_cam_serial:=<SN>

Starts: robot + both cameras + calibrated camera TFs, aruco_tag_mapper,
vr_registration, vr_pose_relay, and (tcp_endpoint:=true) the ROS-TCP-Endpoint
the Quest app connects to on port 10000.
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
    config = PathJoinSubstitution([bringup, 'config', 'vr.yaml'])
    arg = LaunchConfiguration

    return LaunchDescription([
        DeclareLaunchArgument('launch_robot', default_value='true'),
        DeclareLaunchArgument('launch_cameras', default_value='true'),
        DeclareLaunchArgument('ur_type', default_value='ur5'),
        DeclareLaunchArgument('robot_ip', default_value='192.168.1.102'),
        DeclareLaunchArgument('wrist_cam_serial', default_value=''),
        DeclareLaunchArgument('table_cam_serial', default_value=''),
        DeclareLaunchArgument('tcp_endpoint', default_value='true',
                              description='start ros_tcp_endpoint for the Unity app'),
        DeclareLaunchArgument('tcp_port', default_value='10000'),
        DeclareLaunchArgument('rviz', default_value='true'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([bringup, 'launch', 'ur5.launch.py'])),
            launch_arguments={'ur_type': arg('ur_type'), 'robot_ip': arg('robot_ip')}.items(),
            condition=IfCondition(arg('launch_robot'))),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([bringup, 'launch', 'cameras.launch.py'])),
            launch_arguments={'wrist_cam_serial': arg('wrist_cam_serial'),
                              'table_cam_serial': arg('table_cam_serial')}.items(),
            condition=IfCondition(arg('launch_cameras'))),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([bringup, 'launch', 'calibrated_tf.launch.py']))),

        Node(package='aruco_tag_mapper', executable='tag_mapper', name='aruco_tag_mapper',
             parameters=[config], output='screen'),
        Node(package='vr_registration', executable='registration_node', name='vr_registration',
             parameters=[config], output='screen'),
        Node(package='vr_registration', executable='pose_relay', name='vr_pose_relay',
             parameters=[config], output='screen'),
        Node(package='ros_tcp_endpoint', executable='default_server_endpoint',
             name='ros_tcp_endpoint', output='screen',
             parameters=[{'ROS_IP': '0.0.0.0', 'ROS_TCP_PORT': arg('tcp_port')}],
             condition=IfCondition(arg('tcp_endpoint'))),
        Node(package='rviz2', executable='rviz2', name='rviz2',
             arguments=['-d', PathJoinSubstitution([bringup, 'rviz', 'detection.rviz'])],
             condition=IfCondition(arg('rviz'))),
    ])
