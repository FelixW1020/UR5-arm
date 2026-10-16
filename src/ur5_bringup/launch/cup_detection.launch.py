"""Run-time stack: calibrated camera TFs + YOLO + 3D localization on one camera.

  ros2 launch ur5_bringup cup_detection.launch.py robot_ip:=<UR_IP> \
      wrist_cam_serial:=<SN> table_cam_serial:=<SN> detection_camera:=table_cam
  ros2 service call /object_localization/locate handeye_interfaces/srv/LocateObject \
      "{class_name: cup, timeout_s: 2.0}"
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _detection_nodes(context):
    cam = LaunchConfiguration('detection_camera').perform(context)
    config = PathJoinSubstitution([FindPackageShare('ur5_bringup'), 'config', 'detection.yaml'])
    arg = LaunchConfiguration
    return [
        Node(package='yolo_detector', executable='yolo_node', name='yolo_detector',
             output='screen',
             parameters=[config, {'image_topic': f'/camera/{cam}/color/image_raw',
                                  'model': arg('model'), 'classes': arg('target_class'),
                                  'device': arg('device')}]),
        Node(package='object_localization', executable='localization_node',
             name='object_localization', output='screen',
             parameters=[config, {
                 'depth_topic': f'/camera/{cam}/aligned_depth_to_color/image_raw',
                 'camera_info_topic': f'/camera/{cam}/color/camera_info',
                 'target_class': arg('target_class')}]),
    ]


def generate_launch_description():
    bringup = FindPackageShare('ur5_bringup')
    arg = LaunchConfiguration
    return LaunchDescription([
        DeclareLaunchArgument('launch_robot', default_value='true'),
        DeclareLaunchArgument('launch_cameras', default_value='true'),
        DeclareLaunchArgument('ur_type', default_value='ur5'),
        DeclareLaunchArgument('robot_ip', default_value='192.168.1.102'),
        DeclareLaunchArgument('wrist_cam_serial', default_value=''),
        DeclareLaunchArgument('table_cam_serial', default_value=''),
        DeclareLaunchArgument('detection_camera', default_value='table_cam'),
        DeclareLaunchArgument('model', default_value='yolo11n.pt'),
        DeclareLaunchArgument('target_class', default_value='cup'),
        DeclareLaunchArgument('device', default_value='auto', description='auto, cpu or cuda:0'),
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
        OpaqueFunction(function=_detection_nodes),
        Node(package='rviz2', executable='rviz2', name='rviz2',
             arguments=['-d', PathJoinSubstitution([bringup, 'rviz', 'detection.rviz'])],
             condition=IfCondition(arg('rviz'))),
    ])
