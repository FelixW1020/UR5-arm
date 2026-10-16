"""Hand-eye calibration session for ONE camera. Run it once per camera.

  # wrist camera: board fixed on the table
  ros2 launch ur5_bringup calibration.launch.py camera:=wrist_cam robot_ip:=<UR_IP> \
      wrist_cam_serial:=<SN> table_cam_serial:=<SN>
  # table camera: board clamped to the tool flange
  ros2 launch ur5_bringup calibration.launch.py camera:=table_cam ...

mount defaults to eye_in_hand for wrist_cam and eye_to_hand for table_cam.
The result is written to ~/.ros/handeye/<camera>.yaml.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

DEFAULT_MOUNT = {'wrist_cam': 'eye_in_hand', 'table_cam': 'eye_to_hand'}


def _calibration_nodes(context):
    cam = LaunchConfiguration('camera').perform(context)
    mount = LaunchConfiguration('mount').perform(context)
    if mount == 'auto':
        if cam not in DEFAULT_MOUNT:
            raise RuntimeError(f'Set mount:=eye_in_hand|eye_to_hand for camera {cam!r}')
        mount = DEFAULT_MOUNT[cam]
    config = PathJoinSubstitution([FindPackageShare('ur5_bringup'), 'config', 'calibration.yaml'])
    return [
        Node(package='calib_target_detector', executable='target_detector',
             name='target_detector', output='screen',
             parameters=[config, {'image_topic': f'/camera/{cam}/color/image_raw',
                                  'camera_info_topic': f'/camera/{cam}/color/camera_info'}]),
        Node(package='handeye_calibration', executable='calibration_node',
             name='handeye_calibration', output='screen',
             parameters=[config, {'mount': mount,
                                  'output_file': f'~/.ros/handeye/{cam}.yaml'}]),
        Node(package='handeye_motion', executable='motion_node', name='handeye_motion',
             output='screen',
             parameters=[config, {'poses_file': f'~/.ros/handeye/{cam}_poses.yaml'}],
             condition=IfCondition(LaunchConfiguration('use_motion'))),
    ]


def generate_launch_description():
    bringup = FindPackageShare('ur5_bringup')
    arg = LaunchConfiguration
    return LaunchDescription([
        DeclareLaunchArgument('camera', default_value='table_cam',
                              description='wrist_cam or table_cam'),
        DeclareLaunchArgument('mount', default_value='auto',
                              description='auto, eye_in_hand or eye_to_hand'),
        DeclareLaunchArgument('launch_robot', default_value='true'),
        DeclareLaunchArgument('launch_cameras', default_value='true'),
        DeclareLaunchArgument('ur_type', default_value='ur5'),
        DeclareLaunchArgument('robot_ip', default_value='192.168.1.102'),
        DeclareLaunchArgument('wrist_cam_serial', default_value=''),
        DeclareLaunchArgument('table_cam_serial', default_value=''),
        DeclareLaunchArgument('use_motion', default_value='false',
                              description='start handeye_motion for automatic pose replay'),
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
        OpaqueFunction(function=_calibration_nodes),
        Node(package='rviz2', executable='rviz2', name='rviz2',
             arguments=['-d', PathJoinSubstitution([bringup, 'rviz', 'calibration.rviz'])],
             condition=IfCondition(arg('rviz'))),
    ])
