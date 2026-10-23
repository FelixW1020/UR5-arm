"""The whole Quest <-> UR5 pipeline in RViz, against a kinematic UR5: no robot, no cameras.

  ros2 launch ur5_bringup vr_sim.launch.py                            # fake Quest, drag in RViz
  ros2 launch ur5_bringup vr_sim.launch.py controller_mode:=scripted  # hands-free square test
  ros2 launch ur5_bringup vr_sim.launch.py quest:=real                # real headset, sim arm

quest:=sim   vr_sim's quest_sim renders the tags into fake passthrough frames, registers once
             through vr_tag_detector -> vr_registration, and publishes the ground truth
             (vr_world_truth, quest_*_truth) so the registration and the arm's direction can be
             scored: /quest_sim/registration_error_{m,deg}, /vr_alignment_monitor/*.
quest:=real  the Quest app connects through ros_tcp_endpoint (tcp_port) and registers against
             the robot cameras' tag map, so the real headset drives the simulated arm before
             the real arm is involved.

Both run robot_state_publisher (ur_description, name:=ur5), ur5_kinematic_sim (follows
/vr_teleop/target_pose), vr_registration (writes ~/.ros/vr_sim/vr_registration.yaml, never the
real one), vr_tag_detector, quest_bridge, vr_teleop and vr_alignment_monitor.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import (Command, FindExecutable, LaunchConfiguration,
                                  PathJoinSubstitution)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

DEFAULT_TAG_MAP = {'sim': '~/.ros/vr_sim/tag_map.yaml', 'real': '~/.ros/handeye/tag_map.yaml'}
REFERENCE_FRAME = {'sim': 'quest_right_controller_truth', 'real': 'quest_right_controller'}


def _quest_nodes(context):
    arg = lambda n: LaunchConfiguration(n).perform(context)  # noqa: E731
    quest = arg('quest')
    if quest not in DEFAULT_TAG_MAP:
        raise RuntimeError(f"quest must be 'sim' or 'real', got {quest!r}")
    config = PathJoinSubstitution([FindPackageShare('ur5_bringup'), 'config', 'vr_sim.yaml'])
    tag_map = arg('tag_map_file') or DEFAULT_TAG_MAP[quest]
    nodes = [
        Node(package='vr_registration', executable='registration_node', name='vr_registration',
             parameters=[config, {'tag_map_file': tag_map}], output='screen'),
        Node(package='vr_teleop', executable='alignment_monitor', name='vr_alignment_monitor',
             parameters=[config, {'reference_frame': REFERENCE_FRAME[quest]}], output='screen'),
    ]
    if quest == 'sim':
        nodes.append(Node(
            package='vr_sim', executable='quest_sim', name='quest_sim', output='screen',
            parameters=[config, {'tag_map_file': tag_map,
                                 'controller_mode': arg('controller_mode')}]))
    else:
        nodes.append(Node(
            package='ros_tcp_endpoint', executable='default_server_endpoint',
            name='ros_tcp_endpoint', output='screen',
            parameters=[{'ROS_IP': '0.0.0.0', 'ROS_TCP_PORT': int(arg('tcp_port'))}]))
    return nodes


def generate_launch_description():
    bringup = FindPackageShare('ur5_bringup')
    config = PathJoinSubstitution([bringup, 'config', 'vr_sim.yaml'])
    arg = LaunchConfiguration
    description = ParameterValue(Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]), ' ',
        PathJoinSubstitution([FindPackageShare('ur_description'), 'urdf', 'ur.urdf.xacro']),
        ' name:=ur5 ur_type:=', arg('ur_type')]), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument('quest', default_value='sim',
                              description='sim (vr_sim quest_sim) or real (Quest app via '
                                          'ros_tcp_endpoint)'),
        DeclareLaunchArgument('controller_mode', default_value='interactive',
                              description='quest:=sim: interactive (drag the controller in '
                                          'RViz) or scripted (loops a square with the grip held)'),
        DeclareLaunchArgument('ur_type', default_value='ur5'),
        DeclareLaunchArgument('tag_map_file', default_value='',
                              description="'' = ~/.ros/vr_sim/tag_map.yaml (written by quest_sim) "
                                          'for quest:=sim, ~/.ros/handeye/tag_map.yaml for '
                                          'quest:=real'),
        DeclareLaunchArgument('tcp_port', default_value='10000',
                              description='ros_tcp_endpoint port (quest:=real)'),
        DeclareLaunchArgument('rviz', default_value='true'),

        Node(package='robot_state_publisher', executable='robot_state_publisher',
             name='robot_state_publisher', output='screen',
             parameters=[{'robot_description': description}]),
        Node(package='vr_sim', executable='ur5_kinematic_sim', name='ur5_kinematic_sim',
             parameters=[config, {'ur_type': arg('ur_type')}], output='screen'),
        Node(package='vr_tag_detector', executable='tag_detector', name='vr_tag_detector',
             parameters=[config], output='screen'),
        Node(package='vr_tracking', executable='quest_bridge', name='quest_bridge',
             parameters=[config], output='screen'),
        Node(package='vr_teleop', executable='teleop', name='vr_teleop',
             parameters=[config], output='screen'),
        OpaqueFunction(function=_quest_nodes),
        Node(package='rviz2', executable='rviz2', name='rviz2',
             arguments=['-d', PathJoinSubstitution([bringup, 'rviz', 'vr_sim.rviz'])],
             condition=IfCondition(arg('rviz'))),
    ])
