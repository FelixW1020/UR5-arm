"""Attach every calibrated camera to the robot TF tree (one handeye_publisher each).

Expects ~/.ros/handeye/<camera>.yaml from calibration.launch.py.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _publishers(context):
    cameras = [c.strip() for c in LaunchConfiguration('cameras').perform(context).split(',')
               if c.strip()]
    directory = LaunchConfiguration('calibration_dir').perform(context).rstrip('/')
    return [Node(package='handeye_publisher', executable='publisher',
                 name=f'handeye_publisher_{cam}',
                 parameters=[{'calibration_file': f'{directory}/{cam}.yaml',
                              'camera_base_frame': f'{cam}_link'}],
                 output='screen')
            for cam in cameras]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('cameras', default_value='wrist_cam,table_cam'),
        DeclareLaunchArgument('calibration_dir', default_value='~/.ros/handeye'),
        OpaqueFunction(function=_publishers),
    ])
