"""Both RealSense cameras, depth aligned to colour.

  wrist_cam  on the UR5 tool flange  -> /camera/wrist_cam/...,  frames wrist_cam_*
  table_cam  fixed on the table      -> /camera/table_cam/...,  frames table_cam_*

Serial numbers are required with two cameras (otherwise both drivers grab the same
device). Find them with:  rs-enumerate-devices -s
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

CAMERAS = ('wrist_cam', 'table_cam')


def _camera(name):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('realsense2_camera'), 'launch', 'rs_launch.py'])),
        launch_arguments={
            'camera_namespace': 'camera',
            'camera_name': name,
            # Quoted so the driver keeps it a string (serials can start with 0).
            'serial_no': ["'", LaunchConfiguration(f'{name}_serial'), "'"],
            'align_depth.enable': 'true',
            'enable_sync': 'true',
            'pointcloud.enable': 'false',
        }.items())


def generate_launch_description():
    return LaunchDescription(
        [DeclareLaunchArgument(f'{n}_serial', default_value='',
                               description=f'RealSense serial number of {n}') for n in CAMERAS]
        + [_camera(n) for n in CAMERAS])
