"""Launch hw4_control with hw4_localization.yaml (override with params_file:=..., waypoints_file:=..., log_dir:=...)."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(get_package_share_directory('robot_control'), 'config')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=os.path.join(config, 'hw4_localization.yaml')),
        DeclareLaunchArgument('waypoints_file', default_value=os.path.join(config, 'hw4_fast.txt')),
        DeclareLaunchArgument('log_dir', default_value='/home/ubuntu/hw4_logs'),
        Node(package='robot_control', executable='hw4_control', name='hw4_control', output='screen',
             parameters=[LaunchConfiguration('params_file'), {'waypoints_file': LaunchConfiguration('waypoints_file'),
                                                              'log_dir': LaunchConfiguration('log_dir')}]),
    ])
