"""Launch slam_control with slam_full.yaml (override with params_file:=..., waypoints_file:=...)."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(get_package_share_directory('robot_control'), 'config')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=os.path.join(config, 'slam_full.yaml')),
        DeclareLaunchArgument('waypoints_file', default_value=os.path.join(config, 'hw3_square.txt')),
        Node(package='robot_control', executable='slam_control', name='slam_control', output='screen',
             parameters=[LaunchConfiguration('params_file'), {'waypoints_file': LaunchConfiguration('waypoints_file')}]),
    ])
