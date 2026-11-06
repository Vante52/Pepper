"""Arranque del bridge con reloj ROS explícito y catálogo instalado."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory('robot_skill_bridge'))
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('config_file', default_value=str(share/'config/pepper_skills.yaml')),
        DeclareLaunchArgument('port', default_value='8765'),
        DeclareLaunchArgument('node_name', default_value='robot_skill_bridge'),
        Node(package='robot_skill_bridge', executable='skill_bridge', output='screen',
             name=LaunchConfiguration('node_name'),
             parameters=[{
                 'use_sim_time': ParameterValue(
                     LaunchConfiguration('use_sim_time'), value_type=bool),
                 'config_file': LaunchConfiguration('config_file'),
                 'port': ParameterValue(LaunchConfiguration('port'), value_type=int),
             }]),
    ])
