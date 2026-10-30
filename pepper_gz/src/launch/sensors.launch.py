"""Entrada compatible para la simulación de Pepper con sensores y control."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    """Usa el lanzamiento único para evitar duplicar Gazebo, TF y puentes."""
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([
            get_package_share_directory('pepper_gz'), 'launch', 'simulation.launch.py',
        ]))),
    ])
