"""Visualización de Pepper."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import xacro


def launch_setup(context):
    """Configura el modelo y los nodos de visualización."""
    package_share = Path(get_package_share_directory('pepper_description'))
    height = LaunchConfiguration('base_link_height').perform(context)
    robot_xml = xacro.process_file(
        str(package_share / 'urdf' / 'pepper_robot.xacro'),
        mappings={'base_link_height': height},
    ).toxml()
    use_sim_time = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
    return [
        Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_xml, 'use_sim_time': use_sim_time}],
        ),
        Node(
            package='joint_state_publisher_gui', executable='joint_state_publisher_gui',
            output='screen', condition=IfCondition(LaunchConfiguration('gui')),
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        Node(
            package='rviz2', executable='rviz2', output='screen',
            condition=IfCondition(LaunchConfiguration('rviz')),
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-d', str(package_share / 'rviz' / 'pepper.rviz')],
        ),
    ]


def generate_launch_description():
    """Genera el lanzamiento de visualización."""
    return LaunchDescription([
        DeclareLaunchArgument(
            'base_link_height', default_value='0.602',
            description='Altura nominal de base_link en metros',
        ),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        OpaqueFunction(function=launch_setup),
    ])
