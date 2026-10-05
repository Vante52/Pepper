from pathlib import Path

import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    #Encontrar los recursos del paquete
    package_share = Path(
        get_package_share_directory('pepper_description')
    )

    # Encontrar el xacro base
    xacro_path = package_share / 'urdf' / 'pepper_robot.xacro'
    #Archivo de configuración de rviz
    rviz_config_path = package_share / 'rviz' / 'pepper.rviz'
    
    # Procesar Xacro y obtener el URDF como texto XML.
    robot_xml = xacro.process_file(str(xacro_path)).toxml()    
    
    # Nodo que publica los modelos del robot a los tf
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_xml}
        ],
    )

    # gui con sliders que publica a los joints para ver el comportamiento de las articulaciones del robot
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
    )

    # Abrir RViz.
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', str(rviz_config_path)],
    )

    # Devolver las acciones que ROS 2 debe ejecutar.
    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz,
    ])