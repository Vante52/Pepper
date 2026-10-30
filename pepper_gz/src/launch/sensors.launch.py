"""Inicia los sensores de Pepper en Gazebo Harmonic con Bullet Featherstone."""

import math
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess
from launch.actions import LogInfo, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
import xacro
import yaml


def flag(context, name):
    """Lee un argumento booleano de launch."""
    value = LaunchConfiguration(name).perform(context).lower()
    if value not in ('true', 'false'):
        raise ValueError(f'{name} debe ser true o false')
    return value == 'true'


def guard_instances(start_gazebo, publish_tf, start_bridge, bridges):
    """Detecta servidores y publicadores activos en la partición y dominio actuales."""
    result = subprocess.run(
        ['gz', 'service', '-l'], capture_output=True, text=True, timeout=8, check=True,
    )
    services = result.stdout.splitlines()
    worlds = [s for s in services if s.startswith('/world/') and s.endswith('/control')]
    if start_gazebo and worlds:
        raise RuntimeError(
            'Ya existe Gazebo en esta GZ_PARTITION. Use otra partición o '
            'start_gazebo:=false para reutilizar un mundo con los sistemas de sensores.',
        )
    if not start_gazebo and '/world/minimal_world/control' not in worlds:
        raise RuntimeError('No existe minimal_world en esta GZ_PARTITION')
    context = Context()
    rclpy.init(args=[], context=context)
    node = rclpy.create_node('pepper_sensors_preflight', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    try:
        for _ in range(5):
            executor.spin_once(timeout_sec=0.15)
        topics = ['/robot_description'] if publish_tf else []
        if start_bridge:
            topics += [bridge['ros_topic_name'] for bridge in bridges]
            topics += ['/pepper/camera/depth/points']
        conflicts = [topic for topic in topics if node.count_publishers(topic)]
        if publish_tf and any('robot_state_publisher' in name for name in node.get_node_names()):
            conflicts.append('robot_state_publisher')
        if conflicts:
            raise RuntimeError(
                'Publicadores ya activos: ' + ', '.join(conflicts)
                + '. Reutilice publish_tf:=false o start_bridge:=false según corresponda.',
            )
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown(context=context)


def validate_model(robot, config, bridges):
    """Comprueba frames, parámetros provisionales y correspondencia con el puente."""
    if config.get('profile') != 'provisional_simulation':
        raise ValueError('sensors.yaml debe identificar el perfil provisional_simulation')
    frames = {link.get('name') for link in robot.findall('link')}
    expected = {}
    names = []
    for camera in config['cameras'].values():
        names.append(camera['name'])
        for frame in (camera['frame'], camera['optical_frame']):
            if frame not in frames:
                raise ValueError(f'Frame ausente: {frame}')
        if (camera['width'] <= 0 or camera['height'] <= 0
                or not 0 < camera['horizontal_fov'] < math.pi
                or not 0 < camera['near'] < camera['far']):
            raise ValueError(f'Parámetros inválidos: {camera["name"]}')
        expected[camera['topic']] = ('sensor_msgs/msg/Image', camera['optical_frame'])
        expected[camera['info_topic']] = ('sensor_msgs/msg/CameraInfo', camera['optical_frame'])
        if camera['type'] == 'depth_camera':
            expected[camera['topic'] + '/points'] = (
                'sensor_msgs/msg/PointCloud2', camera['frame'],
            )
    for imu in config['imus'].values():
        names.append(imu['name'])
        if imu['frame'] not in frames:
            raise ValueError(f'Frame ausente: {imu["frame"]}')
        expected[imu['topic']] = ('sensor_msgs/msg/Imu', imu['frame'])
    for sensor in [*config['cameras'].values(), *config['imus'].values()]:
        if not math.isfinite(sensor['update_rate']) or sensor['update_rate'] <= 0:
            raise ValueError(f'Frecuencia inválida: {sensor["name"]}')
        for key, value in sensor.items():
            if key.endswith(('_mean', '_stddev')):
                if not math.isfinite(value) or (key.endswith('_stddev') and value < 0):
                    raise ValueError(f'Ruido inválido: {sensor["name"]}.{key}')
    if len(names) != len(set(names)) or len(names) != 5:
        raise ValueError('Deben existir cinco sensores distintos')
    expected['/pepper/joint_states'] = ('sensor_msgs/msg/JointState', None)
    expected['/world/minimal_world/clock'] = ('rosgraph_msgs/msg/Clock', None)
    if len(bridges) != len(expected):
        raise ValueError('Cantidad inesperada de puentes')
    if len({b['ros_topic_name'] for b in bridges}) != len(bridges):
        raise ValueError('Tópicos ROS duplicados en el puente')
    for bridge in bridges:
        if bridge['direction'] != 'GZ_TO_ROS':
            raise ValueError('Todos los puentes deben ser GZ_TO_ROS')
        contract = expected.pop(bridge['gz_topic_name'])
        if (bridge['ros_type_name'], bridge.get('frame_id')) != contract:
            raise ValueError(f'Contrato de puente incorrecto: {bridge["gz_topic_name"]}')
    if expected:
        raise ValueError(f'Puentes ausentes: {expected}')


def launch_setup(context):
    """Prepara el SDF y las instancias solicitadas."""
    share = Path(get_package_share_directory('pepper_gz'))
    description = Path(get_package_share_directory('pepper_description'))
    config_path = Path(LaunchConfiguration('sensors_config').perform(context))
    bridge_path = Path(LaunchConfiguration('bridge_config').perform(context))
    config = yaml.safe_load(config_path.read_text())
    bridges = yaml.safe_load(bridge_path.read_text())
    robot_xml = xacro.process_file(
        str(share / 'urdf' / 'pepper_gz.xacro'),
        mappings={'sensors_config': str(config_path)},
    ).toxml()
    robot = ET.fromstring(robot_xml)
    validate_model(robot, config, bridges)
    start_gazebo = flag(context, 'start_gazebo')
    publish_tf = flag(context, 'publish_tf')
    start_bridge = flag(context, 'start_bridge')
    guard_instances(start_gazebo, publish_tf, start_bridge, bridges)
    directory = Path(tempfile.mkdtemp(prefix='pepper_gz_'))
    urdf = directory / 'pepper.urdf'
    sdf = directory / 'pepper.sdf'
    try:
        urdf.write_text(robot_xml)
        result = subprocess.run(
            ['gz', 'sdf', '-p', str(urdf)], capture_output=True, text=True,
            timeout=20, check=True,
        )
        model = ET.fromstring(result.stdout)
        sensors = model.findall('model/link/sensor')
        if len(sensors) != 5 or len({s.get('name') for s in sensors}) != 5:
            raise RuntimeError('La conversión a SDF perdió o duplicó sensores')
        sdf.write_text(result.stdout)
    except Exception:
        shutil.rmtree(directory)
        raise
    paths = [str(description.parent), str(share.parent), os.getenv('GZ_SIM_RESOURCE_PATH', '')]
    environment = {'GZ_SIM_RESOURCE_PATH': os.pathsep.join(p for p in paths if p)}
    actions = []

    def cleanup(_context, *args, **kwargs):
        shutil.rmtree(directory, ignore_errors=True)
        return []

    actions.append(RegisterEventHandler(OnShutdown(
        on_shutdown=[OpaqueFunction(function=cleanup)],
    )))
    if start_gazebo:
        command = ['gz', 'sim', '-r', '-v', '3']
        if not flag(context, 'gui'):
            command += ['-s', '--headless-rendering']
        command += [str(share / 'worlds' / 'minimal_world.sdf')]
        gazebo = ExecuteProcess(cmd=command, additional_env=environment, output='screen')
        actions += [gazebo, RegisterEventHandler(OnProcessExit(
            target_action=gazebo,
            on_exit=[EmitEvent(event=Shutdown(reason='Gazebo terminó'))],
        ))]
    if publish_tf:
        actions.append(Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            name='pepper_robot_state_publisher', output='screen',
            parameters=[{'robot_description': robot_xml, 'use_sim_time': True}],
        ))
    if start_bridge:
        depth = config['cameras']['depth']
        actions += [
            Node(
                package='ros_gz_bridge', executable='parameter_bridge',
                name='pepper_sensors_bridge', output='screen',
                parameters=[{'config_file': str(bridge_path), 'use_sim_time': True}],
            ),
            Node(
                package='pepper_gz', executable='depth_points_optical', output='screen',
                parameters=[{
                    'use_sim_time': True,
                    'source_frame': depth['frame'], 'target_frame': depth['optical_frame'],
                }],
            ),
        ]
    checker = Node(
        package='pepper_gz', executable='check_sensors', output='screen',
        parameters=[{
            'use_sim_time': True, 'bridge_config': str(bridge_path),
            'sensors_config': str(config_path), 'timeout': 45.0,
        }],
    )

    def checked(event, _context):
        if event.returncode:
            return [EmitEvent(event=Shutdown(reason='Falló la comprobación de sensores'))]
        return [LogInfo(msg='Sensores y frames verificados')]

    if flag(context, 'check_sensors'):
        actions.append(RegisterEventHandler(OnProcessExit(target_action=checker, on_exit=checked)))
    if flag(context, 'spawn_robot'):
        z = LaunchConfiguration('spawn_z').perform(context)
        if not math.isfinite(float(z)):
            raise ValueError('spawn_z debe ser finito')
        spawn = Node(
            package='ros_gz_sim', executable='create', name='pepper_spawn', output='screen',
            arguments=['-world', 'minimal_world', '-file', str(sdf), '-name', 'pepper',
                       '-allow_renaming', 'false', '-z', z],
            parameters=[{'use_sim_time': True}], additional_env=environment,
        )

        def spawned(event, _context):
            if event.returncode:
                return [EmitEvent(event=Shutdown(reason='No se pudo insertar Pepper'))]
            return [checker] if flag(context, 'check_sensors') else []

        actions += [spawn, RegisterEventHandler(OnProcessExit(
            target_action=spawn, on_exit=spawned,
        ))]
    elif flag(context, 'check_sensors'):
        actions.append(checker)
    return actions


def generate_launch_description():
    """Declara opciones para iniciar o reutilizar componentes de simulación."""
    share = Path(get_package_share_directory('pepper_gz'))
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('start_gazebo', default_value='true'),
        DeclareLaunchArgument('spawn_robot', default_value='true'),
        DeclareLaunchArgument('publish_tf', default_value='true'),
        DeclareLaunchArgument('start_bridge', default_value='true'),
        DeclareLaunchArgument('check_sensors', default_value='true'),
        DeclareLaunchArgument('spawn_z', default_value='0.607'),
        DeclareLaunchArgument('sensors_config', default_value=str(share / 'config/sensors.yaml')),
        DeclareLaunchArgument(
            'bridge_config', default_value=str(share / 'config/sensors_bridge.yaml'),
        ),
        OpaqueFunction(function=launch_setup),
    ])
