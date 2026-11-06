"""Inicia Pepper, sus sensores y ros2_control con Bullet Featherstone."""

from functools import partial
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
from launch.actions import LogInfo, OpaqueFunction, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit, OnProcessStart, OnShutdown
from launch.events import Shutdown
from launch.logging import get_logger
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from pepper_gz.control_config import startup_groups, validate_control
from pepper_gz.wheel_contacts import configure_wheel_contacts
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


def shutdown_with_error(message):
    """Muestra la causa antes de detener los procesos de este lanzamiento."""
    get_logger('pepper_simulation').error(message)
    return [EmitEvent(event=Shutdown(reason=message))]


def guard_instances(
    start_gazebo, publish_tf, start_bridge, startup_controllers, spawn_robot, bridges, world_name,
    start_cmd_vel_guard=False,
):
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
    if not start_gazebo and f'/world/{world_name}/control' not in worlds:
        raise RuntimeError(f'No existe {world_name} en esta GZ_PARTITION')
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
        conflicts = [topic for topic in topics if node.count_publishers(topic)]
        if publish_tf and any('robot_state_publisher' in name for name in node.get_node_names()):
            conflicts.append('robot_state_publisher')
        existing_nodes = set(node.get_node_names())
        if start_cmd_vel_guard and 'cmd_vel_guard' in existing_nodes:
            conflicts.append('cmd_vel_guard: use start_cmd_vel_guard:=false para reutilizarlo')
        for name in startup_controllers:
            if name in existing_nodes:
                conflicts.append(f'{name}: use start_controllers:=false para reutilizarlo')
        if spawn_robot and 'controller_manager' in node.get_node_names():
            conflicts.append('controller_manager: ya existe un robot con control en este dominio')
        if conflicts:
            raise RuntimeError(
                'Componentes ROS ya activos: ' + ', '.join(conflicts)
                + '. Desactive su arranque con los argumentos correspondientes.',
            )
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown(context=context)


def validate_model(robot, config, bridges, world_name):
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
    expected[f'/world/{world_name}/clock'] = ('rosgraph_msgs/msg/Clock', None)
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


def resolve_world(value, share):
    """Resuelve un mundo instalado o una ruta SDF y obtiene su nombre interno."""
    path = Path(value).expanduser()
    if not path.is_absolute():
        path = share / 'worlds' / path
    path = path.resolve(strict=True)
    worlds = ET.parse(path).getroot().findall('world')
    if len(worlds) != 1 or not worlds[0].get('name'):
        raise ValueError(f'{path} debe contener un único world con nombre')
    return path, worlds[0].get('name')


def launch_setup(context):
    """Prepara el SDF y las instancias solicitadas."""
    share = Path(get_package_share_directory('pepper_gz'))
    paused = flag(context, 'paused')
    description = Path(get_package_share_directory('pepper_description'))
    world_path, world_name = resolve_world(LaunchConfiguration('world').perform(context), share)
    spawn_timeout = float(LaunchConfiguration('spawn_timeout').perform(context))
    if not math.isfinite(spawn_timeout) or spawn_timeout <= 0:
        raise ValueError('spawn_timeout debe ser un número positivo de segundos reales')
    config_path = Path(LaunchConfiguration('sensors_config').perform(context))
    config_path = config_path.expanduser().resolve(strict=True)
    bridge_path = Path(LaunchConfiguration('bridge_config').perform(context))
    bridge_path = bridge_path.expanduser().resolve(strict=True)
    controllers_path = Path(LaunchConfiguration('controllers_config').perform(context))
    controllers_path = controllers_path.expanduser().resolve(strict=True)
    initial_positions_path = Path(LaunchConfiguration('initial_positions_config').perform(context))
    initial_positions_path = initial_positions_path.expanduser().resolve(strict=True)
    config = yaml.safe_load(config_path.read_text())
    bridges = yaml.safe_load(bridge_path.read_text())
    for bridge in bridges:
        if bridge['ros_topic_name'] == '/clock':
            bridge['gz_topic_name'] = f'/world/{world_name}/clock'
    robot_xml = xacro.process_file(
        str(share / 'urdf' / 'pepper_gz.xacro'),
        mappings={
            'sensors_config': str(config_path), 'controllers_config': str(controllers_path),
            'initial_positions_config': str(initial_positions_path),
        },
    ).toxml()
    robot = ET.fromstring(robot_xml)
    validate_model(robot, config, bridges, world_name)
    control_plan = validate_control(robot, yaml.safe_load(controllers_path.read_text()))
    start_gazebo = flag(context, 'start_gazebo')
    publish_tf = flag(context, 'publish_tf')
    start_bridge = flag(context, 'start_bridge')
    start_controllers = flag(context, 'start_controllers')
    spawn_robot = flag(context, 'spawn_robot')
    start_cmd_vel_guard = flag(context, 'start_cmd_vel_guard')
    guard_instances(
        start_gazebo, publish_tf, start_bridge,
        control_plan.active + control_plan.inactive if start_controllers else (), spawn_robot,
        bridges, world_name, start_cmd_vel_guard,
    )
    directory = Path(tempfile.mkdtemp(prefix='pepper_gz_'))
    urdf = directory / 'pepper.urdf'
    sdf = directory / 'pepper.sdf'
    resolved_bridge = directory / 'sensors_bridge.yaml'
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
        plugins = model.findall(
            'model/plugin[@name="gz_ros2_control::GazeboSimROS2ControlPlugin"]',
        )
        if len(plugins) != 1 or plugins[0].findtext('parameters') != str(controllers_path):
            raise RuntimeError('La conversión a SDF perdió el plugin o su YAML de control')
        moving = {j.get('name') for j in robot.findall('joint') if j.get('type') != 'fixed'}
        if not moving <= {j.get('name') for j in model.findall('model/joint')}:
            raise RuntimeError('La conversión a SDF perdió articulaciones móviles')
        configure_wheel_contacts(model, yaml.safe_load(Path(
            LaunchConfiguration('wheel_contacts_config').perform(context)).read_text()))
        sdf.write_text(ET.tostring(model, encoding='unicode'))
        resolved_bridge.write_text(yaml.safe_dump(bridges, sort_keys=False))
    except Exception:
        shutil.rmtree(directory)
        raise
    paths = [str(description.parent), str(share.parent), str(world_path.parent),
             os.getenv('GZ_SIM_RESOURCE_PATH', '')]
    environment = {'GZ_SIM_RESOURCE_PATH': os.pathsep.join(p for p in paths if p)}
    actions = []

    def cleanup(_context, *args, **kwargs):
        shutil.rmtree(directory, ignore_errors=True)
        return []

    actions.append(RegisterEventHandler(OnShutdown(
        on_shutdown=[OpaqueFunction(function=cleanup)],
    )))
    if start_gazebo:
        command = ['gz', 'sim', '-v', '3']
        if not paused:
            command.append('-r')
        if not flag(context, 'gui'):
            command += ['-s', '--headless-rendering']
        command += [str(world_path)]
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
        actions.append(Node(
            package='ros_gz_bridge', executable='parameter_bridge',
            name='pepper_sensors_bridge', output='screen',
            parameters=[{'config_file': str(resolved_bridge), 'use_sim_time': True}],
        ))
    if start_cmd_vel_guard:
        actions.append(Node(
            package='pepper_gz', executable='cmd_vel_guard', output='screen',
            parameters=[LaunchConfiguration('teleop_config')],
        ))
    spawner_arguments = [
        '--controller-manager', '/controller_manager',
        '--controller-manager-timeout', '120', '--service-call-timeout', '30',
        '--switch-timeout', '10',
    ]
    groups = []
    spawners = []
    if start_controllers:
        for state, names in startup_groups(control_plan, paused):
            if not names:
                continue
            groups.append((state, names))
            options = ['--activate-as-group'] if state == 'active' else ['--inactive']
            spawners.append(Node(
                package='controller_manager', executable='spawner',
                name=f'pepper_{state}_controller_spawner', output='screen',
                arguments=[*names, *options, *spawner_arguments],
                parameters=[{'use_sim_time': True}],
            ))

    def group_started(event, _context, index):
        if _context.is_shutdown:
            return []
        state, names = groups[index]
        selected = ', '.join(names)
        if event.returncode:
            return shutdown_with_error(
                f'Falló el arranque en estado {state} de: {selected}. Revise el plugin, '
                'los parámetros de esos controladores y sus interfaces en /controller_manager.',
            )
        result = [LogInfo(msg=f'Controladores en estado {state}: {selected}')]
        if index + 1 < len(spawners):
            result.append(spawners[index + 1])
        elif paused and control_plan.active:
            request = yaml.safe_dump({
                'activate_controllers': list(control_plan.active), 'strictness': 2,
                'timeout': {'sec': 10, 'nanosec': 0},
            }, default_flow_style=True, width=10000).strip()
            result.append(LogInfo(msg=(
                'Controladores configurados sin activar. Pulse Play y ejecute: '
                'ros2 service call /controller_manager/switch_controller '
                'controller_manager_msgs/srv/SwitchController ' + "'" + request + "'"
            )))
        return result

    for index, spawner in enumerate(spawners):
        actions.append(RegisterEventHandler(OnProcessExit(
            target_action=spawner, on_exit=partial(group_started, index=index),
        )))
    after_spawn = spawners[:1]
    if spawn_robot:
        z = LaunchConfiguration('spawn_z').perform(context)
        if not math.isfinite(float(z)):
            raise ValueError('spawn_z debe ser finito')
        spawn = Node(
            package='ros_gz_sim', executable='create', name='pepper_spawn', output='screen',
            arguments=['-world', world_name, '-file', str(sdf), '-name', 'pepper',
                       '-allow_renaming', 'false', '-x', '0', '-y', '0', '-z', z],
            parameters=[{'use_sim_time': True}], additional_env=environment,
        )

        def spawned(event, _context):
            watchdog.cancel()
            if _context.is_shutdown:
                return []
            if event.returncode:
                return shutdown_with_error('No se pudo insertar Pepper; revise ros_gz_sim/create')
            return after_spawn

        def insertion_expired(_context):
            return shutdown_with_error(
                f'No terminó la inserción en {spawn_timeout:g} s. Revise el servicio '
                f'/world/{world_name}/create, UserCommands y los errores de gz_ros2_control.',
            )

        watchdog = TimerAction(
            period=spawn_timeout, actions=[OpaqueFunction(function=insertion_expired)],
        )

        actions += [RegisterEventHandler(OnProcessExit(
            target_action=spawn, on_exit=spawned,
        )), RegisterEventHandler(OnProcessStart(
            target_action=spawn, on_start=[watchdog],
        )), spawn]
    else:
        actions += after_spawn
    return actions


def generate_launch_description():
    """Declara opciones para iniciar o reutilizar componentes de simulación."""
    share = Path(get_package_share_directory('pepper_gz'))
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument(
            'paused', default_value='true',
            description='Iniciar en pausa; false ejecuta la simulación sin pulsar Play.',
        ),
        DeclareLaunchArgument('start_gazebo', default_value='true'),
        DeclareLaunchArgument('spawn_robot', default_value='true'),
        DeclareLaunchArgument('publish_tf', default_value='true'),
        DeclareLaunchArgument('start_bridge', default_value='true'),
        DeclareLaunchArgument('start_controllers', default_value='true'),
        DeclareLaunchArgument('start_cmd_vel_guard', default_value='true'),
        DeclareLaunchArgument(
            'teleop_config', default_value=str(share / 'config/base_teleop.yaml')),
        DeclareLaunchArgument(
            'wheel_contacts_config', default_value=str(share / 'config/wheel_contacts.yaml')),
        DeclareLaunchArgument('spawn_z', default_value='0.602'),
        DeclareLaunchArgument('world', default_value='minimal_world.sdf', description=(
            'Archivo de pepper_gz/worlds o ruta absoluta a un SDF con un único mundo.'
        )),
        DeclareLaunchArgument('spawn_timeout', default_value='60.0', description=(
            'Plazo máximo de inserción en segundos reales; create espera al servicio de Gazebo.'
        )),
        DeclareLaunchArgument('sensors_config', default_value=str(share / 'config/sensors.yaml')),
        DeclareLaunchArgument(
            'bridge_config', default_value=str(share / 'config/sensors_bridge.yaml'),
        ),
        DeclareLaunchArgument(
            'controllers_config', default_value=str(share / 'config/controllers.yaml'),
        ),
        DeclareLaunchArgument(
            'initial_positions_config', default_value=str(share / 'config/initial_positions.yaml'),
            description='Posiciones iniciales de las articulaciones actuadas, en radianes.',
        ),
        OpaqueFunction(function=launch_setup),
    ])
