"""Ensayos abiertos de la base con odometría ROS y pose independiente de Gazebo."""

import argparse
import csv
import gzip
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import signal
import subprocess
import time
import uuid
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from pepper_gz.base_metrics import fitted_parameters, trajectory_metrics
from pepper_gz.omni_kinematics import controller_geometry
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, qos_profile_sensor_data, QoSProfile
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import yaml


def euler(q):
    """Convierte el cuaternión de cada transporte a roll, pitch y yaw."""
    return (math.atan2(2*(q.w*q.x+q.y*q.z), 1-2*(q.x*q.x+q.y*q.y)),
            math.asin(max(-1., min(1., 2*(q.w*q.y-q.z*q.x)))),
            math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))


class BaseRecorder(Node):
    """Registra timestamps nativos de cada fuente; no publica pose ni odometría."""

    def __init__(self, world, model, geometry):
        super().__init__('base_validation_recorder', parameter_overrides=[
            Parameter('use_sim_time', value=True)])
        from gz.msgs10.pose_v_pb2 import Pose_V
        from gz.transport13 import Node as GzNode

        self.sim_time, self.model, self.robot_xml = 0., model, None
        self.truth, self.odom, self.applied, self.wheels = [], [], [], []
        self.odom_frames = None
        self.raw_commands = []
        self.next_command_wall = 0.
        self.last_requested = None
        self.wheel_names = geometry['wheel_names']
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 1)
        self.create_subscription(Clock, '/clock', self.clock, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/base_controller/odom', self.odometry, 10)
        self.create_subscription(TwistStamped, '/base_controller/cmd_vel', self.command, 10)
        self.create_subscription(JointState, '/joint_states', self.joints, qos_profile_sensor_data)
        self.create_subscription(String, '/robot_description', self.description, QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.gz = GzNode()
        self.pose_topic = f'/world/{world}/pose/info'
        if not self.gz.subscribe(Pose_V, self.pose_topic, self.pose):
            raise RuntimeError('No se pudo suscribir a '+self.pose_topic)

    def clock(self, message):
        """Usa el reloj de simulación para duración y progreso de los ensayos."""
        self.sim_time = message.clock.sec+message.clock.nanosec*1e-9

    def description(self, message):
        """Conserva el URDF exacto usado por la simulación para trazabilidad."""
        self.robot_xml = message.data

    def pose(self, message):
        """La verdad terreno proviene directamente de GZ Transport Pose_V."""
        stamp = message.header.stamp.sec+message.header.stamp.nsec*1e-9
        for pose in message.pose:
            if pose.name == self.model:
                self.truth.append([stamp, pose.position.x, pose.position.y, pose.position.z,
                                   *euler(pose.orientation)])

    def odometry(self, message):
        """Conserva pose y velocidad del controlador sin usarlas como verdad terreno."""
        self.odom_frames = (message.header.frame_id, message.child_frame_id)
        p, q, v = message.pose.pose.position, message.pose.pose.orientation, message.twist.twist
        self.odom.append([message.header.stamp.sec+message.header.stamp.nanosec*1e-9,
                          p.x, p.y, p.z, euler(q)[2], v.linear.x, v.linear.y, v.angular.z])

    def command(self, message):
        """Mide el Twist aplicado después de límites y rampas."""
        v = message.twist
        self.applied.append([message.header.stamp.sec+message.header.stamp.nanosec*1e-9,
                             v.linear.x, v.linear.y, v.angular.z])

    def joints(self, message):
        """Registra posición y velocidad de cada rueda para distinguir seguimiento y slip."""
        indices = {name: i for i, name in enumerate(message.name)}
        if not all(name in indices for name in self.wheel_names):
            return
        self.wheels.append([message.header.stamp.sec+message.header.stamp.nanosec*1e-9,
                            *[message.position[indices[name]] for name in self.wheel_names],
                            *[message.velocity[indices[name]] for name in self.wheel_names]])

    def send(self, velocity):
        """La prueba manda solo Twist; controller_manager conserva la autoridad del robot."""
        message = Twist()
        message.linear.x, message.linear.y, message.angular.z = map(float, velocity)
        self.publisher.publish(message)
        self.raw_commands.append([self.sim_time, *map(float, velocity)])


def spin(recorder, process, wall_seconds, velocity=(0., 0., 0.)):
    """Mantiene heartbeat y termina si el proceso de simulación falla."""
    deadline = time.monotonic()+wall_seconds
    while time.monotonic() < deadline:
        if process.poll() is not None:
            raise RuntimeError('Gazebo/launch terminó; consulte simulation.log')
        now = time.monotonic()
        if now >= recorder.next_command_wall or tuple(velocity) != recorder.last_requested:
            recorder.send(velocity)
            recorder.next_command_wall = now+1./30.
            recorder.last_requested = tuple(velocity)
        rclpy.spin_once(recorder, timeout_sec=.02)


def settle(recorder, process, seconds):
    """Espera en tiempo simulado con velocidad cero, con deadline en tiempo real."""
    end, deadline = recorder.sim_time+seconds, time.monotonic()+120.
    while recorder.sim_time < end:
        if time.monotonic() > deadline:
            raise RuntimeError('El reloj no avanzó durante la estabilización')
        spin(recorder, process, .03)


def run_trial(recorder, process, trial, settle_time):
    """Termina por integral del comando aplicado, sin cerrar lazo con la verdad terreno."""
    start, progress = recorder.sim_time, 0.
    index, previous = len(recorder.applied), recorder.applied[-1]
    target = trial.get('distance', trial.get('angle'))
    angular = 'angle' in trial
    velocity = [float(v) for v in trial['velocity']]
    if (not math.isfinite(target) or target <= 0 or len(velocity) != 3
            or not all(math.isfinite(v) for v in velocity)
            or (abs(velocity[2]) if angular else math.hypot(*velocity[:2])) == 0):
        raise ValueError('Ensayo inválido: '+trial['name'])
    deadline = time.monotonic()+120.
    while progress < target:
        if time.monotonic() > deadline:
            raise RuntimeError('No se alcanzó la integral prevista: '+trial['name'])
        spin(recorder, process, .03, velocity)
        for row in recorder.applied[index:]:
            dt = max(0., row[0]-max(start, previous[0]))
            progress += dt*(abs(previous[3]) if angular else math.hypot(*previous[1:3]))
            previous = row
        index = len(recorder.applied)
    stop = recorder.sim_time
    settle(recorder, process, settle_time)
    # Un margen pequeño permite que las dos fuentes cubran el mismo final.
    end = min(recorder.sim_time, recorder.truth[-1][0], recorder.odom[-1][0])
    result, aligned, speeds = trajectory_metrics(
        recorder.truth, recorder.odom, recorder.applied, start, stop, end)
    result.update(name=trial['name'], nominal_distance=trial.get('distance'),
                  nominal_angle=trial.get('angle'), velocity=velocity)
    return result, aligned, speeds


def assess(result, criteria):
    """Evalúa tolerancias explícitas para odometría, seguimiento y estabilidad."""
    endpoint = max(criteria['endpoint_error_m'],
                   criteria['endpoint_error_fraction']*result['reference_path_m'])
    checks = {'endpoint_error_m': endpoint, 'tracking_endpoint_error_m': endpoint,
              'orientation_error_rad': criteria['orientation_error_rad'],
              'tracking_yaw_error_rad': criteria['orientation_error_rad']}
    checks.update({key: criteria.get('max_sample_gap_s', .1) for key in (
        'ground_truth_max_gap_s', 'odom_max_gap_s')})
    checks.update({key: criteria[key] for key in (
        'linear_velocity_rmse_mps', 'angular_velocity_rmse_rps',
        'max_base_tilt_rad', 'base_height_range_m')})
    failures = {key: {'measured': result[key], 'limit': limit} for key, limit in checks.items()
                if abs(result[key]) > limit}
    return failures


def write_csv(path, header, rows):
    """Guarda muestras originales comprimidas, sin redondear sus valores."""
    with gzip.open(path, 'wt', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(header)
        writer.writerows(rows)


def save_recording(output, recorder):
    """Exporta las fuentes originales con sus timestamps y nombres de columnas."""
    write_csv(output/'gazebo_pose.csv.gz', ['t', 'x', 'y', 'z', 'roll', 'pitch', 'yaw'],
              list(recorder.truth))
    write_csv(output/'odometry.csv.gz', ['t', 'x', 'y', 'z', 'yaw', 'vx', 'vy', 'wz'],
              recorder.odom)
    write_csv(output/'applied_cmd.csv.gz', ['t', 'vx', 'vy', 'wz'], recorder.applied)
    write_csv(output/'requested_cmd.csv.gz', ['t', 'vx', 'vy', 'wz'], recorder.raw_commands)
    wheel_header = ['t', *[n+'/position' for n in recorder.wheel_names],
                    *[n+'/velocity' for n in recorder.wheel_names]]
    write_csv(output/'wheel_states.csv.gz', wheel_header, recorder.wheels)
    if recorder.robot_xml:
        (output/'robot_description.urdf').write_text(recorder.robot_xml)


def write_report(output, manifest, results):
    """Genera una tabla de errores sin ocultar ensayos fuera de criterio."""
    lines = ['# Validación de base', '', f"Estado: **{manifest['status']}**.", '',
             'Pose real: GZ Transport; odometría: controlador. Tiempo de simulación.',
             'Velocidades comparadas con una ventana común de 100 ms.', '',
             '| Ensayo | Repetición | Error posición (cm) | Error yaw (°) | '
             'RMSE velocidad (m/s) | RMSE giro (rad/s) | Criterio |',
             '| --- | --- | ---: | ---: | ---: | ---: | --- |']
    for result in results:
        failures = ', '.join(result['failures']) or 'OK'
        lines.append(
            f"| {result['name']} | {result['repeat']} | "
            f"{100*result['endpoint_error_m']:.3f} | "
            f"{math.degrees(result['orientation_error_rad']):.3f} | "
            f"{result['linear_velocity_rmse_mps']:.4f} | "
            f"{result['angular_velocity_rmse_rps']:.4f} | {failures} |")
    lines += ['', 'Los CSV comprimidos conservan las fuentes originales y los timestamps.',
              'summary.json incluye seguimiento de la orden, estabilidad y huecos de muestreo.',
              'manifest.json contiene configuración, versiones y ajuste diagnóstico de radios.',
              'Las sugerencias de ajuste no se aplican automáticamente.', '']
    (output/'report.md').write_text('\n'.join(lines))


def analyze_saved(output):
    """Recalcula métricas desde muestras guardadas, sin iniciar ni mover Gazebo."""
    def read(name):
        with gzip.open(output/name, 'rt', newline='') as stream:
            reader = csv.reader(stream)
            next(reader)
            return [[float(value) for value in row] for row in reader]

    truth, odom, applied = (read(name) for name in (
        'gazebo_pose.csv.gz', 'odometry.csv.gz', 'applied_cmd.csv.gz'))
    manifest = json.loads((output/'manifest.json').read_text())
    plan = yaml.safe_load((output/'plan.yaml').read_text())
    results = []
    for original in json.loads((output/'summary.json').read_text()):
        result, aligned, speeds = trajectory_metrics(
            truth, odom, applied, original['start'], original['stop'], original['end'])
        result.update({key: original[key] for key in (
            'name', 'repeat', 'nominal_distance', 'nominal_angle', 'velocity')})
        result['failures'] = assess(result, plan['criteria'])
        results.append(result)
        label = f"{result['repeat']:02d}_{result['name']}"
        write_csv(output/(label+'_aligned.csv.gz'),
                  ['t', 'gz_x', 'gz_y', 'gz_yaw', 'odom_x', 'odom_y', 'odom_yaw'], aligned)
        write_csv(output/(label+'_velocity.csv.gz'),
                  ['t', 'gz_vx', 'gz_vy', 'gz_wz', 'odom_vx', 'odom_vy', 'odom_wz'], speeds)
    geometry = manifest['geometry']
    manifest['fit'] = fitted_parameters(
        results, geometry['wheel_radius'], geometry['robot_radius'])
    manifest['aggregate'], _, _ = trajectory_metrics(
        truth, odom, applied, results[0]['start'], results[-1]['stop'], results[-1]['end'])
    manifest['status'] = 'passed' if not any(r['failures'] for r in results) else 'failed'
    manifest['velocity_comparison'] = 'centered 100 ms on both sources'
    manifest['analysis_sha256'] = hashlib.sha256(
        Path(__file__).with_name('base_metrics.py').read_bytes()).hexdigest()
    (output/'summary.json').write_text(json.dumps(results, indent=2, allow_nan=False))
    (output/'manifest.json').write_text(json.dumps(manifest, indent=2, allow_nan=False))
    write_report(output, manifest, results)
    print(json.dumps({'status': manifest['status'], 'trials': len(results),
                      'failed_trials': sum(bool(r['failures']) for r in results)}))
    return 0 if manifest['status'] == 'passed' else 2


def main(args=None):
    """Ejecuta una simulación aislada y entrega datos, métricas y criterios reproducibles."""
    share = Path(get_package_share_directory('pepper_gz'))
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--reanalyze', action='store_true',
                        help='Recalcular métricas de --output sin iniciar Gazebo')
    parser.add_argument('--plan', type=Path, default=share/'config/base_validation.yaml')
    parser.add_argument('--controllers-config', type=Path, default=share/'config/controllers.yaml')
    parser.add_argument('--contacts-config', type=Path, default=share/'config/wheel_contacts.yaml')
    parser.add_argument('--sensors-config', type=Path, default=share/'config/sensors.yaml')
    parser.add_argument('--world', default='minimal_world.sdf')
    parser.add_argument('--teleop-config', type=Path, default=share/'config/base_teleop.yaml')
    parser.add_argument('--initial-positions-config', type=Path,
                        default=share/'config/initial_positions.yaml')
    parser.add_argument('--model', default='pepper')
    parser.add_argument('--domain-id', type=int, default=187)
    parser.add_argument('--repetitions', type=int)
    parser.add_argument('--cases', help='Nombres separados por coma; por defecto todos')
    options = parser.parse_args(args)
    output = options.output.expanduser().resolve()
    if options.reanalyze:
        return analyze_saved(output)
    output.mkdir(parents=True, exist_ok=False)
    world_path = Path(options.world).expanduser()
    if not world_path.is_absolute():
        world_path = share/'worlds'/world_path
    worlds = ET.parse(world_path).getroot().findall('world')
    if len(worlds) != 1 or not worlds[0].get('name'):
        raise ValueError('El SDF debe contener un único world con nombre')
    world_name = worlds[0].get('name')
    shutil.copyfile(world_path, output/'world.sdf')
    config = yaml.safe_load(options.controllers_config.read_text())
    geometry = controller_geometry(config, 'base_controller')
    plan = yaml.safe_load(options.plan.read_text())
    trials = plan['trials']
    if options.cases:
        selected = options.cases.split(',')
        if set(selected)-{trial['name'] for trial in trials}:
            raise ValueError('Hay nombres de ensayo desconocidos en --cases')
        trials = [trial for trial in trials if trial['name'] in selected]
    repetitions = options.repetitions if options.repetitions is not None else plan['repetitions']
    if repetitions < 1:
        raise ValueError('--repetitions debe ser positivo')
    os.environ['ROS_DOMAIN_ID'] = str(options.domain_id)
    os.environ['GZ_PARTITION'] = 'pepper_validation_'+uuid.uuid4().hex[:12]
    for name, path in [('controllers.yaml', options.controllers_config),
                       ('wheel_contacts.yaml', options.contacts_config),
                       ('sensors.yaml', options.sensors_config), ('plan.yaml', options.plan),
                       ('base_teleop.yaml', options.teleop_config),
                       ('initial_positions.yaml', options.initial_positions_config),
                       ('sensors_bridge.yaml', share/'config/sensors_bridge.yaml')]:
        shutil.copyfile(path, output/name)
    command = ['ros2', 'launch', 'pepper_gz', 'simulation.launch.py', 'gui:=false',
               'paused:=false', 'world:='+str(output/'world.sdf'),
               'controllers_config:='+str(output/'controllers.yaml'),
               'wheel_contacts_config:='+str(output/'wheel_contacts.yaml'),
               'sensors_config:='+str(output/'sensors.yaml'),
               'teleop_config:='+str(output/'base_teleop.yaml'),
               'initial_positions_config:='+str(output/'initial_positions.yaml'),
               'bridge_config:='+str(output/'sensors_bridge.yaml')]
    manifest = {'command': command, 'ROS_DOMAIN_ID': options.domain_id,
                'GZ_PARTITION': os.environ['GZ_PARTITION'], 'geometry': geometry,
                'ground_truth': 'GZ Transport Pose_V, modelo '+options.model,
                'odometry': '/base_controller/odom', 'status': 'running',
                'physics_engine': worlds[0].findtext(
                    "plugin[@name='gz::sim::systems::Physics']/engine/filename"),
                'max_step_size': worlds[0].findtext('physics/max_step_size'),
                'repetitions': repetitions, 'trial_names': [t['name'] for t in trials],
                'criteria': plan['criteria'], 'settle_time': plan['settle_time'],
                'command_publish_rate_hz': 30.,
                'velocity_comparison': 'centered 100 ms on both sources',
                'analysis_sha256': hashlib.sha256(
                    Path(__file__).with_name('base_metrics.py').read_bytes()).hexdigest(),
                'units': {'time': 'simulation seconds', 'position': 'm', 'angle': 'rad',
                          'linear_velocity': 'm/s', 'angular_velocity': 'rad/s'},
                'config_sha256': {f.name: hashlib.sha256(f.read_bytes()).hexdigest()
                                  for f in output.iterdir() if f.suffix in ('.yaml', '.sdf')}}
    recorder, process, results = None, None, []
    wall_start = time.monotonic()
    log = (output/'simulation.log').open('w')
    try:
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                   start_new_session=True)
        rclpy.init(args=[])
        recorder = BaseRecorder(world_name, options.model, geometry)
        deadline = time.monotonic()+120.
        while not (recorder.truth and recorder.odom and recorder.applied and recorder.wheels
                   and recorder.robot_xml):
            if time.monotonic() > deadline:
                raise RuntimeError('Faltan pose, odometría, reloj, comandos o estados de ruedas')
            spin(recorder, process, .05)
        robot = ET.fromstring(recorder.robot_xml)
        children = {joint.find('child').get('link') for joint in robot.findall('joint')}
        roots = {link.get('name') for link in robot.findall('link')}-children
        if len(roots) != 1 or recorder.odom_frames[1] not in roots:
            raise RuntimeError('El child_frame_id de odom no coincide con la raíz del modelo')
        manifest['model_root_frame'] = next(iter(roots))
        manifest['odometry_frames'] = list(recorder.odom_frames)
        for suffix, arguments in [('list', ['-l']), ('info', ['-i', '-t', recorder.pose_topic])]:
            response = subprocess.run(['gz', 'topic', *arguments], capture_output=True,
                                      text=True, check=True, timeout=10)
            (output/('gz_topic_'+suffix+'.txt')).write_text(response.stdout)
            if suffix == 'info' and 'gz.msgs.Pose_V' not in response.stdout:
                raise RuntimeError('El tópico real de Gazebo no publica gz.msgs.Pose_V')
        settle(recorder, process, 1.)
        for repeat in range(repetitions):
            for trial in trials:
                result, aligned, speeds = run_trial(
                    recorder, process, trial, plan['settle_time'])
                result['repeat'] = repeat+1
                result['failures'] = assess(result, plan['criteria'])
                results.append(result)
                label = f"{repeat+1:02d}_{trial['name']}"
                write_csv(output/(label+'_aligned.csv.gz'),
                          ['t', 'gz_x', 'gz_y', 'gz_yaw', 'odom_x', 'odom_y', 'odom_yaw'], aligned)
                write_csv(output/(label+'_velocity.csv.gz'),
                          ['t', 'gz_vx', 'gz_vy', 'gz_wz', 'odom_vx', 'odom_vy', 'odom_wz'],
                          speeds)
                print(json.dumps({key: result[key] for key in (
                    'name', 'repeat', 'endpoint_error_m', 'orientation_error_rad',
                    'linear_velocity_rmse_mps', 'angular_velocity_rmse_rps', 'failures')}),
                      flush=True)
                (output/'summary.json').write_text(json.dumps(results, indent=2, allow_nan=False))
        manifest['fit'] = fitted_parameters(
            results, geometry['wheel_radius'], geometry['robot_radius'])
        aggregate, _, _ = trajectory_metrics(
            recorder.truth, recorder.odom, recorder.applied, results[0]['start'],
            results[-1]['stop'], results[-1]['end'])
        manifest['aggregate'] = aggregate
        manifest['status'] = 'passed' if not any(r['failures'] for r in results) else 'failed'
    except BaseException as exc:
        manifest['status'], manifest['error'] = 'error', repr(exc)
        raise
    finally:
        if recorder:
            if rclpy.ok():
                recorder.send((0., 0., 0.))
            save_recording(output, recorder)
            manifest['simulation_seconds'] = recorder.sim_time
            manifest['wall_seconds'] = time.monotonic()-wall_start
            if recorder.robot_xml:
                manifest['urdf_sha256'] = hashlib.sha256(recorder.robot_xml.encode()).hexdigest()
            recorder.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if process and process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
        log.close()
        versions = subprocess.run(
            ['dpkg-query', '-W', 'ros-jazzy-gz-physics-vendor', 'ros-jazzy-gz-sim-vendor',
             'ros-jazzy-gz-ros2-control', 'ros-jazzy-omni-wheel-drive-controller'],
            capture_output=True, text=True, timeout=10)
        manifest['package_versions'] = versions.stdout
        (output/'manifest.json').write_text(json.dumps(manifest, indent=2, allow_nan=False))
        write_report(output, manifest, results)
    print(json.dumps(manifest['fit'], indent=2))
    return 0 if manifest['status'] == 'passed' else 2
