"""Teclado UTF-8 para velocidad de base y controladores articulares configurados."""

import codecs
import math
from pathlib import Path
import select
import signal
import sys
import termios
import time
import tty
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from controller_manager_msgs.msg import ControllerManagerActivity
from geometry_msgs.msg import Twist
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, qos_profile_sensor_data, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import yaml


ARROWS = {'\x1b[A': 'up', '\x1b[B': 'down', '\x1b[C': 'right', '\x1b[D': 'left'}


def decode_keys(buffer):
    """Conserva secuencias de flechas incompletas entre lecturas de terminal."""
    keys = []
    while buffer:
        if buffer.startswith('\x1b'):
            if any(arrow.startswith(buffer) for arrow in ARROWS):
                if buffer in ARROWS:
                    keys.append(ARROWS[buffer])
                    buffer = ''
                break
            sequence = buffer[:3]
            if sequence in ARROWS:
                keys.append(ARROWS[sequence])
                buffer = buffer[3:]
            else:
                buffer = buffer[1:]
        else:
            keys.append(buffer[0].lower())
            buffer = buffer[1:]
    return keys, buffer


def joint_target(target, actual, direction, step, lead, lower, upper):
    """Acota el objetivo por el URDF y por su adelanto respecto al estado medido."""
    return max(lower, actual-lead, min(upper, actual+lead, target+direction*step))


def wheel_limited_velocity(velocity, geometry, wheel_limits):
    """Escala la orden completa sin deformar su dirección ni exceder ninguna rueda."""
    x, y, yaw = velocity
    wheels = geometry['wheel_names']
    scale = 1.
    for index, wheel in enumerate(wheels):
        angle = geometry['wheel_offset'] + index*2.*math.pi/len(wheels)
        speed = (math.sin(angle)*x - math.cos(angle)*y
                 - geometry['robot_radius']*yaw) / geometry['wheel_radius']
        if wheel not in wheel_limits:
            raise ValueError(f'Falta el límite de velocidad URDF de {wheel}')
        if abs(speed) > wheel_limits[wheel]:
            scale = min(scale, wheel_limits[wheel]/abs(speed))
    return x*scale, y*scale, yaw*scale


class KeyboardTeleop(Node):
    """Descubre joints y límites desde YAML y robot_description, sin otra instancia TF."""

    def __init__(self):
        super().__init__('pepper_keyboard_teleop')
        share = Path(get_package_share_directory('pepper_gz'))
        self.declare_parameter('config_file', str(share/'config/keyboard_teleop.yaml'))
        self.declare_parameter('controllers_config', str(share/'config/controllers.yaml'))
        self.config = yaml.safe_load(Path(self.get_parameter('config_file').value).read_text())
        c = self.config
        for key in ('publish_rate', 'key_timeout', 'feedback_timeout', 'linear_speed',
                    'angular_speed', 'joint_step', 'trajectory_duration', 'max_joint_lead'):
            if type(c[key]) not in (float, int) or not math.isfinite(c[key]) or c[key] <= 0:
                raise ValueError(f'{key} debe ser positivo y finito')
        for key, vector in c['base_keys'].items():
            if (not isinstance(key, str) or not isinstance(vector, list) or len(vector) != 3
                    or any(type(v) not in (int, float) or not math.isfinite(v) for v in vector)):
                raise ValueError(f'base_keys inválido: {key}')
        controllers = yaml.safe_load(Path(
            self.get_parameter('controllers_config').value).read_text())
        declarations = controllers['controller_manager']['ros__parameters']
        base_controller = c.get('base_controller', 'base_controller')
        if declarations.get(base_controller, {}).get('type') != (
            'omni_wheel_drive_controller/OmniWheelDriveController'
        ):
            raise ValueError('base_controller debe seleccionar un controlador omni')
        self.base_geometry = controllers[base_controller]['ros__parameters']
        self.groups = {
            name: controllers[name]['ros__parameters']['joints']
            for name, spec in declarations.items() if isinstance(spec, dict)
            and spec.get('type') == 'joint_trajectory_controller/JointTrajectoryController'
        }
        if not self.groups:
            raise ValueError('No hay controladores articulares en controllers_config')
        for key, binding in c['joint_keys'].items():
            if (not isinstance(binding, list) or len(binding) != 3
                    or binding[0] not in self.groups
                    or binding[1] not in self.groups[binding[0]] or binding[2] not in (-1, 1)):
                raise ValueError(f'joint_keys inválido: {key}')
        self.publishers_by_group = {
            name: self.create_publisher(JointTrajectory, '/'+name+'/joint_trajectory', 1)
            for name in self.groups}
        self.base = self.create_publisher(Twist, c['base_topic'], 1)
        self.positions, self.limits, self.targets, self.last_joint_command = {}, {}, {}, {}
        self.active = set()
        self.wheel_limits = {}
        self.feedback_at = None
        self.base_command, self.key_at = (0., 0., 0.), None
        self.group_index, self.joint_index = 0, 0
        transient = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, c['robot_description_topic'], self.description, transient)
        self.create_subscription(ControllerManagerActivity, c['controller_activity_topic'],
                                 self.activity, transient)
        self.create_subscription(JointState, c['joint_states_topic'], self.feedback,
                                 qos_profile_sensor_data)
        self.decoder, self.buffer = codecs.getincrementaldecoder('utf-8')(), ''
        self.create_timer(1./c['publish_rate'], self.tick,
                          clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info(
            'WASD/flechas: avanzar, retroceder y girar; Q/E: lateral; espacio: parar. '
            'Cabeza: K izquierda, Ñ derecha, O arriba, L abajo. '
            'Tab: controlador; [ ]: joint; - +: mover joint. Ctrl+C: salir. '
            'Mantenga la tecla pulsada; se detiene tras el timeout de teclado. '
            f"Base: {c['linear_speed']:g} m/s, giro: {c['angular_speed']:g} rad/s; "
            f"paso articular: {c['joint_step']:g} rad.")
        self.selection()

    def description(self, message):
        """Usa los límites reales sin duplicarlos en el teleoperador."""
        self.limits, self.wheel_limits = {}, {}
        for joint in ET.fromstring(message.data).findall('joint'):
            limit = joint.find('limit')
            if limit is not None and joint.get('type') in ('continuous', 'revolute'):
                velocity = float(limit.get('velocity'))
                if not math.isfinite(velocity) or velocity <= 0:
                    raise ValueError(f"Límite de velocidad inválido: {joint.get('name')}")
                self.wheel_limits[joint.get('name')] = velocity
            if limit is not None and joint.get('type') == 'revolute':
                self.limits[joint.get('name')] = (
                    float(limit.get('lower')), float(limit.get('upper')),
                    float(limit.get('velocity')))

    def feedback(self, message):
        """Registra las posiciones medidas y su edad en tiempo real."""
        self.positions.update(zip(message.name, message.position))
        self.feedback_at = time.monotonic()

    def activity(self, message):
        """Olvida objetivos cuando un controlador cambia de estado."""
        active = {c.name for c in message.controllers if c.state.label == 'active'}
        for name in self.active.symmetric_difference(active):
            self.targets.pop(name, None)
            self.last_joint_command.pop(name, None)
        self.active = active

    def selection(self):
        """Anuncia el joint para el control genérico de postura y brazos."""
        group = list(self.groups)[self.group_index]
        joint = self.groups[group][self.joint_index]
        self.get_logger().info(f'Selección: {group} / {joint}')
        return group, joint

    def move_joint(self, group, joint, direction):
        """Envía el grupo completo preservando los otros objetivos de ese controlador."""
        now = time.monotonic()
        joints = self.groups[group]
        if (group not in self.active or self.feedback_at is None
                or now-self.feedback_at > self.config['feedback_timeout']
                or any(j not in self.positions or j not in self.limits
                       or not math.isfinite(self.positions[j]) for j in joints)):
            self.get_logger().warning(
                f'{group}: requiere controlador activo, URDF y estados recientes. '
                'Compruebe Play y la activación.', throttle_duration_sec=2.)
            return
        if now-self.last_joint_command.get(group, -math.inf) > self.config['feedback_timeout']:
            self.targets[group] = [self.positions[j] for j in joints]
        target = self.targets[group]
        index = joints.index(joint)
        lower, upper, velocity = self.limits[joint]
        duration = self.config['trajectory_duration']
        step = min(self.config['joint_step'], velocity*duration*.5)
        target[index] = joint_target(target[index], self.positions[joint], direction,
                                     step, min(self.config['max_joint_lead'], velocity*duration),
                                     lower, upper)
        message = JointTrajectory()
        message.joint_names = joints
        point = JointTrajectoryPoint()
        point.positions = list(target)
        point.time_from_start.sec, point.time_from_start.nanosec = divmod(
            round(duration*1e9), 1_000_000_000)
        message.points = [point]
        self.publishers_by_group[group].publish(message)
        self.last_joint_command[group] = now

    def handle_key(self, key):
        """Traduce una tecla mediante los mapas de configuración."""
        if key == ' ':
            self.base_command, self.key_at = (0., 0., 0.), None
        elif key in self.config['base_keys']:
            x, y, yaw = self.config['base_keys'][key]
            requested = (x*self.config['linear_speed'], y*self.config['linear_speed'],
                         yaw*self.config['angular_speed'])
            try:
                self.base_command = wheel_limited_velocity(
                    requested, self.base_geometry, self.wheel_limits)
                self.key_at = time.monotonic()
            except ValueError as exc:
                self.base_command, self.key_at = (0., 0., 0.), None
                self.get_logger().warning(str(exc), throttle_duration_sec=2.)
        elif key in self.config['joint_keys']:
            self.move_joint(*self.config['joint_keys'][key])
        elif key == '\t':
            self.group_index = (self.group_index+1) % len(self.groups)
            self.joint_index = 0
            self.selection()
        elif key in ('[', ']'):
            group = list(self.groups)[self.group_index]
            self.joint_index = (self.joint_index+(1 if key == ']' else -1)) % len(
                self.groups[group])
            self.selection()
        elif key in ('-', '+', '='):
            self.move_joint(*self.selection(), -1 if key == '-' else 1)

    def tick(self):
        """Lee sin bloquear ROS y repite la orden hasta que caduque en tiempo real."""
        import os

        while select.select([sys.stdin], [], [], 0)[0]:
            data = os.read(sys.stdin.fileno(), 128)
            if not data:
                raise EOFError('La terminal de teleop se cerró')
            self.buffer += self.decoder.decode(data)
        keys, self.buffer = decode_keys(self.buffer)
        for key in keys:
            self.handle_key(key)
        live = (self.key_at is not None
                and time.monotonic()-self.key_at < self.config['key_timeout'])
        x, y, yaw = self.base_command if live else (0., 0., 0.)
        message = Twist()
        message.linear.x, message.linear.y, message.angular.z = x, y, yaw
        self.base.publish(message)

    def stop(self):
        """Detiene la base antes de cerrar la terminal."""
        self.base.publish(Twist())


def main(args=None):
    """Restaura siempre la terminal, incluso ante errores o Ctrl+C."""
    if not sys.stdin.isatty():
        raise SystemExit('Ejecute keyboard_teleop directamente en una terminal interactiva.')
    original = termios.tcgetattr(sys.stdin)
    rclpy.init(args=args)
    node = None
    try:
        node = KeyboardTeleop()
        tty.setcbreak(sys.stdin.fileno())
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException, EOFError):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, original)
        if node:
            if rclpy.ok():
                node.stop()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
