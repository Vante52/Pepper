"""Adapta Twist a TwistStamped con límites y caducidad en tiempo real."""

import math
import signal
import time

from controller_manager_msgs.msg import ControllerManagerActivity
from geometry_msgs.msg import Twist, TwistStamped
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile


def limited_velocity(message, max_linear, max_angular):
    """Rechaza datos no planares y limita XY conservando su dirección."""
    values = (message.linear.x, message.linear.y, message.linear.z,
              message.angular.x, message.angular.y, message.angular.z)
    if not all(math.isfinite(v) for v in values):
        raise ValueError('cmd_vel contiene valores no finitos')
    if any(v != 0. for v in (values[2], values[3], values[4])):
        raise ValueError('La base solo admite linear.x, linear.y y angular.z')
    scale = min(1., max_linear / max(math.hypot(values[0], values[1]), 1e-12))
    return values[0]*scale, values[1]*scale, max(-max_angular, min(max_angular, values[5]))


class CmdVelGuard(Node):
    """Publica solo cuando el controlador está activo y la orden no ha caducado."""

    def __init__(self):
        super().__init__('cmd_vel_guard')
        defaults = {
            'input_topic': '/cmd_vel', 'output_topic': '/base_controller/cmd_vel',
            'base_frame': 'base_link', 'controller_manager': '/controller_manager',
            'controller_name': 'base_controller', 'publish_rate': 30., 'command_timeout': .3,
            'max_linear_speed': .50, 'max_angular_speed': 1.25,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value, ParameterDescriptor(read_only=True))
        self.settings = {name: self.get_parameter(name).value for name in defaults}
        for name, default in defaults.items():
            value = self.settings[name]
            if isinstance(default, str):
                if not isinstance(value, str) or not value.strip():
                    raise ValueError(f'{name} debe ser texto no vacío')
            elif type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
                raise ValueError(f'{name} debe ser positivo y finito')
        if self.resolve_topic_name(self.settings['input_topic']) == self.resolve_topic_name(
            self.settings['output_topic']
        ):
            raise ValueError('Los tópicos de entrada y salida deben ser distintos')
        self.command, self.received_at, self.controller_active = (0., 0., 0.), None, False
        self.publisher = self.create_publisher(TwistStamped, self.settings['output_topic'], 1)
        self.create_subscription(Twist, self.settings['input_topic'], self.receive, 1)
        self.create_subscription(
            ControllerManagerActivity,
            self.settings['controller_manager'].rstrip('/')+'/activity', self.activity,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_timer(1./self.settings['publish_rate'], self.publish,
                          clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info('Esperando base activa; entrada '+self.settings['input_topic'])

    def activity(self, message):
        """Descarta órdenes de otra activación del controlador."""
        active = any(c.name == self.settings['controller_name'] and c.state.label == 'active'
                     for c in message.controllers)
        if active != self.controller_active:
            self.command, self.received_at = (0., 0., 0.), None
            self.get_logger().info('Base '+('activa' if active else 'inactiva'))
        self.controller_active = active

    def receive(self, message):
        """Recibe una velocidad en el frame de la base."""
        if not self.controller_active:
            self.get_logger().warning('Base inactiva: pulse Play y active los controladores.',
                                      throttle_duration_sec=5.)
            return
        try:
            self.command = limited_velocity(message, self.settings['max_linear_speed'],
                                            self.settings['max_angular_speed'])
            self.received_at = time.monotonic()
        except ValueError as exc:
            self.command, self.received_at = (0., 0., 0.), None
            self.get_logger().warning(str(exc), throttle_duration_sec=2.)

    def publish(self):
        """Usa el reloj monotónico incluso cuando /clock está detenido."""
        if not self.controller_active:
            return
        fresh = (self.received_at is not None
                 and time.monotonic()-self.received_at < self.settings['command_timeout'])
        if self.count_publishers(self.settings['input_topic']) > 1:
            fresh = False
            self.get_logger().warning('Varios emisores en cmd_vel: base detenida; use un mux.',
                                      throttle_duration_sec=5.)
        x, y, yaw = self.command if fresh else (0., 0., 0.)
        message = TwistStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.settings['base_frame']
        message.twist.linear.x, message.twist.linear.y = x, y
        message.twist.angular.z = yaw
        self.publisher.publish(message)

    def stop(self):
        """Envía una última orden de parada."""
        self.command, self.received_at = (0., 0., 0.), None
        self.publish()


def main(args=None):
    """Ejecuta el adaptador y libera sus recursos al salir."""
    rclpy.init(args=args)
    node = None
    try:
        node = CmdVelGuard()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        if node:
            if rclpy.ok():
                node.stop()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
