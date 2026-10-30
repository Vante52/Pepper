#!/usr/bin/env python3
"""Comprueba tópicos Gazebo, mensajes ROS y frames TF de los sensores."""

import math
from pathlib import Path
import subprocess
import time

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image, Imu, JointState, PointCloud2
from tf2_ros import Buffer, TransformListener
import yaml


TYPES = {
    'sensor_msgs/msg/Image': Image,
    'sensor_msgs/msg/CameraInfo': CameraInfo,
    'sensor_msgs/msg/PointCloud2': PointCloud2,
    'sensor_msgs/msg/Imu': Imu,
    'sensor_msgs/msg/JointState': JointState,
    'rosgraph_msgs/msg/Clock': Clock,
}


class SensorCheck(Node):
    """Espera mediciones reales y comprueba el contrato configurado."""

    def __init__(self):
        super().__init__('pepper_sensor_check')
        share = Path(get_package_share_directory('pepper_gz'))
        self.declare_parameter('bridge_config', str(share / 'config/sensors_bridge.yaml'))
        self.declare_parameter('sensors_config', str(share / 'config/sensors.yaml'))
        self.declare_parameter('timeout', 45.0)
        self.config = yaml.safe_load(Path(self.get_parameter('sensors_config').value).read_text())
        self.bridges = yaml.safe_load(Path(self.get_parameter('bridge_config').value).read_text())
        self.messages = {}
        self.stamps = {}
        self.errors = []
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.subscriptions_ = []
        entries = list(self.bridges) + [{
            'ros_topic_name': '/pepper/camera/depth/points',
            'ros_type_name': 'sensor_msgs/msg/PointCloud2',
            'frame_id': self.config['cameras']['depth']['optical_frame'],
        }]
        self.entries = entries
        for entry in entries:
            topic = entry['ros_topic_name']
            self.subscriptions_.append(self.create_subscription(
                TYPES[entry['ros_type_name']], topic,
                lambda msg, entry=entry: self.receive(entry, msg), qos_profile_sensor_data,
            ))

    def receive(self, entry, message):
        """Valida frames y estructura de las mediciones recibidas."""
        topic = entry['ros_topic_name']
        self.messages[topic] = message
        if hasattr(message, 'header'):
            stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
            previous = self.stamps.setdefault(topic, [])
            if previous and stamp < previous[-1]:
                self.errors.append(f'{topic}: sello de tiempo retrocede')
            if len(previous) < 10:
                previous.append(stamp)
            if entry.get('frame_id') and message.header.frame_id != entry['frame_id']:
                self.errors.append(f'{topic}: frame incorrecto {message.header.frame_id}')
        if isinstance(message, Imu):
            values = [message.angular_velocity.x, message.angular_velocity.y,
                      message.angular_velocity.z, message.linear_acceleration.x,
                      message.linear_acceleration.y, message.linear_acceleration.z]
            if not all(math.isfinite(v) for v in values):
                self.errors.append(f'{topic}: IMU no finita')
        if isinstance(message, JointState):
            if not message.name or len(message.name) != len(message.position):
                self.errors.append(f'{topic}: estados articulares inválidos')
        if isinstance(message, (Image, CameraInfo, PointCloud2)):
            camera = next(c for c in self.config['cameras'].values()
                          if topic.startswith(c['topic'].rsplit('/', 1)[0] + '/'))
            if (message.width, message.height) != (camera['width'], camera['height']):
                self.errors.append(f'{topic}: resolución incorrecta')
            if isinstance(message, Image):
                encoding = '32FC1' if camera['type'] == 'depth_camera' else 'rgb8'
                if message.encoding != encoding:
                    self.errors.append(f'{topic}: encoding incorrecto {message.encoding}')
                if len(message.data) != message.height * message.step:
                    self.errors.append(f'{topic}: tamaño de imagen incorrecto')
            if isinstance(message, CameraInfo):
                expected_fx = camera['width'] / (2 * math.tan(camera['horizontal_fov'] / 2))
                if abs(message.k[0] - expected_fx) > 1.0:
                    self.errors.append(f'{topic}: intrínsecos no corresponden al FOV')

    def check_transport(self, deadline):
        """Usa gz topic -l y -i para verificar publicadores y tipos reales."""
        expected = {b['gz_topic_name']: b['gz_type_name'] for b in self.bridges}
        while time.monotonic() < deadline:
            result = subprocess.run(
                ['gz', 'topic', '-l'], capture_output=True, text=True, timeout=8, check=True,
            )
            missing = set(expected) - set(result.stdout.splitlines())
            if not missing:
                break
            rclpy.spin_once(self, timeout_sec=0.1)
        else:
            raise RuntimeError('Tópicos Gazebo ausentes: ' + ', '.join(sorted(missing)))
        for topic, message_type in expected.items():
            result = subprocess.run(
                ['gz', 'topic', '-i', '-t', topic], capture_output=True,
                text=True, timeout=8, check=True,
            )
            publishers = result.stdout.split('Subscribers')[0].split('No subscribers')[0]
            if publishers.count(message_type) != 1:
                raise RuntimeError(f'{topic}: tipo o cantidad de publicadores incorrectos:\n'
                                   + result.stdout)
            self.get_logger().info(f'{topic}: {message_type}')

    def run(self):
        """Falla si falta un sensor, hay duplicados o los frames no están en TF."""
        deadline = time.monotonic() + self.get_parameter('timeout').value
        self.check_transport(deadline)
        frames = {entry['frame_id'] for entry in self.entries if entry.get('frame_id')}
        frames.add('ImuTorsoGyrometer_frame')
        ready = False
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.errors:
                raise RuntimeError(self.errors[0])
            all_received = len(self.messages) == len(self.entries)
            repeated = all(
                len(self.stamps.get(entry['ros_topic_name'], [])) >= 2
                for entry in self.entries
                if entry['ros_type_name'] != 'rosgraph_msgs/msg/Clock'
            )
            transforms = all(self.buffer.can_transform('base_link', f, rclpy.time.Time())
                             for f in frames)
            if all_received and repeated and transforms:
                ready = True
                break
        if not ready:
            missing = [entry['ros_topic_name'] for entry in self.entries
                       if entry['ros_topic_name'] not in self.messages]
            missing_tf = [f for f in frames if not self.buffer.can_transform(
                'base_link', f, rclpy.time.Time())]
            raise RuntimeError(f'Comprobación incompleta. Mensajes: {missing}; TF: {missing_tf}')
        for entry in self.entries:
            topic = entry['ros_topic_name']
            if self.count_publishers(topic) != 1:
                raise RuntimeError(f'{topic}: se esperaba un único publicador ROS')
            frame = getattr(getattr(self.messages[topic], 'header', None), 'frame_id', '')
            self.get_logger().info(f'ROS {topic}: {entry["ros_type_name"]} {frame}')
        clock = self.messages['/clock'].clock
        if clock.sec == 0 and clock.nanosec == 0:
            raise RuntimeError('El reloj simulado no avanza')
        self.get_logger().info('Sensores, tiempo simulado y TF correctos')


def main(args=None):
    """Ejecuta la comprobación y devuelve error si falla un contrato."""
    rclpy.init(args=args)
    node = None
    result = 0
    try:
        node = SensorCheck()
        node.run()
    except (RuntimeError, ValueError, subprocess.SubprocessError) as error:
        if node is not None:
            node.get_logger().error(str(error))
        result = 1
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
    raise SystemExit(result)


if __name__ == '__main__':
    main()
