"""Cámara con contexto métrico acotado: RGB, nube, calibración y TF reales."""

from collections import deque, OrderedDict
import math
import threading
import time
import uuid

from nav_msgs.msg import Odometry
import numpy as np
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, PointCloud2
from sensor_msgs_py.point_cloud2 import read_points_numpy
from tf2_ros import Buffer, TransformListener

from .motion import wrap_angle
from .perception import Observation
from .ros2_camera_provider import ROS2CameraProvider
from .target import rigid_transform, target_from_region


def stamp(message):
    return message.header.stamp.sec + message.header.stamp.nanosec*1e-9


def planar_state(message):
    p, q = message.pose.pose.position, message.pose.pose.orientation
    yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
    t = message.twist.twist
    return (p.x, p.y, yaw), math.hypot(t.linear.x, t.linear.y), abs(t.angular.z)


class ROS2TargetCameraProvider(ROS2CameraProvider):
    def __init__(self, node, topic, info_topic, points_topic, odom_topic, base_frame,
                 timeout_sec=3., max_dimension=640, sync_tolerance_sec=.1,
                 max_context_age_sec=60., target_min_z_m=None, target_max_z_m=None):
        super().__init__(node, topic, timeout_sec, max_dimension)
        self.node, self.base_frame = node, base_frame
        self.sync_tolerance, self.max_age = sync_tolerance_sec, max_context_age_sec
        self.height_range = (-np.inf if target_min_z_m is None else target_min_z_m,
                             np.inf if target_max_z_m is None else target_max_z_m)
        if self.height_range[0] >= self.height_range[1]:
            raise ValueError('Intervalo de altura de objetivos inválido')
        self.context_lock = threading.Condition()
        self.clouds, self.odometry = deque(maxlen=16), deque(maxlen=100)
        self.info, self.contexts = None, OrderedDict()
        self.tf = Buffer(cache_time=Duration(seconds=10.))
        self.listener = TransformListener(self.tf, node)
        self.info_sub = node.create_subscription(
            CameraInfo, info_topic, self._info, qos_profile_sensor_data)
        self.cloud_sub = node.create_subscription(
            PointCloud2, points_topic, self._cloud, qos_profile_sensor_data)
        self.odom_sub = node.create_subscription(
            Odometry, odom_topic, self._odom, qos_profile_sensor_data)

    def _info(self, message):
        with self.context_lock:
            self.info = message
            self.context_lock.notify_all()

    def _cloud(self, message):
        with self.context_lock:
            self.clouds.append(message)
            self.context_lock.notify_all()

    def _odom(self, message):
        with self.context_lock:
            self.odometry.append((message, time.monotonic()))
            self.context_lock.notify_all()

    def _transform(self, points, frame, cloud):
        transform = self.tf.lookup_transform(
            frame, cloud.header.frame_id, Time.from_msg(cloud.header.stamp),
            timeout=Duration(seconds=.5)).transform
        p, q = transform.translation, transform.rotation
        return rigid_transform(points, (p.x, p.y, p.z), (q.x, q.y, q.z, q.w))

    def _snapshot(self, message, sensor_stamp):
        def ready():
            return (self.info is not None and self.clouds and self.odometry
                    and min(abs(stamp(c)-sensor_stamp) for c in self.clouds)
                    <= self.sync_tolerance)

        with self.context_lock:
            if not self.context_lock.wait_for(ready, timeout=self.timeout_sec):
                raise RuntimeError('Sin RGB/nube/odometría sincronizados')
            cloud = min(self.clouds, key=lambda c: abs(stamp(c)-sensor_stamp))
            odom, received = min(self.odometry, key=lambda o: abs(stamp(o[0])-sensor_stamp))
            info = self.info
        if (abs(stamp(odom)-sensor_stamp) > self.sync_tolerance
                or time.monotonic()-received > 1.
                or info.header.frame_id != message.header.frame_id
                or info.width != message.width or info.height != message.height
                or odom.child_frame_id != self.base_frame):
            raise RuntimeError('Calibración, frames o sincronización incompatibles')
        if info.distortion_model not in ('', 'plumb_bob') or any(abs(d) > 1e-8 for d in info.d):
            raise RuntimeError('Se necesita imagen rectificada y calibración sin distorsión')
        pose, linear, angular = planar_state(odom)
        if linear > .02 or angular > .03:
            raise RuntimeError('El robot debe estar detenido durante la observación métrica')
        points = read_points_numpy(cloud, field_names=['x', 'y', 'z'], skip_nans=True)
        points = points.reshape((-1, 3))
        points = points[np.isfinite(points).all(axis=1)]
        return {'optical': self._transform(points, message.header.frame_id, cloud),
                'base': self._transform(points, self.base_frame, cloud),
                'camera': (info.k[0], info.k[4], info.k[2], info.k[5],
                           info.width, info.height), 'pose': pose,
                'frames': (odom.header.frame_id, odom.child_frame_id)}

    def make_observation(self, message, received_at, data, sensor_stamp):
        token = uuid.uuid4().hex
        try:
            context = self._snapshot(message, sensor_stamp)
        except Exception as exc:
            # La descripción RGB sigue disponible cuando falta profundidad o TF.
            context = {'error': str(exc)}
        context.update(captured_at=received_at, sensor_stamp=sensor_stamp)
        with self.context_lock:
            self.contexts[token] = context
            while len(self.contexts) > 4:
                self.contexts.popitem(last=False)
        return Observation(data, 'image/png', received_at, self.topic,
                           message.header.frame_id, sensor_stamp, token)

    def locate(self, observation_id, bounds):
        with self.context_lock:
            context = self.contexts.get(observation_id)
            current = self.odometry[-1] if self.odometry else None
        if context is None or current is None:
            raise RuntimeError('Observación desconocida o sin odometría')
        if 'error' in context:
            raise RuntimeError(context['error'])
        now = self.node.get_clock().now().nanoseconds*1e-9
        if abs(now-context['sensor_stamp']) > 3600.:
            raise RuntimeError('El reloj del bridge no coincide con el de la cámara. '
                               'En simulación use use_sim_time:=true; en hardware, '
                               'compruebe la sincronización de relojes.')
        if (not 0 <= time.time()-context['captured_at'] <= self.max_age
                or not -.1 <= now-context['sensor_stamp'] <= self.max_age):
            raise RuntimeError('Observación caducada o reloj reiniciado')
        odom, received = current
        pose, linear, angular = planar_state(odom)
        if (time.monotonic()-received > 1. or not -.1 <= now-stamp(odom) <= 1.
                or (odom.header.frame_id, odom.child_frame_id) != context['frames']
                or math.hypot(pose[0]-context['pose'][0], pose[1]-context['pose'][1]) > .01
                or abs(wrap_angle(pose[2]-context['pose'][2])) > .02
                or linear > .02 or angular > .03):
            raise RuntimeError('La base cambió desde la imagen; solicite otra observación')
        target = target_from_region(context['optical'], context['base'], context['camera'], bounds,
                                    height_range=self.height_range)
        return {**target, 'frame_id': self.base_frame, 'observation_id': observation_id,
                'sensor_stamp': context['sensor_stamp']}
