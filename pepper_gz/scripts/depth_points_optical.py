#!/usr/bin/env python3
"""Expresa la nube de profundidad de Harmonic en el frame óptico existente."""

import copy

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import dtype_from_fields
from tf2_ros import Buffer, TransformException, TransformListener
from tf2_sensor_msgs.tf2_sensor_msgs import transform_points


def transform_cloud(cloud, transform):
    """Transforma XYZ conservando sello de tiempo, campos y organización."""
    result = copy.copy(cloud)
    result.header = copy.copy(cloud.header)
    result.header.frame_id = transform.header.frame_id
    payload = bytearray(cloud.data)
    points = np.ndarray(
        (cloud.height, cloud.width),
        dtype=dtype_from_fields(cloud.fields, point_step=cloud.point_step),
        buffer=payload, strides=(cloud.row_step, cloud.point_step),
    )
    finite = np.isfinite(points['x']) & np.isfinite(points['y']) & np.isfinite(points['z'])
    xyz = np.stack([points[name][finite] for name in ('x', 'y', 'z')], axis=-1)
    transformed = transform_points(xyz, transform.transform)
    for index, name in enumerate(('x', 'y', 'z')):
        points[name][finite] = transformed[:, index]
    result.data = bytes(payload)
    result.is_dense = bool(finite.all())
    return result


class DepthPointsOptical(Node):
    """Usa la TF de CameraDepth_frame a CameraDepth_optical_frame."""

    def __init__(self):
        super().__init__('pepper_depth_points_optical')
        self.declare_parameter('input_topic', '/pepper/camera/depth/_points_sensor')
        self.declare_parameter('output_topic', '/pepper/camera/depth/points')
        self.declare_parameter('source_frame', 'CameraDepth_frame')
        self.declare_parameter('target_frame', 'CameraDepth_optical_frame')
        self.source = self.get_parameter('source_frame').value
        self.target = self.get_parameter('target_frame').value
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.publisher = self.create_publisher(
            PointCloud2, self.get_parameter('output_topic').value, qos_profile_sensor_data,
        )
        self.subscription = self.create_subscription(
            PointCloud2, self.get_parameter('input_topic').value,
            self.on_cloud, qos_profile_sensor_data,
        )

    def on_cloud(self, cloud):
        """Publica coordenadas ópticas, sin limitarse a cambiar la cabecera."""
        if cloud.header.frame_id != self.source:
            self.get_logger().error(
                f'Frame de entrada inesperado: {cloud.header.frame_id}',
                throttle_duration_sec=5.0,
            )
            return
        try:
            transform = self.buffer.lookup_transform(
                self.target, self.source, rclpy.time.Time(),
            )
        except TransformException:
            self.get_logger().warning('Esperando TF de la cámara', throttle_duration_sec=5.0)
            return
        self.publisher.publish(transform_cloud(cloud, transform))


def main(args=None):
    """Inicia el conversor de coordenadas de profundidad."""
    rclpy.init(args=args)
    try:
        node = DepthPointsOptical()
        try:
            rclpy.spin(node)
        finally:
            node.destroy_node()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
