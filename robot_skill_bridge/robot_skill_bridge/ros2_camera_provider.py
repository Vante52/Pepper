"""Adaptador ROS 2: una imagen nueva por adquisición, sin publicar TF."""

import threading
import time

from PIL import Image

from cv_bridge import CvBridge

from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image as RosImage

from .image_file_provider import png_bytes
from .perception import Observation


class ROS2CameraProvider:
    def __init__(self, node, topic, timeout_sec=3., max_dimension=640):
        self.topic, self.timeout_sec = topic, timeout_sec
        self.max_dimension = max_dimension
        self.condition = threading.Condition()
        self.message, self.received_at = None, 0.
        self.bridge = CvBridge()
        self.subscription = node.create_subscription(
            RosImage, topic, self._receive, qos_profile_sensor_data,
        )

    def _receive(self, message):
        with self.condition:
            self.message, self.received_at = message, time.time()
            self.condition.notify_all()

    def observe(self):
        with self.condition:
            previous = self.message
            if not self.condition.wait_for(
                lambda: self.message is not previous, timeout=self.timeout_sec,
            ):
                raise RuntimeError(f'No llega una imagen nueva de {self.topic}')
            message, received_at = self.message, self.received_at
        try:
            rgb = self.bridge.imgmsg_to_cv2(message, desired_encoding='rgb8')
            data = png_bytes(Image.fromarray(rgb), self.max_dimension)
        except Exception as exc:
            raise ValueError(f'Imagen ROS inválida ({message.encoding}): {exc}') from exc
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        return Observation(data, 'image/png', received_at, self.topic,
                           message.header.frame_id, stamp)
