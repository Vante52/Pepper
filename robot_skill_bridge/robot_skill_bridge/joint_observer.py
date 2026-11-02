"""Observación del movimiento y sus posiciones finales a partir de estados medidos."""

import math
import threading
import time

from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import JointState

from .trajectory_evidence import MotionEvidence


class JointObserver:
    def __init__(self, node, topic):
        self.condition = threading.Condition()
        self.positions = {}
        self.received = 0.0
        self.evidence = None
        self.subscription = node.create_subscription(
            JointState, topic, self._receive, qos_profile_sensor_data,
        )

    def _receive(self, message):
        if (len(message.name) != len(message.position)
                or len(set(message.name)) != len(message.name)):
            return
        positions = dict(zip(message.name, message.position))
        if not all(math.isfinite(value) for value in positions.values()):
            return
        with self.condition:
            self.positions = positions
            self.received = time.monotonic()
            if self.evidence and self.evidence.initial.keys() <= positions.keys():
                self.evidence.observe(positions)
            self.condition.notify_all()

    def wait(self, joints, after, timeout):
        with self.condition:
            available = self.condition.wait_for(
                lambda: self.received > after and set(joints) <= self.positions.keys(), timeout,
            )
            return {j: self.positions[j] for j in joints} if available else None

    def begin(self, positions):
        with self.condition:
            self.evidence = MotionEvidence(positions)
            return self.evidence

    def end(self):
        with self.condition:
            self.evidence = None
