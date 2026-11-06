"""Contrato de percepción independiente de ROS, del modelo y del robot."""

import base64
from dataclasses import dataclass
from typing import Protocol

from .skill_result import SkillResult


@dataclass(frozen=True)
class Observation:
    image: bytes
    media_type: str
    captured_at: float
    source: str
    frame_id: str = ''
    sensor_stamp: float | None = None
    observation_id: str = ''

    def to_dict(self):
        result = {
            'image_base64': base64.b64encode(self.image).decode('ascii'),
            'media_type': self.media_type, 'captured_at': self.captured_at,
            'source': self.source, 'frame_id': self.frame_id, 'sensor_stamp': self.sensor_stamp,
        }
        if self.observation_id:
            result['observation_id'] = self.observation_id
        return result


class PerceptionProvider(Protocol):
    def observe(self) -> Observation:
        """Obtiene una imagen; falla explícitamente si no hay observación válida."""
        ...


class MockPerceptionProvider:
    def __init__(self, observations):
        self.observations = iter(observations)

    def observe(self):
        try:
            return next(self.observations)
        except StopIteration as exc:
            raise RuntimeError('No quedan observaciones simuladas') from exc


def parse_perception(name, data):
    provider = data.get('provider')
    if not isinstance(provider, str) or not provider.strip():
        raise ValueError(f'{name}: falta provider')
    if data.get('kind') != 'perception':
        raise ValueError(f'{name}: kind debe ser perception')
    return provider


class PerceptionBackend:
    def __init__(self, providers):
        self.providers = providers

    def execute(self, skill, parameters=None, cancel=None):
        try:
            observation = self.providers[skill.profile].observe()
            return SkillResult(True, 'observed', skill.name,
                               data={'observation': observation.to_dict()})
        except (RuntimeError, OSError, ValueError, KeyError) as exc:
            return SkillResult(False, 'perception_unavailable', skill.name, str(exc))
