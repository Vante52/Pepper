"""Cinemática y saturación proporcional omni, independientes de nodos y transporte."""

from collections.abc import Mapping, Sequence
import math
from typing import Any
import xml.etree.ElementTree as ET


Velocity = tuple[float, float, float]


def _finite(value, name, positive=False):
    if (type(value) not in (int, float) or not math.isfinite(value)
            or (positive and value <= 0)):
        raise ValueError(f'{name} debe ser un número finito'+(' positivo' if positive else ''))
    return float(value)


def _geometry(geometry):
    names = geometry.get('wheel_names')
    if (not isinstance(names, (list, tuple)) or len(names) < 3
            or not all(isinstance(n, str) and n.strip() for n in names)
            or len(set(names)) != len(names)):
        raise ValueError('wheel_names requiere al menos tres nombres distintos')
    return (names, _finite(geometry.get('wheel_offset'), 'wheel_offset'),
            _finite(geometry.get('robot_radius'), 'robot_radius', positive=True),
            _finite(geometry.get('wheel_radius'), 'wheel_radius', positive=True))


def _parameters(config, name):
    keys = [key for key in (name, '/'+name) if key in config]
    if len(keys) != 1:
        raise ValueError(f'Configuración ausente o ambigua para {name}')
    section = config[keys[0]]
    params = section.get('ros__parameters') if isinstance(section, Mapping) else None
    if not isinstance(params, Mapping):
        raise ValueError(f'{name}.ros__parameters debe ser un diccionario')
    return params


def controller_geometry(config: Mapping[str, Any], name: str) -> dict[str, Any]:
    """Obtiene la geometría del controlador seleccionado en controllers.yaml."""
    declaration = _parameters(config, 'controller_manager').get(name, {})
    if (not isinstance(declaration, Mapping) or declaration.get('type') !=
            'omni_wheel_drive_controller/OmniWheelDriveController'):
        raise ValueError(f'{name} debe seleccionar un controlador omni')
    params = _parameters(config, name)
    names, offset, radius, wheel_radius = _geometry(params)
    return {'wheel_names': list(names), 'wheel_offset': offset,
            'robot_radius': radius, 'wheel_radius': wheel_radius}


def wheel_limits_from_urdf(robot_xml: str, wheel_names: Sequence[str]) -> dict[str, float]:
    """Exige un límite de velocidad positivo para cada rueda configurada en el URDF."""
    try:
        robot = ET.fromstring(robot_xml)
    except ET.ParseError as exc:
        raise ValueError(f'robot_description no es XML válido: {exc}') from exc
    if robot.tag != 'robot':
        raise ValueError('robot_description debe contener un URDF robot')
    limits = {}
    for name in wheel_names:
        joints = [j for j in robot.findall('joint') if j.get('name') == name]
        if len(joints) != 1 or joints[0].get('type') not in ('continuous', 'revolute'):
            raise ValueError(f'Rueda URDF ausente, duplicada o no rotacional: {name}')
        limit = joints[0].find('limit')
        try:
            value = float(limit.get('velocity')) if limit is not None else None
        except (ValueError, TypeError) as exc:
            raise ValueError(f'Límite de velocidad URDF inválido: {name}') from exc
        limits[name] = _finite(value, f'Límite de velocidad URDF de {name}', positive=True)
    return limits


def wheel_velocities(velocity: Sequence[float], geometry: Mapping[str, Any]) -> dict[str, float]:
    """Calcula rad/s según el orden circular y los parámetros del controlador omni."""
    if len(velocity) != 3:
        raise ValueError('La velocidad debe contener vx, vy y wz')
    x, y, yaw = (_finite(v, axis) for v, axis in zip(velocity, ('vx', 'vy', 'wz')))
    names, offset, radius, wheel_radius = _geometry(geometry)
    speeds = {}
    for index, name in enumerate(names):
        angle = offset + index*2.*math.pi/len(names)
        speed = (math.sin(angle)*x - math.cos(angle)*y - radius*yaw)/wheel_radius
        speeds[name] = _finite(speed, f'Velocidad calculada de {name}')
    return speeds


def wheel_limited_velocity(
    velocity: Sequence[float], geometry: Mapping[str, Any], wheel_limits: Mapping[str, float],
) -> Velocity:
    """Aplica un único factor a vx, vy y wz para respetar todas las ruedas."""
    scale = 1.
    for wheel, speed in wheel_velocities(velocity, geometry).items():
        if wheel not in wheel_limits:
            raise ValueError(f'Falta el límite de velocidad URDF de {wheel}')
        limit = _finite(wheel_limits[wheel], f'Límite de {wheel}', positive=True)
        if abs(speed) > limit:
            scale = min(scale, limit/abs(speed))
    return velocity[0]*scale, velocity[1]*scale, velocity[2]*scale
