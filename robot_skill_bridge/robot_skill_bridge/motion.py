"""Objetivos y realimentación planar, independientes del transporte y del robot."""

from dataclasses import dataclass
import math

from .catalog import number


@dataclass(frozen=True)
class MotionProfile:
    cmd_vel_topic: str
    odom_topic: str
    controller: str
    controller_manager: str
    mode: str
    linear_speed: float = .15
    angular_speed: float = .5
    position_tolerance: float = .015
    yaw_tolerance: float = .025
    state_timeout_sec: float = 1.
    stall_timeout_sec: float = 5.
    execution_timeout_sec: float = 150.


def parse_motion(name, data):
    values = {}
    for key in ('cmd_vel_topic', 'odom_topic', 'controller_manager'):
        value = data.get(key)
        if not isinstance(value, str) or not value.startswith('/'):
            raise ValueError(f'{name}: {key} debe ser un nombre absoluto')
        values[key] = value
    values['controller'] = data.get('controller')
    values['mode'] = data.get('mode')
    if not isinstance(values['controller'], str) or not values['controller']:
        raise ValueError(f'{name}: falta controller')
    if values['mode'] not in ('translate', 'rotate'):
        raise ValueError(f'{name}: mode debe ser translate o rotate')
    for key in ('linear_speed', 'angular_speed', 'position_tolerance', 'yaw_tolerance',
                'state_timeout_sec', 'stall_timeout_sec', 'execution_timeout_sec'):
        if key in data:
            values[key] = number(data[key], f'{name}.{key}', positive=True)
    profile = MotionProfile(**values)
    properties = data.get('parameters', {}).get('properties', {})
    required = data.get('parameters', {}).get('required', [])
    expected = ({'distance_m', 'direction_deg'} if profile.mode == 'translate'
                else {'angle_deg'})
    if set(properties) != expected or not expected <= set(required):
        raise ValueError(f'{name}: argumentos requeridos: {sorted(expected)}')
    for key, spec in properties.items():
        if (spec.get('type') != 'number' or 'minimum' not in spec or 'maximum' not in spec
                or not math.isfinite(number(spec['minimum'], key))
                or not math.isfinite(number(spec['maximum'], key))
                or spec['minimum'] >= spec['maximum']):
            raise ValueError(f'{name}: {key} necesita límites numéricos finitos')
        if key == 'distance_m' and spec['minimum'] < 0:
            raise ValueError(f'{name}: distance_m no puede ser negativo')
    return profile


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def relative_goal(initial, distance_m, direction_deg):
    angle = initial[2] + math.radians(direction_deg)
    return (initial[0] + distance_m * math.cos(angle),
            initial[1] + distance_m * math.sin(angle), initial[2])


def feedback_velocity(pose, goal, yaw_error, profile):
    """Error en odometría a Twist en la base, con norma de traslación limitada."""
    dx, dy = goal[0] - pose[0], goal[1] - pose[1]
    distance = math.hypot(dx, dy)
    scale = min(1., profile.linear_speed / max(distance, 1e-12))
    vx = (math.cos(pose[2]) * dx + math.sin(pose[2]) * dy) * scale
    vy = (-math.sin(pose[2]) * dx + math.cos(pose[2]) * dy) * scale
    wz = max(-profile.angular_speed, min(profile.angular_speed, 1.5 * yaw_error))
    if distance <= profile.position_tolerance:
        vx = vy = 0.
    if abs(yaw_error) <= profile.yaw_tolerance:
        wz = 0.
    return (vx, vy, wz), distance <= profile.position_tolerance and wz == 0.
