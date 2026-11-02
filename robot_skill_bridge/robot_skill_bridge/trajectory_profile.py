"""Perfil de FollowJointTrajectory; no forma parte del contrato público."""

from dataclasses import dataclass

from .catalog import number


@dataclass(frozen=True)
class Waypoint:
    time: float
    positions: tuple[float, ...]


@dataclass(frozen=True)
class TrajectoryProfile:
    action: str
    controller: str
    controller_manager: str
    joint_states_topic: str
    joints: tuple[str, ...]
    waypoints: tuple[Waypoint, ...]
    limits: dict
    goal_tolerance_rad: float
    goal_time_tolerance_sec: float
    min_motion_rad: float
    state_timeout_sec: float
    execution_timeout_sec: float


def parse_trajectory(name, data):
    if data.get('kind', 'action') != 'action':
        raise ValueError(f'{name}: una trayectoria debe tener kind=action')
    action = data.get('action')
    if not isinstance(action, str) or not action.startswith('/'):
        raise ValueError(f'{name}: action debe ser un nombre ROS absoluto')
    joints = data.get('joints')
    if (not isinstance(joints, list) or not joints
            or not all(isinstance(j, str) and j for j in joints)
            or len(joints) != len(set(joints))):
        raise ValueError(f'{name}: joints no válidos o duplicados')
    limits = data.get('limits')
    if not isinstance(limits, dict) or set(limits) != set(joints):
        raise ValueError(f'{name}: limits debe cubrir exactamente sus joints')
    parsed_limits = {}
    for j in joints:
        bounds = limits[j]
        if not isinstance(bounds, list) or len(bounds) != 2:
            raise ValueError(f'{name}: límites incompletos para {j}')
        low = number(bounds[0], f'{j}.lower')
        high = number(bounds[1], f'{j}.upper')
        if low >= high:
            raise ValueError(f'{name}: límites invertidos para {j}')
        parsed_limits[j] = (low, high)
    raw_points = data.get('waypoints')
    if not isinstance(raw_points, list) or not (1 <= len(raw_points) <= 30):
        raise ValueError(f'{name}: waypoints debe tener de 1 a 30 puntos')
    waypoints = []
    previous = 0.0
    for idx, point in enumerate(raw_points):
        if not isinstance(point, dict):
            raise ValueError(f'{name}: punto {idx} incorrecto')
        t = number(point.get('time'), f'{name}.time[{idx}]', positive=True)
        if t <= previous:
            raise ValueError(f'{name}: tiempos no estrictamente crecientes')
        previous = t
        positions = point.get('positions')
        if not isinstance(positions, list) or len(positions) != len(joints):
            raise ValueError(f'{name}: punto {idx} no contiene todos los joints')
        values = []
        for joint, value in zip(joints, positions):
            p = number(value, f'{name}.{joint}[{idx}]')
            low, high = parsed_limits[joint]
            if not low <= p <= high:
                raise ValueError(f'{name}: {joint} fuera de límites en punto {idx}')
            values.append(p)
        waypoints.append(Waypoint(t, tuple(values)))
    if previous > 60:
        raise ValueError(f'{name}: duración superior a 60 segundos')
    tolerance = number(data.get('goal_tolerance_rad', 0.15), 'goal_tolerance_rad', positive=True)
    if tolerance > 0.5:
        raise ValueError(f'{name}: goal_tolerance_rad demasiado alta')
    extra_time = number(data.get('goal_time_tolerance_sec', 2),
                        'goal_time_tolerance_sec', positive=True)
    controller = data.get('controller')
    if not isinstance(controller, str) or not controller or '/' in controller:
        raise ValueError(f'{name}: controller debe identificar el controlador')
    manager = data.get('controller_manager', '/controller_manager')
    topic = data.get('joint_states_topic', '/joint_states')
    for label, value in [('controller_manager', manager), ('joint_states_topic', topic)]:
        if not isinstance(value, str) or not value.startswith('/'):
            raise ValueError(f'{name}: {label} debe ser absoluto')
    motion = number(data.get('min_motion_rad', 0.02), f'{name}.min_motion_rad')
    if motion < 0:
        raise ValueError(f'{name}: min_motion_rad debe ser >= 0')
    state_timeout = number(data.get('state_timeout_sec', 3.0), 'state_timeout_sec', positive=True)
    timeout = number(data.get('execution_timeout_sec', previous + extra_time + 8),
                     'execution_timeout_sec', positive=True)
    return TrajectoryProfile(
        action, controller, manager, topic, tuple(joints), tuple(waypoints), parsed_limits,
        tolerance, extra_time, motion, state_timeout, timeout,
    )
