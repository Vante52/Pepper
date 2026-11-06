"""
Valida contratos URDF/YAML antes de que ros2_control configure los plugins.

Los validadores describen los recursos de cada tipo, no articulaciones del robot.
Un tipo nuevo requiere un validador en VALIDATORS, sin modificar el lanzamiento.
controller_manager conserva la validación nativa completa de parámetros y plugins.
"""

from dataclasses import dataclass
import math
import re


@dataclass(frozen=True)
class Interfaces:
    """Recursos de articulaciones exportados por ros2_control."""

    commands: frozenset
    states: frozenset


@dataclass(frozen=True)
class Controller:
    """Contrato de recursos de un controlador declarado en el YAML."""

    name: str
    plugin_type: str
    joints: tuple
    interfaces: Interfaces


@dataclass(frozen=True)
class ControlPlan:
    """Controladores validados y política explícita de arranque."""

    controllers: dict
    active: tuple
    inactive: tuple


def startup_groups(plan, paused):
    """Carga sin activar si la física aún no está avanzando."""
    if paused:
        selected = plan.active + plan.inactive
        return [('inactive', selected)] if selected else []
    return [(state, names) for state, names in (
        ('active', plan.active), ('inactive', plan.inactive),
    ) if names]


def _mapping(value, label):
    if not isinstance(value, dict):
        raise ValueError(f'{label}: se esperaba un diccionario')
    return value


def _names(value, label, required=True):
    if not isinstance(value, list) or any(not isinstance(n, str) or not n.strip() for n in value):
        raise ValueError(f'{label}: se esperaba una lista de nombres')
    if required and not value:
        raise ValueError(f'{label}: la lista no puede estar vacía')
    if len(set(value)) != len(value):
        raise ValueError(f'{label}: contiene nombres duplicados')
    return tuple(value)


def _number(value, label, minimum=None, positive=False):
    if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value):
        raise ValueError(f'{label}: se esperaba un número finito')
    if (positive and value <= 0) or (minimum is not None and value < minimum):
        raise ValueError(f'{label}: valor fuera del rango permitido')
    return value


def _boolean(params, key, default=False):
    value = params.get(key, default)
    if not isinstance(value, bool):
        raise ValueError(f'{key}: se esperaba un booleano')
    return value


def _parameters(config, node):
    keys = [key for key in (node, '/' + node) if key in config]
    if len(keys) > 1:
        raise ValueError(f'Configuración ambigua para {node}')
    if not keys:
        return {}
    section = _mapping(config[keys[0]], node)
    return _mapping(section.get('ros__parameters', {}), node + '.ros__parameters')


def _resources(joints, commands=(), states=()):
    return Interfaces(
        frozenset((joint, interface) for joint in joints for interface in commands),
        frozenset((joint, interface) for joint in joints for interface in states),
    )


def _trajectory(params, available):
    joints = _names(params.get('joints'), 'joints')
    commands = _names(params.get('command_interfaces'), 'command_interfaces')
    states = _names(params.get('state_interfaces'), 'state_interfaces')
    valid_commands = {
        frozenset(['position']), frozenset(['velocity']), frozenset(['effort']),
        frozenset(['position', 'velocity']), frozenset(['position', 'effort']),
        frozenset(['position', 'velocity', 'acceleration']),
    }
    if frozenset(commands) not in valid_commands:
        raise ValueError('combinación de command_interfaces no válida para trayectoria')
    if frozenset(states) not in (
        {'position'}, {'position', 'velocity'}, {'position', 'velocity', 'acceleration'},
    ):
        raise ValueError('combinación de state_interfaces no válida para trayectoria')
    if set(commands) & {'effort', 'acceleration'} or commands == ('velocity',):
        if not {'position', 'velocity'} <= set(states):
            raise ValueError('esta trayectoria requiere estados position y velocity')
    command_joints = _names(params.get('command_joints', []), 'command_joints', False) or joints
    if not set(command_joints) <= set(joints):
        raise ValueError('command_joints debe ser un subconjunto de joints')
    for key in (
        'allow_partial_joints_goal', 'interpolate_from_desired_state',
        'set_last_command_interface_value_as_state_on_activation', 'open_loop_control',
        'allow_integration_in_goal_trajectories', 'allow_nonzero_velocity_at_trajectory_end',
    ):
        if key in params:
            _boolean(params, key)
    if params.get('interpolation_method', 'splines') not in ('splines', 'none'):
        raise ValueError('interpolation_method debe ser splines o none')
    if 'action_monitor_rate' in params:
        _number(params['action_monitor_rate'], 'action_monitor_rate', minimum=0.1)
    if 'cmd_timeout' in params:
        _number(params['cmd_timeout'], 'cmd_timeout', minimum=0)
    for key in ('gains', 'constraints'):
        if key in params:
            values = _mapping(params[key], key)
            for joint, settings in values.items():
                if isinstance(settings, dict) and joint not in joints:
                    raise ValueError(f'{key}: articulación ajena al controlador: {joint}')
                if key == 'gains' or joint in joints:
                    _mapping(settings, f'{key}.{joint}')
                if key == 'gains':
                    for gain in ('p', 'i', 'd', 'ff_velocity_scale'):
                        if gain in settings:
                            _number(settings[gain], f'gains.{joint}.{gain}')
    requirements = _resources(command_joints, commands).commands
    return joints, Interfaces(requirements, _resources(joints, states=states).states)


def _broadcaster(params, available):
    joints = _names(params.get('joints', []), 'joints', False)
    extras = _names(params.get('extra_joints', []), 'extra_joints', False)
    states = _names(params.get('interfaces', []), 'interfaces', False)
    _boolean(params, 'use_local_topics')
    if 'command_interfaces' in params or 'state_interfaces' in params:
        raise ValueError('JointStateBroadcaster utiliza interfaces y no acepta comandos')
    if joints and states:
        if set(joints) & set(extras):
            raise ValueError('extra_joints duplica articulaciones seleccionadas')
        return joints + extras, _resources(joints, states=states)
    if joints and any(j not in {name for name, _ in available.states} for j in joints):
        raise ValueError('joints contiene articulaciones sin estados en ros2_control')
    if not available.states:
        raise ValueError('no hay estados disponibles para el broadcaster')
    missing = set(states) - {interface for _, interface in available.states}
    if missing:
        raise ValueError(f'interfaces de estado no disponibles: {sorted(missing)}')
    if set(extras) & {name for name, _ in available.states}:
        raise ValueError('extra_joints duplica articulaciones con estados disponibles')
    # El plugin solicita todos los estados cuando cualquiera de las dos listas está vacía.
    return joints + extras, Interfaces(frozenset(), available.states)


def _forward(params, available, fixed_interface=None):
    joints = _names(params.get('joints'), 'joints')
    interface = params.get('interface_name', fixed_interface)
    if not isinstance(interface, str) or not interface.strip():
        raise ValueError('interface_name debe ser un nombre de interfaz')
    if fixed_interface is not None and interface != fixed_interface:
        raise ValueError(f'este tipo utiliza la interfaz fija {fixed_interface}')
    if 'command_interfaces' in params or 'state_interfaces' in params:
        raise ValueError('este tipo utiliza interface_name y no solicita estados')
    return joints, _resources(joints, commands=[interface])


def _multi_forward(params, available):
    joint = params.get('joint')
    if not isinstance(joint, str) or not joint.strip():
        raise ValueError('joint debe nombrar una articulación')
    interfaces = _names(params.get('interface_names'), 'interface_names')
    return (joint,), _resources([joint], commands=interfaces)


def _diff_drive(params, available):
    left = _names(params.get('left_wheel_names'), 'left_wheel_names')
    right = _names(params.get('right_wheel_names'), 'right_wheel_names')
    if len(left) != len(right) or set(left) & set(right):
        raise ValueError('las listas de ruedas deben ser disjuntas y del mismo tamaño')
    for key in ('wheel_radius', 'wheel_separation'):
        _number(params.get(key), key, positive=True)
    for key in ('wheel_separation_multiplier', 'left_wheel_radius_multiplier',
                'right_wheel_radius_multiplier'):
        if key in params:
            _number(params[key], key, positive=True)
    open_loop = _boolean(params, 'open_loop')
    feedback = 'position' if _boolean(params, 'position_feedback', True) else 'velocity'
    joints = left + right
    return joints, _resources(
        joints, commands=['velocity'], states=[] if open_loop else [feedback],
    )


def _omni_drive(params, available):
    joints = _names(params.get('wheel_names'), 'wheel_names')
    if len(joints) < 3:
        raise ValueError('omni requiere al menos tres ruedas')
    for key in ('robot_radius', 'wheel_radius', 'cmd_vel_timeout'):
        _number(params.get(key), key, positive=True)
    _number(params.get('wheel_offset'), 'wheel_offset')
    feedback = 'position' if _boolean(params, 'position_feedback', True) else 'velocity'
    states = [] if _boolean(params, 'open_loop') else [feedback]
    return joints, _resources(joints, commands=['velocity'], states=states)


VALIDATORS = {
    'joint_trajectory_controller/JointTrajectoryController': _trajectory,
    'joint_state_broadcaster/JointStateBroadcaster': _broadcaster,
    'forward_command_controller/ForwardCommandController': _forward,
    'forward_command_controller/MultiInterfaceForwardCommandController': _multi_forward,
    'position_controllers/JointGroupPositionController':
        lambda p, a: _forward(p, a, 'position'),
    'velocity_controllers/JointGroupVelocityController':
        lambda p, a: _forward(p, a, 'velocity'),
    'effort_controllers/JointGroupEffortController':
        lambda p, a: _forward(p, a, 'effort'),
    'diff_drive_controller/DiffDriveController': _diff_drive,
    'omni_wheel_drive_controller/OmniWheelDriveController': _omni_drive,
}


def _hardware_interfaces(robot):
    joints = {}
    for joint in robot.findall('joint'):
        name = joint.get('name')
        if not name or name in joints:
            raise ValueError(f'Nombre de articulación URDF vacío o duplicado: {name}')
        joints[name] = joint
    commands, states = set(), set()
    systems = robot.findall('ros2_control')
    if not systems:
        raise ValueError('El URDF no declara ros2_control')
    for system in systems:
        registered = set()
        for joint in system.findall('joint'):
            name = joint.get('name')
            if name not in joints or joints[name].get('type') == 'fixed':
                raise ValueError(f'ros2_control: articulación URDF inexistente o fija: {name}')
            if name in registered:
                raise ValueError(f'ros2_control: articulación duplicada: {name}')
            registered.add(name)
            for tag, resources in [('command_interface', commands), ('state_interface', states)]:
                for interface in joint.findall(tag):
                    resource = (name, interface.get('name'))
                    if not resource[1] or resource in resources:
                        raise ValueError(f'ros2_control: interfaz vacía o duplicada: {resource}')
                    resources.add(resource)
            actuated = bool(joint.findall('command_interface'))
            for state in joint.findall('state_interface'):
                value = state.findtext('param[@name="initial_value"]')
                if value is None and not (actuated and state.get('name') == 'position'):
                    continue
                try:
                    initial = float(value) if value is not None else 0.0
                except ValueError as error:
                    raise ValueError(f'{name}: initial_value no es numérico') from error
                _number(initial, f'{name}/{state.get("name")}.initial_value')
                limit = joints[name].find('limit')
                if state.get('name') == 'position' and joints[name].get('type') != 'continuous':
                    if limit is not None:
                        lower = float(limit.get('lower', '-inf'))
                        upper = float(limit.get('upper', 'inf'))
                        if not lower <= initial <= upper:
                            raise ValueError(f'{name}: posición inicial fuera de límites URDF')
                if state.get('name') == 'velocity' and limit is not None:
                    velocity_limit = limit.get('velocity')
                    if velocity_limit is not None and abs(initial) > float(velocity_limit):
                        raise ValueError(f'{name}: velocidad inicial fuera de límites URDF')
            if (actuated and joints[name].find('mimic') is not None
                    and joint.get('mimic', 'true').lower() != 'false'):
                raise ValueError(f'{name}: un joint mimic no debe exportar comandos')
    return joints, Interfaces(frozenset(commands), frozenset(states))


def validate_control(robot, config):
    """Descubre controladores y valida recursos, tipos y activación simultánea."""
    _mapping(config, 'controllers.yaml')
    joints, available = _hardware_interfaces(robot)
    manager = _parameters(config, 'controller_manager')
    if 'update_rate' in manager:
        rate = manager['update_rate']
        if isinstance(rate, bool) or not isinstance(rate, int) or rate <= 0:
            raise ValueError('controller_manager.update_rate debe ser un entero positivo')
    declared = {}
    for name, value in manager.items():
        if not isinstance(name, str):
            raise ValueError('controller_manager: los nombres de parámetros deben ser texto')
        if isinstance(value, dict) and 'type' in value:
            controller_type = value['type']
        elif name.endswith('.type'):
            name, controller_type = name[:-5], value
        else:
            continue
        if not isinstance(name, str) or not re.fullmatch(r'[A-Za-z_][A-Za-z0-9_]*', name):
            raise ValueError(f'Nombre de controlador no válido: {name!r}')
        if name in declared:
            raise ValueError(f'Controlador vacío o duplicado: {name}')
        declared[name] = controller_type
    if not declared:
        raise ValueError('controller_manager no declara controladores con type')
    controllers = {}
    for name, controller_type in declared.items():
        if not isinstance(controller_type, str) or controller_type not in VALIDATORS:
            raise ValueError(f'{name}: tipo sin validador de recursos: {controller_type!r}')
        params = _parameters(config, name)
        try:
            names, required = VALIDATORS[controller_type](params, available)
            for joint in names:
                if joint not in joints:
                    raise ValueError(f'articulación inexistente en URDF: {joint}')
            for kind, resources, exported in (
                ('comando', required.commands, available.commands),
                ('estado', required.states, available.states),
            ):
                missing = sorted(resources - exported)
                if missing:
                    text = ', '.join(f'{joint}/{interface}' for joint, interface in missing)
                    raise ValueError(f'faltan interfaces de {kind} en ros2_control: {text}')
        except ValueError as error:
            raise ValueError(f'{name}: {error}') from error
        controllers[name] = Controller(name, controller_type, names, required)
    policy = _parameters(config, 'pepper_gz')
    if 'active_controllers' not in policy or 'inactive_controllers' not in policy:
        raise ValueError('pepper_gz debe definir active_controllers e inactive_controllers')
    active = _names(policy['active_controllers'], 'active_controllers', False)
    inactive = _names(policy['inactive_controllers'], 'inactive_controllers', False)
    if set(active) & set(inactive):
        raise ValueError('Un controlador no puede arrancar activo e inactivo a la vez')
    unknown = set(active + inactive) - controllers.keys()
    if unknown:
        raise ValueError(
            f'Política de arranque con controladores no declarados: {sorted(unknown)}',
        )
    owners = {}
    for name in active:
        for resource in sorted(controllers[name].interfaces.commands):
            if resource in owners:
                raise ValueError(
                    f'Conflicto de comando {resource[0]}/{resource[1]}: '
                    f'{owners[resource]} y {name} se activarían simultáneamente',
                )
            owners[resource] = name
    return ControlPlan(controllers, active, inactive)
