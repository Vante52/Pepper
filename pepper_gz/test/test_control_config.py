"""Contrasta el URDF expandido con declaraciones y políticas de control."""

import copy
from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from pepper_gz.control_config import validate_control
import pytest
import xacro
import yaml


@pytest.fixture
def model():
    """Carga la configuración real desde el paquete instalado."""
    share = Path(get_package_share_directory('pepper_gz'))
    robot = ET.fromstring(xacro.process_file(str(share / 'urdf/pepper_gz.xacro')).toxml())
    config = yaml.safe_load((share / 'config/controllers.yaml').read_text())
    return robot, config


def add_controller(config, name, kind, params):
    """Declara un controlador de prueba sin modificar los archivos del paquete."""
    config['controller_manager']['ros__parameters'][name] = {'type': kind}
    config[name] = {'ros__parameters': params}


def policy(config, active=(), inactive=()):
    """Selecciona qué controladores se activan simultáneamente."""
    config['pepper_gz']['ros__parameters'] = {
        'active_controllers': list(active), 'inactive_controllers': list(inactive),
    }


def velocity_wheels(robot):
    """Exporta comandos solo en la copia URDF utilizada por la prueba."""
    for joint in robot.findall('ros2_control/joint'):
        if joint.get('name') in ('WheelFL', 'WheelFR', 'WheelB'):
            ET.SubElement(joint, 'command_interface', name='velocity')


def test_installed_configuration(model):
    """La configuración del robot cumple sus contratos y política de arranque."""
    robot, config = model
    plan = validate_control(robot, config)
    declarations = config['controller_manager']['ros__parameters']
    expected = {name for name, value in declarations.items()
                if isinstance(value, dict) and 'type' in value}
    assert set(plan.controllers) == expected
    assert plan.active == tuple(config['pepper_gz']['ros__parameters']['active_controllers'])
    assert plan.inactive == tuple(config['pepper_gz']['ros__parameters']['inactive_controllers'])


def test_renamed_and_additional_controllers(model):
    """Nombres, agrupaciones y nuevas declaraciones proceden únicamente del YAML."""
    robot, config = model
    params = config.pop('head_controller')['ros__parameters']
    del config['controller_manager']['ros__parameters']['head_controller']
    add_controller(
        config, 'camera_aim', 'joint_trajectory_controller/JointTrajectoryController', params,
    )
    add_controller(config, 'camera_manual', 'position_controllers/JointGroupPositionController', {
        'joints': ['HeadYaw'],
    })
    policy(
        config, ['joint_state_broadcaster', 'posture_controller', 'camera_aim'], ['camera_manual'],
    )
    plan = validate_control(robot, config)
    assert 'head_controller' not in plan.controllers
    assert plan.active[-1] == 'camera_aim'
    assert plan.inactive == ('camera_manual',)


def test_unselected_controllers_are_not_started(model):
    """Declarar un controlador no implica activarlo ni cargarlo."""
    robot, config = model
    policy(config, ['joint_state_broadcaster'])
    plan = validate_control(robot, config)
    assert 'head_controller' in plan.controllers
    assert 'head_controller' not in plan.active + plan.inactive


def test_unknown_urdf_joint(model):
    """Las referencias a articulaciones inexistentes fallan antes de iniciar Gazebo."""
    robot, config = model
    config['head_controller']['ros__parameters']['joints'] = ['MissingJoint']
    with pytest.raises(ValueError, match='head_controller.*inexistente.*MissingJoint'):
        validate_control(robot, config)


@pytest.mark.parametrize('tag, kind', [
    ('command_interface', 'comando'), ('state_interface', 'estado'),
])
def test_missing_hardware_interface(model, tag, kind):
    """Una articulación URDF existente debe exportar todos los recursos requeridos."""
    robot, config = model
    joint = robot.find('ros2_control/joint[@name="HeadYaw"]')
    joint.remove(joint.find(f'{tag}[@name="position"]'))
    with pytest.raises(ValueError, match=f'head_controller.*{kind}.*HeadYaw/position'):
        validate_control(robot, config)


def test_simultaneous_command_conflict(model):
    """Dos controladores activos no pueden reclamar la misma interfaz."""
    robot, config = model
    add_controller(config, 'manual_aim', 'position_controllers/JointGroupPositionController', {
        'joints': ['HeadYaw'],
    })
    policy(config, ['head_controller', 'manual_aim'])
    conflict = 'Conflicto.*HeadYaw/position.*head_controller.*manual_aim'
    with pytest.raises(ValueError, match=conflict):
        validate_control(robot, config)
    policy(config, ['head_controller'], ['manual_aim'])
    assert validate_control(robot, config).inactive == ('manual_aim',)


def test_shared_states_do_not_conflict(model):
    """Los estados admiten varios lectores sin crear conflictos de comandos."""
    robot, config = model
    add_controller(config, 'telemetry', 'joint_state_broadcaster/JointStateBroadcaster', {
        'joints': ['HeadYaw'], 'interfaces': ['position'], 'use_local_topics': True,
    })
    config['pepper_gz']['ros__parameters']['active_controllers'].append('telemetry')
    assert not validate_control(robot, config).controllers['telemetry'].interfaces.commands


@pytest.mark.parametrize('initial', ['3.0', '-3.0', 'nan', 'inf', 'invalid'])
def test_invalid_initial_position(model, initial):
    """Las posiciones iniciales deben ser numéricas, finitas y respetar el URDF."""
    robot, config = model
    robot.find('ros2_control/joint[@name="HeadYaw"]/state_interface/param').text = initial
    with pytest.raises(ValueError, match='HeadYaw'):
        validate_control(robot, config)


def test_individual_initial_positions(model):
    """La macro admite posiciones diferentes para cada articulación."""
    robot, config = model
    share = Path(get_package_share_directory('pepper_gz'))
    xml = f"""<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
      <xacro:include filename="{share / 'urdf/pepper_control.xacro'}"/>
      <xacro:pepper_position_joint name="test_yaw" initial_position="0.2"/>
      <xacro:pepper_position_joint name="test_pitch" initial_position="-0.1"/>
    </robot>"""
    document = xacro.parse(xml)
    xacro.process_doc(document)
    expanded = ET.fromstring(document.toxml())
    for name, value in [('test_yaw', 0.2), ('test_pitch', -0.1)]:
        joint = expanded.find(f'joint[@name="{name}"]')
        assert float(joint.findtext('state_interface[@name="position"]/param')) == value
    robot.find('ros2_control/joint[@name="HeadYaw"]/state_interface/param').text = '0.2'
    assert validate_control(robot, config)


def test_continuous_joint_has_no_position_bounds(model):
    """Las ruedas continuas no reciben límites de posición ficticios."""
    robot, config = model
    state = robot.find('ros2_control/joint[@name="WheelFL"]/state_interface[@name="position"]')
    ET.SubElement(state, 'param', name='initial_value').text = '123.0'
    assert validate_control(robot, config)


def test_velocity_controller_does_not_require_trajectory_parameters(model):
    """Un controlador de ruedas solicita velocity, sin asumir interfaz position ni feedback."""
    robot, config = model
    add_controller(config, 'wheels', 'velocity_controllers/JointGroupVelocityController', {
        'joints': ['WheelFL', 'WheelFR', 'WheelB'],
    })
    policy(config, ['wheels'])
    with pytest.raises(ValueError, match='wheels.*comando.*WheelFL/velocity'):
        validate_control(robot, config)
    velocity_wheels(robot)
    plan = validate_control(robot, config)
    assert plan.controllers['wheels'].interfaces.commands == frozenset(
        (name, 'velocity') for name in ['WheelFL', 'WheelFR', 'WheelB']
    )
    assert not plan.controllers['wheels'].interfaces.states


@pytest.mark.parametrize('open_loop, feedback', [(False, True), (False, False), (True, False)])
def test_diff_drive_feedback_contract(model, open_loop, feedback):
    """El contrato de ruedas depende del tipo y de su modo de realimentación."""
    robot, config = model
    velocity_wheels(robot)
    add_controller(config, 'drive', 'diff_drive_controller/DiffDriveController', {
        'left_wheel_names': ['WheelFL'], 'right_wheel_names': ['WheelFR'],
        'wheel_radius': 0.1, 'wheel_separation': 0.5,
        'open_loop': open_loop, 'position_feedback': feedback,
    })
    policy(config, ['drive'])
    result = validate_control(robot, config).controllers['drive']
    interface = 'position' if feedback else 'velocity'
    assert result.interfaces.states == (
        frozenset() if open_loop else frozenset((j, interface) for j in result.joints)
    )
    config['drive']['ros__parameters']['wheel_radius'] = 0
    with pytest.raises(ValueError, match='drive.*wheel_radius'):
        validate_control(robot, config)


@pytest.mark.parametrize('changes', [
    {'command_interfaces': ['effort', 'velocity']},
    {'state_interfaces': ['velocity']},
    {'allow_partial_joints_goal': 'false'},
    {'interpolation_method': 'unknown'},
    {'joints': ['HeadYaw', 'HeadYaw']},
    {'command_joints': ['KneePitch']},
    {'action_monitor_rate': 0},
    {'gains': {'HeadYaw': {'p': 'invalid'}}},
])
def test_invalid_trajectory_configuration(model, changes):
    """Los parámetros de trayectoria se comprueban según su tipo, sin nombres fijos."""
    robot, config = model
    config['head_controller']['ros__parameters'].update(changes)
    with pytest.raises(ValueError, match='head_controller'):
        validate_control(robot, config)


def test_broadcaster_requested_interface(model):
    """Una selección explícita del broadcaster debe existir en el hardware."""
    robot, config = model
    config['joint_state_broadcaster']['ros__parameters'].update({
        'joints': ['HeadYaw'], 'interfaces': ['effort'],
    })
    with pytest.raises(ValueError, match='joint_state_broadcaster.*estado.*HeadYaw/effort'):
        validate_control(robot, config)


def test_all_declared_controllers_are_validated(model):
    """Un controlador no seleccionado también debe tener una configuración consistente."""
    robot, config = model
    add_controller(config, 'future', 'velocity_controllers/JointGroupVelocityController', {
        'joints': ['MissingWheel'],
    })
    with pytest.raises(ValueError, match='future.*MissingWheel'):
        validate_control(robot, config)


def test_unknown_type_is_not_treated_as_trajectory(model):
    """Los tipos sin contrato conocido fallan de forma explícita."""
    robot, config = model
    config['controller_manager']['ros__parameters']['head_controller']['type'] = 'custom/Unknown'
    with pytest.raises(ValueError, match='tipo sin validador.*custom/Unknown'):
        validate_control(robot, config)


@pytest.mark.parametrize('active, inactive', [
    (['missing'], []), (['head_controller', 'head_controller'], []),
    (['head_controller'], ['head_controller']),
])
def test_invalid_startup_policy(model, active, inactive):
    """La política solo puede seleccionar nombres declarados, sin duplicarlos."""
    robot, config = model
    policy(config, active, inactive)
    with pytest.raises(ValueError):
        validate_control(robot, config)


def test_startup_is_explicit_and_can_select_nothing(model):
    """No existe una activación implícita de todos los controladores."""
    robot, config = model
    policy(config)
    plan = validate_control(robot, config)
    assert not plan.active and not plan.inactive
    del config['pepper_gz']
    with pytest.raises(ValueError, match='active_controllers'):
        validate_control(robot, config)


def test_flat_controller_type_declaration(model):
    """También se admite la notación ROS de parámetros con puntos."""
    robot, config = model
    manager = config['controller_manager']['ros__parameters']
    manager['head_controller.type'] = manager.pop('head_controller')['type']
    assert 'head_controller' in validate_control(robot, config).controllers


def test_duplicate_interface_export(model):
    """Un recurso ambiguo de hardware se detecta antes de comparar controladores."""
    robot, config = model
    joint = robot.find('ros2_control/joint[@name="HeadYaw"]')
    joint.append(copy.deepcopy(joint.find('command_interface')))
    with pytest.raises(ValueError, match='interfaz.*duplicada'):
        validate_control(robot, config)


def test_implicit_position_and_explicit_velocity_limits(model):
    """También se valida el cero implícito del plugin y una velocidad inicial declarada."""
    robot, config = model
    state = robot.find('ros2_control/joint[@name="HeadYaw"]/state_interface[@name="position"]')
    state.remove(state.find('param'))
    limit = robot.find('joint[@name="HeadYaw"]/limit')
    limit.set('lower', '0.1')
    with pytest.raises(ValueError, match='HeadYaw.*posición inicial'):
        validate_control(robot, config)
    limit.set('lower', '-2.0')
    velocity = robot.find(
        'ros2_control/joint[@name="HeadYaw"]/state_interface[@name="velocity"]/param',
    )
    velocity.text = '99'
    with pytest.raises(ValueError, match='HeadYaw.*velocidad inicial'):
        validate_control(robot, config)


@pytest.mark.parametrize('kind, params, interface', [
    ('forward_command_controller/ForwardCommandController',
     {'joints': ['HeadYaw'], 'interface_name': 'effort'}, 'effort'),
    ('forward_command_controller/MultiInterfaceForwardCommandController',
     {'joint': 'HeadYaw', 'interface_names': ['effort']}, 'effort'),
    ('effort_controllers/JointGroupEffortController', {'joints': ['HeadYaw']}, 'effort'),
])
def test_other_command_contracts(model, kind, params, interface):
    """Los contratos de comandos directos no solicitan estados de trayectoria."""
    robot, config = model
    joint = robot.find('ros2_control/joint[@name="HeadYaw"]')
    ET.SubElement(joint, 'command_interface', name=interface)
    add_controller(config, 'direct', kind, params)
    policy(config, ['direct'])
    contract = validate_control(robot, config).controllers['direct'].interfaces
    assert contract.commands == frozenset({('HeadYaw', interface)})
    assert not contract.states


def test_extra_broadcaster_joint_must_exist(model):
    """Incluso los estados sintéticos del broadcaster deben referirse al URDF."""
    robot, config = model
    config['joint_state_broadcaster']['ros__parameters']['extra_joints'] = ['MissingJoint']
    with pytest.raises(ValueError, match='joint_state_broadcaster.*inexistente.*MissingJoint'):
        validate_control(robot, config)


def test_broadcaster_unknown_interface_without_joint_filter(model):
    """Se detectan nombres de interfaz inválidos aunque el plugin use selección global."""
    robot, config = model
    config['joint_state_broadcaster']['ros__parameters']['interfaces'] = ['unknown_state']
    with pytest.raises(ValueError, match='joint_state_broadcaster.*estado.*unknown_state'):
        validate_control(robot, config)


def test_independent_arm_controllers(model):
    """Cada brazo tiene cinco ejes propios, con comandos de posición y estados PV."""
    robot, config = model
    plan = validate_control(robot, config)
    left = plan.controllers['left_arm_controller']
    right = plan.controllers['right_arm_controller']
    suffixes = ('ShoulderPitch', 'ShoulderRoll', 'ElbowYaw', 'ElbowRoll', 'WristYaw')
    for prefix, controller in [('L', left), ('R', right)]:
        assert controller.joints == tuple(prefix + suffix for suffix in suffixes)
        assert controller.interfaces.commands == frozenset(
            (joint, 'position') for joint in controller.joints
        )
        assert controller.interfaces.states == frozenset(
            (joint, interface) for joint in controller.joints
            for interface in ('position', 'velocity')
        )
        assert controller.name in plan.active
    assert not left.interfaces.commands & right.interfaces.commands


def test_initial_positions_configuration_is_applied(model):
    """El YAML de postura determina todos los valores iniciales de los ejes actuados."""
    robot, _ = model
    share = Path(get_package_share_directory('pepper_gz'))
    positions = yaml.safe_load((share / 'config/initial_positions.yaml').read_text())
    actuated = {joint.get('name'): joint for joint in robot.findall('ros2_control/joint')
                if joint.findall('command_interface')}
    assert set(positions['initial_positions']) == set(actuated)
    for name, joint in actuated.items():
        actual = float(joint.findtext('state_interface[@name="position"]/param'))
        assert actual == positions['initial_positions'][name]


def test_custom_arm_initial_positions(model, tmp_path):
    """Se puede cambiar la postura por configuración, sin editar el launch o la geometría."""
    original, config = model
    share = Path(get_package_share_directory('pepper_gz'))
    positions = yaml.safe_load((share / 'config/initial_positions.yaml').read_text())
    positions['initial_positions']['LShoulderPitch'] = 1.2
    positions['initial_positions']['RShoulderPitch'] = 1.4
    path = tmp_path / 'initial_positions.yaml'
    path.write_text(yaml.safe_dump(positions))
    modified = ET.fromstring(xacro.process_file(
        str(share / 'urdf/pepper_gz.xacro'), mappings={'initial_positions_config': str(path)},
    ).toxml())
    assert validate_control(modified, config)
    for name in ('LShoulderPitch', 'RShoulderPitch'):
        joint = modified.find(f'ros2_control/joint[@name="{name}"]')
        actual = float(joint.findtext('state_interface[@name="position"]/param'))
        assert actual == positions['initial_positions'][name]
    for tag in ('joint', 'link'):
        before = [ET.canonicalize(ET.tostring(e, encoding='unicode'), strip_text=True)
                  for e in original.findall(tag)]
        after = [ET.canonicalize(ET.tostring(e, encoding='unicode'), strip_text=True)
                 for e in modified.findall(tag)]
        assert before == after


@pytest.mark.parametrize('name', [
    'LShoulderRoll', 'RShoulderRoll', 'LElbowRoll', 'RElbowRoll',
])
def test_zero_is_not_valid_for_arm_roll(model, name):
    """Un cero genérico no debe superar la validación de la postura de los brazos."""
    robot, config = model
    joint = robot.find(f'ros2_control/joint[@name="{name}"]')
    joint.find('state_interface[@name="position"]/param').text = '0.0'
    with pytest.raises(ValueError, match=f'{name}.*posición inicial'):
        validate_control(robot, config)
