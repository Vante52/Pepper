"""Contrato YAML -> HTTP -> controlador -> estados reales. No usa Gazebo ni Ollama APIs."""

import json
from pathlib import Path
import threading
import time
from urllib.request import Request, urlopen
import xml.etree.ElementTree as ET

import pytest
from robot_skill_bridge.catalog import load_catalog
from robot_skill_bridge.motion import parse_motion
from robot_skill_bridge.perception import parse_perception
from robot_skill_bridge.target import parse_target
from robot_skill_bridge.trajectory_profile import parse_trajectory
import yaml


def pytest_generate_tests(metafunc):
    if 'configured_skill' in metafunc.fixturenames:
        catalog = load_catalog(metafunc.config.getoption('--skills-config'), {
            'follow_joint_trajectory': parse_trajectory, 'perception': parse_perception,
            'base_motion': parse_motion, 'target_location': parse_target,
        })
        motions = [s for s in catalog.values() if s.backend == 'follow_joint_trajectory']
        metafunc.parametrize('configured_skill', motions, ids=[s.name for s in motions])


def test_profile_matches_installed_pepper_description_and_controllers():
    """La duplicación de límites del perfil debe coincidir con su fuente URDF."""
    ament = pytest.importorskip('ament_index_python.packages')
    xacro = pytest.importorskip('xacro')
    try:
        share = Path(ament.get_package_share_directory('pepper_gz'))
    except ament.PackageNotFoundError:
        pytest.skip('Se necesita el overlay de pepper_gz para comprobar el perfil Pepper')
    robot = ET.fromstring(xacro.process_file(str(share / 'urdf/pepper_gz.xacro')).toxml())
    config = Path(__file__).resolve().parents[1] / 'config/pepper_skills.yaml'
    catalog = load_catalog(config, {
        'follow_joint_trajectory': parse_trajectory, 'perception': parse_perception,
        'base_motion': parse_motion, 'target_location': parse_target,
    })
    controllers = yaml.safe_load((share / 'config/controllers.yaml').read_text())
    for skill in catalog.values():
        if skill.backend != 'follow_joint_trajectory':
            continue
        profile = skill.profile
        assert list(profile.joints) == controllers[profile.controller]['ros__parameters']['joints']
        for name in profile.joints:
            joint = robot.find(f"joint[@name='{name}']")
            assert joint is not None
            limit = joint.find('limit')
            assert profile.limits[name] == (float(limit.get('lower')), float(limit.get('upper')))
            export = f"ros2_control/joint[@name='{name}']/command_interface[@name='position']"
            assert robot.find(export) is not None


@pytest.mark.integration
def test_every_configured_skill_moves_and_reaches_its_waypoints(
    configured_skill, request, tmp_path,
):
    rclpy = pytest.importorskip('rclpy')
    from control_msgs.msg import JointTrajectoryControllerState
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String

    skill, profile = configured_skill, configured_skill.profile
    context = Context()
    rclpy.init(args=[], context=context)
    node = rclpy.create_node('skill_integration_observer', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    lock = threading.Lock()
    states, references, descriptions = [], [], []

    def joint_state(message):
        positions = dict(zip(message.name, message.position))
        if set(profile.joints) <= positions.keys():
            with lock:
                states.append([positions[joint] for joint in profile.joints])

    def reference(message):
        positions = dict(zip(message.joint_names, message.reference.positions))
        if set(profile.joints) <= positions.keys():
            with lock:
                references.append([positions[joint] for joint in profile.joints])

    node.create_subscription(JointState, profile.joint_states_topic, joint_state,
                             qos_profile_sensor_data)
    namespace = profile.action.rsplit('/', 1)[0]
    node.create_subscription(JointTrajectoryControllerState, namespace + '/controller_state',
                             reference, qos_profile_sensor_data)
    node.create_subscription(String, '/robot_description',
                             lambda msg: descriptions.append(msg.data),
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        deadline = time.monotonic() + 8
        while time.monotonic() < deadline and not (states and references and descriptions):
            threading.Event().wait(.05)
        assert states and references and descriptions, 'Faltan estados, referencias o URDF'
        robot = ET.fromstring(descriptions[-1])
        for name in profile.joints:
            joint = robot.find(f"joint[@name='{name}']")
            assert joint is not None, name
            if joint.get('type') != 'continuous':
                bounds = joint.find('limit')
                index = profile.joints.index(name)
                assert all(float(bounds.get('lower')) <= wp.positions[index]
                           <= float(bounds.get('upper')) for wp in profile.waypoints), name
        base = request.config.getoption('--bridge-url').rstrip('/')
        with urlopen(base + '/v1/skills', timeout=3) as response:
            public = json.load(response)
        assert skill.name in {entry['name'] for entry in public['skills']}
        with lock:
            initial = list(states[-1])
            states.clear()
            references.clear()
        payload = Request(base + '/v1/skills/execute',
                          data=json.dumps({'skill': skill.name}).encode(),
                          headers={'Content-Type': 'application/json'})
        with urlopen(payload, timeout=profile.execution_timeout_sec + 15) as response:
            result = json.load(response)
        assert result['ok'] and result['status'] == 'succeeded', result
        assert result['accepted'] and result['action_status'] == 4 and result['error_code'] == 0
        with lock:
            measured, commanded = list(states), list(references)
        assert measured and commanded
        # Cada punto debe aparecer en referencias del controlador y estados medidos.

        def waypoint_errors(samples):
            return [min(max(abs(a - b) for a, b in zip(sample, wp.positions))
                        for sample in samples) for wp in profile.waypoints]
        reference_errors = waypoint_errors(commanded)
        measured_errors = waypoint_errors(measured)
        assert max(reference_errors) <= profile.goal_tolerance_rad, reference_errors
        assert max(measured_errors) <= profile.goal_tolerance_rad, measured_errors
        motion = [max(abs(sample[i] - initial[i]) for sample in measured)
                  for i in range(len(initial))]
        assert max(motion) >= profile.min_motion_rad
        final_error = max(abs(a - b) for a, b
                          in zip(measured[-1], profile.waypoints[-1].positions))
        assert final_error <= profile.goal_tolerance_rad
        report = {'skill': skill.name, 'samples': len(measured),
                  'reference_waypoint_errors_rad': reference_errors,
                  'measured_waypoint_errors_rad': measured_errors,
                  'maximum_motion_rad': dict(zip(profile.joints, motion)),
                  'final_max_error_rad': final_error, 'response': result}
        (tmp_path / 'skill_result.json').write_text(json.dumps(report, indent=2))
        print(json.dumps(report, indent=2))
    finally:
        executor.shutdown(timeout_sec=3.)
        thread.join(3.)
        node.destroy_node()
        rclpy.shutdown(context=context)
