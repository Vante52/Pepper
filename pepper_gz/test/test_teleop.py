"""Entrada UTF-8, límites articulares, watchdog real y conservación de masa."""

import math
from pathlib import Path
from types import SimpleNamespace
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist, TwistStamped
import numpy as np
from pepper_gz.cmd_vel_guard import CmdVelGuard, limited_velocity, ramped_velocity
from pepper_gz.keyboard_teleop import decode_keys, joint_target, KeyboardTeleop
from pepper_gz.omni_kinematics import wheel_limited_velocity, wheel_velocities
from pepper_gz.wheel_contacts import configure_wheel_contacts
from pepper_gz.wheel_contacts import parallel_axis, read_inertia, rotation
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, qos_profile_sensor_data, QoSProfile
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String
import xacro
import yaml


@pytest.mark.parametrize('text,keys,pending', [
    ('wñol', ['w', 'ñ', 'o', 'l'], ''),
    ('\x1b[A\x1b[D', ['up', 'left'], ''),
    ('\x1b[', [], '\x1b['),
    ('\x1b[Añ', ['up', 'ñ'], ''),
    ('\t]+', ['\t', ']', '+'], ''),
    ('ÑW', ['ñ', 'w'], ''),
])
def test_keyboard_arrows_and_unicode(text, keys, pending):
    """Los prefijos de flechas no se confunden con WASD ni con ñ."""
    assert decode_keys(text) == (keys, pending)


@pytest.mark.parametrize('direction,expected', [(1, .2), (-1, -.2)])
def test_joint_target_bounds_and_maximum_lead(direction, expected):
    """Una tecla repetida no acumula objetivos lejos de la posición medida."""
    target = 0.
    for _ in range(100):
        target = joint_target(target, 0., direction, .03, .2, -.7, .6)
    assert target == pytest.approx(expected)
    assert joint_target(.59, .59, 1, .03, .2, -.7, .6) == .6
    assert joint_target(-.69, -.69, -1, .03, .2, -.7, .6) == -.7


def test_planar_limits():
    """La saturación preserva dirección; datos no finitos son rechazados."""
    command = Twist()
    command.linear.x, command.linear.y, command.angular.z = .3, .4, -.8
    x, y, yaw = limited_velocity(command, .15, .4)
    assert math.hypot(x, y) == pytest.approx(.15)
    assert y/x == pytest.approx(4./3.)
    assert yaw == pytest.approx(-.24)
    command.linear.x = float('nan')
    with pytest.raises(ValueError):
        limited_velocity(command, .15, .4)


def test_watchdog_while_sim_time_is_frozen(monkeypatch):
    """El watchdog usa tiempo real y no reproduce órdenes anteriores a la activación."""
    rclpy.init(args=[])
    node, observer = CmdVelGuard(), rclpy.create_node('guard_test_observer')
    share = Path(get_package_share_directory('pepper_gz'))
    node.description(String(data=xacro.process_file(str(share/'urdf/pepper_gz.xacro')).toxml()))
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(observer)
    outputs = []
    observer.create_subscription(TwistStamped, '/base_controller/cmd_vel', outputs.append, 10)
    node.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
    clock = observer.create_publisher(Clock, '/clock', qos_profile_sensor_data)
    wall = [100.]
    monkeypatch.setattr('pepper_gz.cmd_vel_guard.time.monotonic', lambda: wall[0])
    command = Twist()
    command.linear.x = .1

    def drain():
        for _ in range(15):
            executor.spin_once(timeout_sec=.02)

    try:
        node.receive(command)
        drain()
        assert not outputs
        node.activity(SimpleNamespace(controllers=[SimpleNamespace(
            name='base_controller', state=SimpleNamespace(label='active'))]))
        drain()
        assert outputs[-1].twist.linear.x == 0.
        node.receive(command)
        drain()
        assert outputs[-1].twist.linear.x == 0.  # El reloj sigue pausado.
        tick = Clock()
        tick.clock.sec = 1
        clock.publish(tick)
        drain()
        assert outputs[-1].twist.linear.x == .1
        wall[0] += .5
        drain()
        assert outputs[-1].twist.linear.x == 0.
        assert outputs[-1].header.stamp.sec == 1
        assert outputs[-1].header.frame_id == 'base_link'
        node.receive(command)
        monkeypatch.setattr(node, 'count_publishers', lambda _: 2)
        drain()
        assert outputs[-1].twist.linear.x == 0.
    finally:
        executor.shutdown()
        observer.destroy_node()
        node.destroy_node()
        rclpy.shutdown()


def aggregate(sdf):
    """Integra masa, primer momento y tensor en el frame rueda."""
    mass, moment, tensor = 0., np.zeros(3), np.zeros((3, 3))
    for link in sdf.findall('model/link'):
        pose = [float(v) for v in link.findtext('pose', '0 0 0 0 0 0').split()]
        r = rotation(*pose[3:])
        inertial = link.find('inertial')
        local = [float(v) for v in inertial.findtext('pose').split()]
        center = np.array(pose[:3]) + r @ np.array(local[:3])
        r = r @ rotation(*local[3:])
        m = float(inertial.findtext('mass'))
        mass += m
        moment += m*center
        tensor += r @ read_inertia(inertial.find('inertia')) @ r.T + parallel_axis(m, center)
    return mass, moment, tensor


def test_simulation_rollers_preserve_aggregate_dynamics():
    """Se conserva masa, centro e inercia incluso con frame inercial rotado."""
    sdf = ET.fromstring("""<sdf version="1.9"><model name="test"><link name="wheel">
      <inertial><mass>1.58028</mass><pose>.001 .002 -.001 .2 .1 -.3</pose>
      <inertia><ixx>.003</ixx><iyy>.0031</iyy><izz>.0032</izz>
      <ixy>.00001</ixy><ixz>.00002</ixz><iyz>.00003</iyz></inertia></inertial>
      <visual name="original"/><collision name="contact"><geometry><sphere>
      <radius>.07</radius></sphere></geometry></collision></link></model></sdf>""")
    share = Path(get_package_share_directory('pepper_gz'))
    config = yaml.safe_load((share/'config/wheel_contacts.yaml').read_text())
    config['wheel_links'] = ['wheel']
    before = aggregate(sdf)
    configure_wheel_contacts(sdf, config)
    for original, result in zip(before, aggregate(sdf)):
        np.testing.assert_allclose(original, result, atol=1e-14)
    assert len(sdf.findall('model/joint')) == config['roller_count']
    assert sdf.find("model/link[@name='wheel']/visual").get('name') == 'original'
    assert sdf.find("model/link[@name='wheel']/collision") is None


@pytest.mark.parametrize('key', ['w', 's', 'a', 'd', 'q', 'e'])
def test_fast_commands_fit_guard_and_original_wheel_limits(key):
    """Cada tecla respeta todas las ruedas y llega al controlador sin otro recorte."""
    share = Path(get_package_share_directory('pepper_gz'))
    keyboard = yaml.safe_load((share/'config/keyboard_teleop.yaml').read_text())
    guard = yaml.safe_load((share/'config/base_teleop.yaml').read_text())[
        'cmd_vel_guard']['ros__parameters']
    controllers = yaml.safe_load((share/'config/controllers.yaml').read_text())
    geometry = controllers[keyboard['base_controller']]['ros__parameters']
    robot = ET.fromstring(xacro.process_file(str(share/'urdf/pepper_gz.xacro')).toxml())
    limits = {joint.get('name'): float(joint.find('limit').get('velocity'))
              for joint in robot.findall('joint') if joint.find('limit') is not None}
    x, y, yaw = keyboard['base_keys'][key]
    requested = (x*keyboard['linear_speed'], y*keyboard['linear_speed'],
                 yaw*keyboard['angular_speed'])
    actual = wheel_limited_velocity(requested, geometry, limits)
    message = Twist()
    message.linear.x, message.linear.y, message.angular.z = actual
    assert limited_velocity(message, guard['max_linear_speed'], guard['max_angular_speed']) == (
        pytest.approx(actual))
    for i, wheel in enumerate(geometry['wheel_names']):
        angle = geometry['wheel_offset'] + i*2.*math.pi/len(geometry['wheel_names'])
        speed = (math.sin(angle)*actual[0] - math.cos(angle)*actual[1]
                 - geometry['robot_radius']*actual[2])/geometry['wheel_radius']
        assert abs(speed) <= limits[wheel] + 1e-10
    if key in ('w', 's', 'a', 'd'):
        assert actual == pytest.approx(requested)
    else:
        assert 0.43 < abs(actual[1]) < .45


def test_wheel_saturation_preserves_combined_motion_direction():
    """No recorta ruedas por separado al combinar traslación y giro."""
    geometry = {'wheel_names': ['a', 'b', 'c'], 'wheel_offset': math.pi/3.,
                'wheel_radius': .07, 'robot_radius': .176}
    requested = (.5, .5, 1.25)
    actual = wheel_limited_velocity(requested, geometry, {'a': 6.28, 'b': 6.28, 'c': 6.28})
    factors = [result/value for result, value in zip(actual, requested)]
    assert 0. < factors[0] < 1.
    assert factors == pytest.approx([factors[0]]*3)
    with pytest.raises(ValueError, match='límite'):
        wheel_limited_velocity(requested, geometry, {})


@pytest.mark.parametrize('current,target,dt,expected', [
    ((0., 0., 0.), (.5, 0., 1.25), .1, (.2, 0., .5)),
    ((.5, 0., 0.), (-.5, 0., 0.), .1, (.3, 0., 0.)),
    ((.5, 0., 0.), (-.5, 0., 0.), 0., (.5, 0., 0.)),
    ((0., 0., 0.), (.3, .4, 1.25), .05, (.06, .08, .25)),
])
def test_acceleration_ramps_include_reverse_and_frozen_clock(current, target, dt, expected):
    """La rampa limita aceleración vectorial y giro, sin avanzar con reloj congelado."""
    assert ramped_velocity(current, target, 2., 5., dt) == pytest.approx(expected)


def test_keyboard_holds_through_initial_repeat_delay(monkeypatch):
    """Una primera repetición con 580 ms de retraso no inserta una orden cero."""
    rclpy.init(args=[])
    node, observer = KeyboardTeleop(), rclpy.create_node('keyboard_repeat_test')
    executor = SingleThreadedExecutor()
    executor.add_node(observer)
    outputs = []
    observer.create_subscription(Twist, '/cmd_vel', outputs.append, 10)
    wall = [100.]
    monkeypatch.setattr('pepper_gz.keyboard_teleop.time.monotonic', lambda: wall[0])
    monkeypatch.setattr('pepper_gz.keyboard_teleop.select.select', lambda *args: ([], [], []))
    description = String()
    description.data = '<robot name="test">' + ''.join(
        f'<joint name="{name}" type="continuous"><limit velocity="6.28319"/></joint>'
        for name in ['WheelFL', 'WheelB', 'WheelFR']) + '</robot>'
    node.description(description)

    def publish_and_drain():
        for _ in range(15):
            node.tick()
            executor.spin_once(timeout_sec=.02)

    try:
        node.handle_key('w')
        publish_and_drain()
        assert outputs[-1].linear.x == .5
        wall[0] += .58
        publish_and_drain()
        assert outputs[-1].linear.x == .5
        node.handle_key('w')
        wall[0] += .1
        publish_and_drain()
        assert outputs[-1].linear.x == .5
        node.handle_key(' ')
        publish_and_drain()
        assert outputs[-1].linear.x == 0.
        node.handle_key('w')
        wall[0] += node.config['key_timeout'] + .01
        publish_and_drain()
        assert outputs[-1].linear.x == 0.
    finally:
        executor.shutdown()
        observer.destroy_node()
        node.destroy_node()
        rclpy.shutdown()


def test_external_cmd_vel_is_limited_without_keyboard():
    """El adaptador protege su tópico de salida frente a cualquier emisor de /cmd_vel."""
    import time

    rclpy.init(args=[])
    node, observer = CmdVelGuard(), rclpy.create_node('external_cmd_vel_test')
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(observer)
    outputs = []
    observer.create_subscription(TwistStamped, '/base_controller/cmd_vel', outputs.append, 10)
    publisher = observer.create_publisher(Twist, '/cmd_vel', 1)
    description = observer.create_publisher(String, '/robot_description', QoSProfile(
        depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    node.activity(SimpleNamespace(controllers=[SimpleNamespace(
        name='base_controller', state=SimpleNamespace(label='active'))]))
    command = Twist()
    command.linear.x, command.angular.z = .5, 1.25

    def send_for(seconds):
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            publisher.publish(command)
            executor.spin_once(timeout_sec=.02)

    try:
        send_for(.4)
        assert outputs and all(m.twist.linear.x == 0. for m in outputs)
        share = Path(get_package_share_directory('pepper_gz'))
        description.publish(String(data=xacro.process_file(
            str(share/'urdf/pepper_gz.xacro')).toxml()))
        send_for(.7)
        actual = outputs[-1].twist
        expected = wheel_limited_velocity((.5, 0., 1.25), node.geometry, node.wheel_limits)
        assert (actual.linear.x, actual.linear.y, actual.angular.z) == pytest.approx(expected)
        assert 0. < actual.linear.x < .5
        assert actual.angular.z/actual.linear.x == pytest.approx(2.5)
        # Incluye muestras de rampa, no solo el objetivo final.
        for message in outputs:
            value = message.twist
            speeds = wheel_velocities((value.linear.x, value.linear.y, value.angular.z),
                                      node.geometry)
            for wheel, speed in speeds.items():
                assert abs(speed) <= node.wheel_limits[wheel]+1e-10
        description.publish(String(data='<robot name="missing_wheels"/>'))
        send_for(.2)
        assert outputs[-1].twist.linear.x == 0.
        assert outputs[-1].twist.angular.z == 0.
    finally:
        executor.shutdown()
        observer.destroy_node()
        node.destroy_node()
        rclpy.shutdown()
