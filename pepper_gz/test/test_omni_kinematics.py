"""Cinemática real de la configuración instalada y conservación del movimiento."""

import copy
import math
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from pepper_gz.omni_kinematics import controller_geometry, wheel_limited_velocity
from pepper_gz.omni_kinematics import wheel_limits_from_urdf, wheel_velocities
import pytest
import xacro
import yaml


@pytest.fixture(scope='module')
def model():
    """Carga geometría y límites desde las mismas fuentes que los nodos."""
    share = Path(get_package_share_directory('pepper_gz'))
    config = yaml.safe_load((share/'config/controllers.yaml').read_text())
    geometry = controller_geometry(config, 'base_controller')
    xml = xacro.process_file(str(share/'urdf/pepper_gz.xacro')).toxml()
    return geometry, wheel_limits_from_urdf(xml, geometry['wheel_names'])


@pytest.mark.parametrize('requested', [
    (0., 0., 0.), (.5, 0., 0.), (-.5, 0., 0.),
    (0., .5, 0.), (0., -.5, 0.), (0., 0., 1.25), (0., 0., -1.25),
    (.5, 0., 1.25), (.5, 0., -1.25), (-.5, 0., 1.25), (-.5, 0., -1.25),
    (.3, .4, 1.25), (-.3, -.4, -1.25), (.5, -.5, 1.25),
])
def test_each_wheel_is_bounded_and_all_components_share_one_scale(model, requested):
    """Longitudinal, lateral, giro y combinaciones conservan proporciones y signos."""
    geometry, limits = model
    actual = wheel_limited_velocity(requested, geometry, limits)
    for wheel, speed in wheel_velocities(actual, geometry).items():
        assert abs(speed) <= limits[wheel]+1e-10
    factors = []
    for result, original in zip(actual, requested):
        if original:
            factors.append(result/original)
        else:
            assert result == 0.
    if factors:
        assert 0. < factors[0] <= 1.
        assert factors == pytest.approx([factors[0]]*len(factors))
    else:
        assert actual == requested


def test_known_inverse_kinematics_signs_and_order(model):
    """Casos analíticos independientes fijan signos, orden y unidades del controlador."""
    geometry, _ = model
    front_left, back, front_right = geometry['wheel_names']
    r, radius = geometry['wheel_radius'], geometry['robot_radius']
    assert wheel_velocities((.5, 0., 0.), geometry) == pytest.approx({
        front_left: math.sqrt(3.)*.5/(2.*r), back: 0.,
        front_right: -math.sqrt(3.)*.5/(2.*r)})
    assert wheel_velocities((0., .5, 0.), geometry) == pytest.approx({
        front_left: -.5/(2.*r), back: .5/r, front_right: -.5/(2.*r)})
    assert wheel_velocities((0., 0., 1.25), geometry) == pytest.approx({
        name: -radius*1.25/r for name in geometry['wheel_names']})


def test_combined_command_that_previously_requested_9_33_radians_per_second(model):
    """El ejemplo problemático satura globalmente en vez de recortar una rueda."""
    geometry, limits = model
    requested = (.5, 0., 1.25)
    raw = wheel_velocities(requested, geometry)
    assert max(abs(v) for v in raw.values()) == pytest.approx(9.33, abs=.005)
    actual = wheel_limited_velocity(requested, geometry, limits)
    speeds = wheel_velocities(actual, geometry)
    assert max(abs(v) for v in speeds.values()) == pytest.approx(6.28319)
    assert actual[2]/actual[0] == pytest.approx(requested[2]/requested[0])
    assert wheel_limited_velocity(actual, geometry, limits) == pytest.approx(actual)


def test_unequal_wheel_limits_and_renamed_controller(model):
    """Respeta la rueda más restrictiva y permite cambiar nombres desde el YAML."""
    geometry, limits = model
    limits = dict(limits)
    limits[geometry['wheel_names'][0]] = 1.
    actual = wheel_limited_velocity((.5, .2, -.5), geometry, limits)
    for wheel, speed in wheel_velocities(actual, geometry).items():
        assert abs(speed) <= limits[wheel]+1e-10
    config = {
        'controller_manager': {'ros__parameters': {'mobile': {
            'type': 'omni_wheel_drive_controller/OmniWheelDriveController'}}},
        '/mobile': {'ros__parameters': geometry},
    }
    assert controller_geometry(config, 'mobile') == geometry


@pytest.mark.parametrize('invalid', [None, 0., -1., float('nan'), float('inf')])
def test_invalid_or_missing_wheel_limit_is_rejected(model, invalid):
    """Nunca autoriza movimiento usando límites ausentes o no finitos."""
    geometry, limits = model
    limits = dict(limits)
    wheel = geometry['wheel_names'][0]
    if invalid is None:
        del limits[wheel]
    else:
        limits[wheel] = invalid
    with pytest.raises(ValueError, match='límite|Límite'):
        wheel_limited_velocity((.5, 0., 1.25), geometry, limits)


@pytest.mark.parametrize('key,value', [
    ('wheel_names', ['one', 'two']), ('wheel_names', ['a', 'b', 'a']),
    ('wheel_radius', 0.), ('robot_radius', -1.), ('wheel_offset', float('nan')),
])
def test_invalid_geometry_is_rejected(model, key, value):
    """Una geometría inválida produce un error explícito antes de dividir o escalar."""
    geometry = copy.deepcopy(model[0])
    geometry[key] = value
    with pytest.raises(ValueError):
        wheel_limited_velocity((.5, 0., 1.25), geometry, model[1])


@pytest.mark.parametrize('value', ['nan', 'inf', '-1', '0', 'missing'])
def test_invalid_urdf_velocity_is_rejected(value):
    """Los límites se leen del URDF sin inventar valores de reserva."""
    attribute = '' if value == 'missing' else f'velocity="{value}"'
    xml = f'<robot name="test"><joint name="wheel" type="continuous"><limit {attribute}/>'
    xml += '</joint></robot>'
    with pytest.raises(ValueError):
        wheel_limits_from_urdf(xml, ['wheel'])
