"""Verifica alineación de frames, integración SE(2) y errores con trayectorias conocidas."""

import math

import numpy as np
from pepper_gz.base_metrics import command_reference, fitted_parameters
from pepper_gz.base_metrics import relative_pose, series, trajectory_metrics
import pytest


def sources(gain=1., yaw_rate=0.):
    """Genera verdad terreno y odometría con orígenes y orientaciones distintos."""
    truth, odom = [], []
    commands = [[0., .2, 0., yaw_rate], [2., 0., 0., 0.]]
    for t in np.linspace(0., 2., 201):
        pose, _ = command_reference(commands, 0., t) if t else (np.zeros(3), 0.)
        for rows, origin, rotation, scale in [(truth, (2., -1.), .7, 1.),
                                              (odom, (-4., 3.), -.4, gain)]:
            x, y, yaw = pose*scale
            c, s = math.cos(rotation), math.sin(rotation)
            x, y = origin[0]+c*x-s*y, origin[1]+s*x+c*y
            if rows is truth:
                rows.append([t, x, y, .602, 0., 0., yaw+rotation])
            else:
                rows.append([t, x, y, 0., yaw+rotation, .2*scale, 0., yaw_rate*scale])
    return truth, odom, commands


def test_arbitrary_frames_cancel_for_equal_motion():
    """No compara directamente coordenadas absolutas de odom y mundo."""
    truth, odom, commands = sources()
    result, _, _ = trajectory_metrics(truth, odom, commands, 0., 2., 2.)
    assert result['endpoint_error_m'] < 1e-12
    assert result['orientation_error_rad'] == pytest.approx(0.)
    assert result['linear_velocity_rmse_mps'] < 1e-12
    assert result['tracking_endpoint_error_m'] < 1e-12


def test_known_scale_error_and_fit():
    """Un error del 10% produce 4 cm en 40 cm y una corrección de radio conocida."""
    truth, odom, commands = sources(gain=1.1)
    result, _, _ = trajectory_metrics(truth, odom, commands, 0., 2., 2.)
    assert result['endpoint_error_m'] == pytest.approx(.04)
    assert result['linear_velocity_rmse_mps'] == pytest.approx(.02)
    fit = fitted_parameters([result], .07, .176)
    assert fit['suggested_effective_wheel_radius'] == pytest.approx(.07/1.1)
    assert fit['suggested_effective_robot_radius'] is None


def test_constant_twist_exact_arc():
    """La referencia integra un arco exacto, incluyendo componente lateral."""
    pose, path = command_reference([[0., .2, .1, .5], [4., 0., 0., 0.]], 0., math.pi)
    assert pose == pytest.approx([.2, .6, math.pi/2.])
    assert path == pytest.approx(math.hypot(.2, .1)*math.pi)
    truth, odom, commands = sources(yaw_rate=.5)
    result, _, _ = trajectory_metrics(truth, odom, commands, 0., 2., 2.)
    assert result['endpoint_error_m'] < 1e-12
    assert result['angular_velocity_rmse_rps'] < 1e-12
    assert result['linear_velocity_rmse_mps'] < .00003


def test_command_zero_order_hold_and_angle_unwrap():
    """No interpola velocidades entre mensajes ni confunde el salto en pi con giro."""
    pose, path = command_reference([[0., .2, 0., 0.], [1., 0., 0., 0.],
                                    [2., 0., 0., 0.]], 0., 2.)
    assert pose == pytest.approx([.2, 0., 0.])
    assert path == pytest.approx(.2)
    ordered = series([[1., -3.1], [0., 3.1], [1., -3.], [2., -2.9]], 1)
    assert ordered[:, 0] == pytest.approx([0., 1., 2.])
    assert np.diff(ordered[:, 1]).min() > 0.


def test_empty_data_is_not_a_passing_measurement():
    """Ausencia de datos nunca se convierte en un error cero."""
    with pytest.raises(ValueError):
        trajectory_metrics([], [], [], 0., 1., 1.)
    with pytest.raises(ValueError):
        series([[0., float('nan')], [1., 0.]])


def test_relative_pose_rotation():
    """Un metro en Y del mundo es X local con origen girado noventa grados."""
    result = relative_pose(np.array([2., 3., math.pi/2.]),
                           [[2., 4., math.pi/2.]])
    assert result[0] == pytest.approx([1., 0., 0.], abs=1e-12)


def test_velocity_comparison_uses_equal_bandwidth_at_a_stop():
    """No atribuye al robot el desfase de filtrar solo una de las dos velocidades."""
    truth, odom = [], []
    for t in np.linspace(0., 2., 2001):
        position = .2*min(t, 1.)
        velocity = .2 if t < 1. else 0.
        truth.append([t, position, 0., .602, 0., 0., 0.])
        odom.append([t, position, 0., 0., 0., velocity, 0., 0.])
    commands = [[0., .2, 0., 0.], [1., 0., 0., 0.], [2., 0., 0., 0.]]
    result, _, velocities = trajectory_metrics(truth, odom, commands, 0., 1., 2.)
    assert result['endpoint_error_m'] == pytest.approx(0.)
    assert result['linear_velocity_rmse_mps'] < .001
    assert np.max(np.abs(velocities[:, 1]-velocities[:, 4])) < .0011


def test_default_ground_has_explicit_bullet_contact_properties():
    """Evita reintroducir la resistencia torsional por valores implícitos del motor."""
    from pathlib import Path
    import xml.etree.ElementTree as ET

    from ament_index_python.packages import get_package_share_directory

    share = Path(get_package_share_directory('pepper_gz'))
    world = ET.parse(share/'worlds/minimal_world.sdf').getroot().find('world')
    friction = world.find("model[@name='ground']/link/collision/surface/friction")
    assert friction is not None
    assert float(friction.findtext('torsional/coefficient')) == 0.
    assert float(friction.findtext('bullet/rolling_friction')) == 0.
    assert float(friction.findtext('bullet/friction')) == 1.
    assert world.findtext('plugin[@name="gz::sim::systems::Physics"]/engine/filename') == (
        'gz-physics-bullet-featherstone-plugin')
