"""Proyección de profundidad y asociación a regiones sin suponer un frame óptico."""

import math
import threading
import time
from unittest.mock import Mock

import numpy as np
import pytest
from robot_skill_bridge.catalog import Skill
from robot_skill_bridge.target import rigid_transform, target_from_region, TargetBackend


def test_transform_rotates_xyz_instead_of_only_changing_frame_id():
    result = rigid_transform([[1., 0., 0.]], (1., 2., 0.),
                             (0., 0., math.sin(math.pi/4), math.cos(math.pi/4)))
    assert result[0] == pytest.approx((1., 3., 0.))


def test_projection_uses_intrinsics_and_rejects_background_and_nan():
    optical = np.array([[x, y, 2.] for x in np.linspace(-.1, .1, 8)
                        for y in np.linspace(-.1, .1, 8)] + [[5., 5., 2.], [np.nan]*3])
    base = np.column_stack((optical[:, 2], -optical[:, 0], -optical[:, 1]))
    result = target_from_region(optical, base, (200., 200., 320., 240., 640, 480),
                                (.45, .4, .55, .6))
    assert result['distance_m'] == pytest.approx(2.)
    assert abs(result['bearing_deg']) < 1e-6 and result['depth_samples'] == 64


@pytest.mark.parametrize('bounds', [
    (-.1, 0, 1, 1), (1, 0, 0, 1), (0, 0, 0, 0), (0, float('nan'), 1, 1)])
def test_invalid_regions_fail(bounds):
    with pytest.raises(ValueError):
        target_from_region(np.ones((20, 3)), np.ones((20, 3)), (1, 1, 1, 1, 2, 2), bounds)


def test_no_depth_cannot_be_replaced_by_a_language_model_distance():
    with pytest.raises(ValueError, match='profundidad'):
        target_from_region(np.empty((0, 3)), np.empty((0, 3)),
                           (200, 200, 320, 240, 640, 480), (.4, .4, .6, .6))


def test_a_box_on_the_floor_is_not_a_metric_object_detection():
    optical = np.array([[x, y, 2.] for x in np.linspace(-.1, .1, 8)
                        for y in np.linspace(-.1, .1, 8)])
    base = np.column_stack((optical[:, 2], -optical[:, 0], np.full(len(optical), -.602)))
    with pytest.raises(ValueError, match='profundidad'):
        target_from_region(optical, base, (200, 200, 320, 240, 640, 480),
                           (.45, .4, .55, .6), height_range=(-.45, 1.5))


def test_provider_is_injected_and_errors_are_domain_results():
    class Provider:
        def locate(self, observation_id, bounds):
            raise RuntimeError('La base cambió desde la imagen')
    skill = Skill('locate', 'Locate', 'target', 'camera', 'perception')
    result = TargetBackend({'camera': Provider()}).execute(skill, {
        'observation_id': 'old', 'left': .4, 'top': .4, 'right': .6, 'bottom': .6})
    assert not result.ok and result.status == 'target_unavailable'


@pytest.mark.parametrize('problem', ['unknown', 'expired', 'moved', 'rotated', 'moving',
                                     'stale_odom', 'reset_clock', 'frames'])
def test_live_context_is_invalidated_before_motion(problem):
    pytest.importorskip('rclpy')
    from nav_msgs.msg import Odometry
    from robot_skill_bridge.ros2_target_provider import ROS2TargetCameraProvider
    provider = ROS2TargetCameraProvider.__new__(ROS2TargetCameraProvider)
    provider.context_lock = threading.Condition()
    provider.max_age = 60.
    provider.base_frame = 'base'
    provider.node = Mock()
    provider.node.get_clock.return_value.now.return_value.nanoseconds = 10_000_000_000
    odom = Odometry()
    odom.header.stamp.sec = 10
    odom.header.frame_id, odom.child_frame_id = 'odom', 'base'
    odom.pose.pose.orientation.w = 1.
    context = {'captured_at': time.time(), 'sensor_stamp': 10., 'pose': (0., 0., 0.),
               'frames': ('odom', 'base')}
    received = time.monotonic()
    if problem == 'expired':
        context['captured_at'] -= 61.
    elif problem == 'moved':
        odom.pose.pose.position.x = .02
    elif problem == 'rotated':
        odom.pose.pose.orientation.z = math.sin(.1/2)
        odom.pose.pose.orientation.w = math.cos(.1/2)
    elif problem == 'moving':
        odom.twist.twist.linear.x = .1
    elif problem == 'stale_odom':
        received -= 2.
    elif problem == 'reset_clock':
        provider.node.get_clock.return_value.now.return_value.nanoseconds = 0
    elif problem == 'frames':
        odom.header.frame_id = 'another_odom'
    provider.contexts = {'token': context} if problem != 'unknown' else {}
    provider.odometry = [(odom, received)]
    with pytest.raises(RuntimeError):
        provider.locate('token', (.4, .4, .6, .6))
