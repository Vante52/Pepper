"""Control planar y estados de fallo sin ejecutar Gazebo."""

from dataclasses import replace
import math
from pathlib import Path
import threading
from unittest.mock import Mock, patch

import pytest
from robot_skill_bridge.catalog import load_catalog, Skill, validate_parameters
from robot_skill_bridge.motion import feedback_velocity, MotionProfile, parse_motion, relative_goal
from robot_skill_bridge.perception import parse_perception
from robot_skill_bridge.target import parse_target
from robot_skill_bridge.trajectory_profile import parse_trajectory


PROFILE = MotionProfile('/cmd_vel', '/odom', 'base', '/controller_manager', 'translate')


@pytest.mark.parametrize('heading,direction,expected', [
    (0., 0., (1., 0.)), (0., 90., (0., 1.)), (0., -90., (0., -1.)),
    (0., 180., (-1., 0.)), (math.pi/2, 0., (0., 1.)),
])
def test_goal_uses_initial_robot_orientation(heading, direction, expected):
    goal = relative_goal((2., 3., heading), 1., direction)
    assert goal[:2] == pytest.approx((2.+expected[0], 3.+expected[1]))


@pytest.mark.parametrize('direction', [-180., -90., -45., 0., 45., 90., 180.])
def test_feedback_reaches_translations_and_holds_orientation(direction):
    pose = (0., 0., .6)
    goal = relative_goal(pose, .5, direction)
    for _ in range(2000):
        velocity, reached = feedback_velocity(pose, goal, .6-pose[2], PROFILE)
        vx, vy, wz = velocity
        assert math.hypot(vx, vy) <= PROFILE.linear_speed + 1e-12
        if reached:
            break
        pose = (pose[0]+.05*(math.cos(pose[2])*vx-math.sin(pose[2])*vy),
                pose[1]+.05*(math.sin(pose[2])*vx+math.cos(pose[2])*vy), pose[2]+.05*wz)
    assert reached and math.dist(pose[:2], goal[:2]) <= PROFILE.position_tolerance


def test_catalog_bounds_reject_unbounded_or_malformed_motion():
    path = Path(__file__).resolve().parents[1]/'config/pepper_skills.yaml'
    skill = load_catalog(path, {
        'base_motion': parse_motion, 'perception': parse_perception,
        'target_location': parse_target,
        'follow_joint_trajectory': parse_trajectory})['move_relative']
    for parameters in ({'distance_m': 6., 'direction_deg': 0.},
                       {'distance_m': .2, 'direction_deg': float('nan')},
                       {'distance_m': .2}, {'distance_m': True, 'direction_deg': 0.}):
        with pytest.raises(ValueError):
            validate_parameters(skill, parameters)


def test_ros_backend_always_stops_and_releases_publisher_on_exception():
    pytest.importorskip('rclpy')
    from robot_skill_bridge.ros2_motion_backend import ROS2MotionBackend
    node = Mock()
    node.get_clock.return_value.now.return_value.nanoseconds = 0
    node.count_publishers.return_value = 0
    node.create_publisher.return_value.get_subscription_count.return_value = 1
    backend = ROS2MotionBackend(node)
    backend.states['/odom'] = (
        (0., 0., 0.), 0., __import__('time').monotonic(), ('odom', 'base'), (0., 0., 0.))
    with patch.object(backend, '_controller_active', return_value=True), patch.object(
        backend, '_run', side_effect=RuntimeError('lost transport')
    ):
        result = backend.execute(Skill('move', 'Move', 'motion', PROFILE), {})
    assert result.status == 'ros_error'
    publisher = node.create_publisher.return_value
    assert len(publisher.publish.call_args_list) == 3
    for call in publisher.publish.call_args_list:
        assert call.args[0].linear.x == 0. and call.args[0].angular.z == 0.
    node.destroy_publisher.assert_called_once_with(publisher)


def test_clock_mismatch_is_reported_before_creating_a_velocity_publisher():
    pytest.importorskip('rclpy')
    from robot_skill_bridge.ros2_motion_backend import ROS2MotionBackend
    node = Mock()
    node.get_clock.return_value.now.return_value.nanoseconds = 1_790_000_000_000_000_000
    backend = ROS2MotionBackend(node)
    backend.states['/odom'] = (
        (0., 0., 0.), 63., __import__('time').monotonic(), ('odom', 'base'), (0., 0., 0.))
    with patch.object(backend, '_controller_active', return_value=True):
        result = backend.execute(Skill('move', 'Move', 'motion', PROFILE), {})
    assert not result.ok and result.status == 'clock_mismatch'
    assert 'use_sim_time' in result.detail
    node.create_publisher.assert_not_called()


@pytest.mark.parametrize('scenario,expected', [
    ('cancel', 'canceled'), ('stale', 'state_unavailable'), ('duplicate', 'busy'),
    ('paused', 'clock_stopped'), ('stalled', 'stalled'), ('reset', 'state_unavailable'),
])
def test_ros_run_fails_closed(scenario, expected):
    pytest.importorskip('rclpy')
    import threading
    from robot_skill_bridge.ros2_motion_backend import ROS2MotionBackend
    node = Mock()
    node.count_publishers.return_value = 2 if scenario == 'duplicate' else 1
    node.get_clock.return_value.now.return_value.nanoseconds = 0
    backend = ROS2MotionBackend(node)
    profile = replace(PROFILE, state_timeout_sec=.1, stall_timeout_sec=.2)
    skill = Skill('move', 'Move', 'motion', profile)
    cancel = threading.Event()
    if scenario == 'cancel':
        cancel.set()
    clock = [0.]
    backend.states['/odom'] = ((0., 0., 0.), 0., 0., ('odom', 'base'), (0., 0., 0.))

    def advance(_):
        clock[0] += .05
        stamp = clock[0] if scenario == 'stalled' else 0.
        if scenario == 'reset':
            stamp = -.05
        received = -2. if scenario == 'stale' else clock[0]
        node.get_clock.return_value.now.return_value.nanoseconds = round(stamp*1e9)
        backend.states['/odom'] = (
            (0., 0., 0.), stamp, received, ('odom', 'base'), (0., 0., 0.))

    if scenario == 'stale':
        advance(0)
    with patch('robot_skill_bridge.ros2_motion_backend.time.monotonic', lambda: clock[0]), patch(
        'robot_skill_bridge.ros2_motion_backend.time.sleep', advance
    ):
        result = backend._run(skill, {'distance_m': .3, 'direction_deg': 0.}, Mock(), cancel)
    assert not result.ok and result.status == expected


@pytest.mark.parametrize('mode,parameters', [
    ('translate', {'distance_m': .4, 'direction_deg': 0.}),
    ('translate', {'distance_m': .4, 'direction_deg': 90.}),
    ('rotate', {'angle_deg': 90.}), ('rotate', {'angle_deg': -90.}),
    ('rotate', {'angle_deg': 360.}), ('rotate', {'angle_deg': -360.}),
])
def test_ros_feedback_loop_integrates_full_turns_and_checks_a_stopped_endpoint(mode, parameters):
    pytest.importorskip('rclpy')
    from robot_skill_bridge.ros2_motion_backend import ROS2MotionBackend
    node = Mock()
    node.count_publishers.return_value = 1
    backend = ROS2MotionBackend(node)
    skill = Skill('motion', 'Move', 'motion', replace(PROFILE, mode=mode))
    clock, pose, command = [0.], [2., 3., math.pi/2], [0., 0., 0.]
    node.get_clock.return_value.now.return_value.nanoseconds = 0
    backend.states['/odom'] = (tuple(pose), 0., 0., ('odom', 'base'), tuple(command))
    publisher = Mock()

    def publish(msg):
        command[:] = msg.linear.x, msg.linear.y, msg.angular.z

    publisher.publish.side_effect = publish

    def advance(dt):
        clock[0] += dt
        vx, vy, wz = command
        pose[0] += dt*(math.cos(pose[2])*vx-math.sin(pose[2])*vy)
        pose[1] += dt*(math.sin(pose[2])*vx+math.cos(pose[2])*vy)
        pose[2] = math.atan2(math.sin(pose[2]+dt*wz), math.cos(pose[2]+dt*wz))
        node.get_clock.return_value.now.return_value.nanoseconds = round(clock[0]*1e9)
        backend.states['/odom'] = (
            tuple(pose), clock[0], clock[0], ('odom', 'base'), tuple(command))

    with patch('robot_skill_bridge.ros2_motion_backend.time.monotonic', lambda: clock[0]), patch(
        'robot_skill_bridge.ros2_motion_backend.time.sleep', advance
    ):
        result = backend._run(skill, parameters, publisher, threading.Event())
    assert result.ok, result
    assert result.data['verification']['position_error_m'] <= PROFILE.position_tolerance
    assert abs(result.data['verification']['yaw_error_rad']) <= PROFILE.yaw_tolerance
    assert command == [0., 0., 0.]
