"""Pruebas deterministas del adaptador: sin servidores ROS ni simulación."""

from concurrent.futures import Future
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

import pytest
from robot_skill_bridge.catalog import load_catalog
from robot_skill_bridge.motion import parse_motion
from robot_skill_bridge.perception import parse_perception
from robot_skill_bridge.skill_result import SkillResult
from robot_skill_bridge.target import parse_target
from robot_skill_bridge.trajectory_profile import parse_trajectory

pytest.importorskip('rclpy')
GoalStatus = pytest.importorskip('action_msgs.msg').GoalStatus
adapter = pytest.importorskip('robot_skill_bridge.trajectory_backend')
TrajectoryBackend, to_duration = adapter.TrajectoryBackend, adapter.to_duration

CONFIG = Path(__file__).resolve().parents[1] / 'config/pepper_skills.yaml'


def resolved(value):
    future = Future()
    future.set_result(value)
    return future


@pytest.fixture
def skill():
    return load_catalog(CONFIG, {
        'follow_joint_trajectory': parse_trajectory, 'perception': parse_perception,
        'base_motion': parse_motion, 'target_location': parse_target,
    })['wave_right']


def action_client(accepted=True, status=GoalStatus.STATUS_SUCCEEDED, error_code=0, pending=False):
    handle = Mock(accepted=accepted)
    result = SimpleNamespace(status=status, result=SimpleNamespace(
        error_code=error_code, error_string='',
    ))
    handle.get_result_async.return_value = Future() if pending else resolved(result)
    client = Mock()
    client.send_goal_async.return_value = resolved(handle)
    return client, handle


def test_complete_goal_includes_all_points_and_tolerances(skill):
    client, _ = action_client()
    outcome = TrajectoryBackend(Mock())._run_action(skill, client)
    result = outcome.to_dict()
    assert outcome.ok and result['accepted']
    goal = client.send_goal_async.call_args.args[0]
    assert goal.trajectory.joint_names == list(skill.profile.joints)
    for point, expected in zip(goal.trajectory.points, skill.profile.waypoints):
        assert list(point.positions) == list(expected.positions)
        assert list(point.velocities) == [0.] * len(skill.profile.joints)
        assert point.time_from_start == to_duration(expected.time)
    assert len(goal.trajectory.points) == len(skill.profile.waypoints)
    assert [t.name for t in goal.goal_tolerance] == list(skill.profile.joints)


@pytest.mark.parametrize('accepted,status,error,expected', [
    (False, GoalStatus.STATUS_SUCCEEDED, 0, 'rejected'),
    (True, GoalStatus.STATUS_ABORTED, -4, 'aborted'),
    (True, GoalStatus.STATUS_CANCELED, 0, 'canceled'),
    (True, GoalStatus.STATUS_SUCCEEDED, -5, 'aborted'),
])
def test_terminal_failures_not_success(skill, accepted, status, error, expected):
    client, _ = action_client(accepted, status, error)
    outcome = TrajectoryBackend(Mock())._run_action(skill, client)
    assert not outcome.ok and outcome.status == expected


def test_timeout_requests_cancel_and_blocks_until_terminal_result(skill):
    skill = replace(skill, profile=replace(skill.profile, execution_timeout_sec=.01))
    backend = TrajectoryBackend(Mock())
    client, handle = action_client(pending=True)
    outcome = backend._run_action(skill, client)
    assert not outcome.ok and outcome.status == 'timeout'
    handle.cancel_goal_async.assert_called_once()
    assert backend.execute(skill).status == 'cancel_pending'
    handle.get_result_async.return_value.set_result(SimpleNamespace(
        status=GoalStatus.STATUS_CANCELED,
        result=SimpleNamespace(error_code=0, error_string=''),
    ))
    assert not backend._cancel_pending.is_set()


def test_late_acceptance_still_cancels(skill):
    skill = replace(skill, profile=replace(skill.profile, execution_timeout_sec=.01))
    backend = TrajectoryBackend(Mock())
    acceptance = Future()
    client, handle = action_client(pending=True)
    client.send_goal_async.return_value = acceptance
    assert backend._run_action(skill, client).status == 'timeout'
    acceptance.set_result(handle)
    handle.cancel_goal_async.assert_called_once()
    assert backend._cancel_pending.is_set()


@pytest.mark.parametrize('controller,state,expected', [
    ('right_arm_controller', 'inactive', 'controller_inactive'),
    ('other_controller', 'active', 'controller_inactive'),
])
def test_requires_active_controller(skill, controller, state, expected):
    node = Mock()
    node.create_client.return_value.call_async.return_value = resolved(SimpleNamespace(
        controller=[SimpleNamespace(name=controller, state=state)],
    ))
    outcome = TrajectoryBackend(node).execute(skill)
    assert not outcome.ok and outcome.status == expected


def test_missing_manager_reported(skill):
    node = Mock()
    node.create_client.return_value.wait_for_service.return_value = False
    assert TrajectoryBackend(node).execute(skill).status == 'controller_unavailable'


def test_action_unavailable_reported(skill):
    backend = TrajectoryBackend(Mock())
    backend.clients[skill.profile.action] = Mock()
    backend.clients[skill.profile.action].wait_for_server.return_value = False
    with patch.object(backend, '_controller_error', return_value=None):
        assert backend.execute(skill).status == 'controller_unavailable'


@pytest.mark.parametrize('states,verification,status', [
    ([None], True, 'state_unavailable'),
    ([{}, None], True, 'verification_failed'),
    ([{}, {}], False, 'verification_failed'),
    ([{}, {}], True, 'succeeded'),
])
def test_success_requires_measurements(skill, states, verification, status):
    import threading
    backend = TrajectoryBackend(Mock())
    observer = Mock(condition=threading.Condition())
    observer.wait.side_effect = states
    observer.begin.return_value.evaluate.return_value = (verification, {'samples': 1})
    backend.observers[skill.profile.joint_states_topic] = observer
    backend.clients[skill.profile.action] = Mock()
    with patch.object(backend, '_controller_error', return_value=None), patch.object(
        backend, '_run_action', return_value=SkillResult.from_dict({
            'ok': True, 'status': 'succeeded', 'accepted': True,
            'action_status': 4, 'error_code': 0,
        }),
    ):
        outcome = backend.execute(skill)
    assert outcome.status == status
    assert outcome.ok == (status == 'succeeded')


@pytest.mark.parametrize('stage', ['send', 'result'])
def test_unknown_transport_outcome_blocks_another_goal(skill, stage):
    backend = TrajectoryBackend(Mock())
    client, handle = action_client()
    if stage == 'send':
        client.send_goal_async.side_effect = RuntimeError('transport lost')
    else:
        failed = Future()
        failed.set_exception(RuntimeError('result lost'))
        handle.get_result_async.return_value = failed
    outcome = backend._run_action(skill, client)
    assert not outcome.ok and outcome.status == 'ros_error'
    assert backend.execute(skill).status == 'cancel_pending'
