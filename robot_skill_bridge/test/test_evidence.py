"""Un resultado exitoso del servidor no sustituye a las mediciones."""

from types import SimpleNamespace

from robot_skill_bridge.trajectory_evidence import MotionEvidence


def profile(**kwargs):
    values = {'joints': ['a', 'b'], 'waypoints': [SimpleNamespace(positions=[0., 0.])],
              'goal_tolerance_rad': .05, 'min_motion_rad': .1}
    return SimpleNamespace(**dict(values, **kwargs))


def test_return_to_start_counts_motion_not_only_final_displacement():
    evidence = MotionEvidence({'a': 0., 'b': 0.})
    evidence.observe({'a': .4, 'b': -.2})
    evidence.observe({'a': 0., 'b': 0.})
    passed, data = evidence.evaluate({'a': .001, 'b': 0.}, profile())
    assert passed
    assert data['maximum_motion_rad'] == {'a': .4, 'b': .2}


def test_static_robot_cannot_pass_a_gesture():
    evidence = MotionEvidence({'a': 0., 'b': 0.})
    evidence.observe({'a': 0., 'b': 0.})
    assert not evidence.evaluate({'a': 0., 'b': 0.}, profile())[0]


def test_one_joint_outside_tolerance_fails():
    evidence = MotionEvidence({'a': 0., 'b': 0.})
    evidence.observe({'a': .4, 'b': 0.})
    assert not evidence.evaluate({'a': .001, 'b': .06}, profile())[0]


def test_hold_skill_can_explicitly_allow_no_motion():
    evidence = MotionEvidence({'a': 0., 'b': 0.})
    evidence.observe({'a': 0., 'b': 0.})
    assert evidence.evaluate({'a': 0., 'b': 0.}, profile(min_motion_rad=0.))[0]
