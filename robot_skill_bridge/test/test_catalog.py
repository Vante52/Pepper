"""Pruebas sin ROS: validación del contrato del catálogo."""

from pathlib import Path
import tempfile
import unittest

from robot_skill_bridge.catalog import load_catalog
from robot_skill_bridge.motion import parse_motion
from robot_skill_bridge.perception import parse_perception
from robot_skill_bridge.target import parse_target
from robot_skill_bridge.trajectory_profile import parse_trajectory
import yaml

CONFIG = Path(__file__).resolve().parents[1] / 'config' / 'pepper_skills.yaml'


class CatalogTests(unittest.TestCase):
    def test_valid_pepper_skill(self):
        catalog = load_catalog(CONFIG, {
            'follow_joint_trajectory': parse_trajectory, 'perception': parse_perception,
            'base_motion': parse_motion, 'target_location': parse_target,
        })
        self.assertEqual(set(catalog), {
            'wave_right', 'look_front', 'move_relative', 'turn_relative', 'locate_target'})
        skill = catalog['wave_right'].profile
        self.assertEqual(len(skill.joints), 5)
        self.assertEqual(skill.action, '/right_arm_controller/follow_joint_trajectory')
        self.assertLessEqual(skill.waypoints[-1].time + skill.goal_time_tolerance_sec, 5.0)

    def _invalid(self, edit):
        data = yaml.safe_load(CONFIG.read_text())
        edit(data['skills']['wave_right'])
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / 'skills.yaml'
            path.write_text(yaml.safe_dump(data))
            with self.assertRaises(ValueError):
                load_catalog(path, {
                    'follow_joint_trajectory': parse_trajectory, 'perception': parse_perception,
                    'base_motion': parse_motion, 'target_location': parse_target,
                })

    def test_rejects_nonexistent_waypoint_joint(self):
        self._invalid(lambda skill: skill['waypoints'][0].update(
            {'positions': [0.1, 0.2]},
        ))

    def test_rejects_outside_limits(self):
        self._invalid(lambda skill: skill['waypoints'][0].update(
            {'positions': [0.2, 0.6, 0.0, 0.85, 0.0]},
        ))

    def test_rejects_unsorted_time(self):
        self._invalid(lambda skill: skill['waypoints'][1].update({'time': 0.1}))

    def test_rejects_unsupported_backend(self):
        self._invalid(lambda skill: skill.update({'backend': 'arbitrary_shell'}))


if __name__ == '__main__':
    unittest.main()


def test_core_accepts_an_unrelated_backend(tmp_path):
    from robot_skill_bridge.catalog import BackendRouter, Skill
    path = tmp_path / 'other.yaml'
    path.write_text(yaml.safe_dump({'version': 1, 'skills': {
        'take_measurement': {'description': 'Medir', 'backend': 'meter', 'channel': 7},
    }}))
    catalog = load_catalog(path, {'meter': lambda name, data: data['channel']})
    skill = catalog['take_measurement']
    assert isinstance(skill, Skill) and skill.profile == 7

    from robot_skill_bridge.skill_result import SkillResult

    class Meter:
        def execute(self, selected, parameters=None, cancel=None):
            return SkillResult(True, 'measured', data={'channel': selected.profile})

    result = BackendRouter({'meter': Meter()}).execute(skill)
    assert result.ok and result.data == {'channel': 7}


def test_cancel_reaches_an_in_flight_domain_executor_and_shutdown_rejects_new_work():
    import threading
    from robot_skill_bridge.catalog import BackendRouter, Skill
    from robot_skill_bridge.skill_result import SkillResult
    entered = threading.Event()

    class Executor:
        def execute(self, skill, parameters=None, cancel=None):
            entered.set()
            assert cancel.wait(2.)
            return SkillResult(False, 'canceled', skill.name)

    router, results = BackendRouter({'executor': Executor()}), []
    skill = Skill('move', 'Move', 'executor', None)
    worker = threading.Thread(target=lambda: results.append(router.execute(skill)))
    worker.start()
    try:
        assert entered.wait(1.)
        assert router.cancel().status == 'cancel_requested'
    finally:
        worker.join(3.)
    assert results[0].status == 'canceled'
    router.close()
    assert router.execute(skill).status == 'unavailable'
