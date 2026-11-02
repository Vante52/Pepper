"""Pruebas sin ROS: validación del contrato del catálogo."""

import tempfile
import unittest
from pathlib import Path

from robot_skill_bridge.catalog import load_catalog
from robot_skill_bridge.trajectory_profile import parse_trajectory

import yaml

CONFIG = Path(__file__).resolve().parents[1] / 'config' / 'pepper_skills.yaml'


class CatalogTests(unittest.TestCase):
    def test_valid_pepper_skill(self):
        catalog = load_catalog(CONFIG, {'follow_joint_trajectory': parse_trajectory})
        self.assertEqual(set(catalog), {'wave_right'})
        skill = catalog['wave_right'].profile
        self.assertEqual(len(skill.joints), 5)
        self.assertEqual(skill.action, '/right_arm_controller/follow_joint_trajectory')
        self.assertEqual(skill.waypoints[-1].time, 6.5)

    def _invalid(self, edit):
        data = yaml.safe_load(CONFIG.read_text())
        edit(data['skills']['wave_right'])
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / 'skills.yaml'
            path.write_text(yaml.safe_dump(data))
            with self.assertRaises(ValueError):
                load_catalog(path, {'follow_joint_trajectory': parse_trajectory})

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

    class Meter:
        def execute(self, selected):
            return 200, {'ok': True, 'channel': selected.profile}

    assert BackendRouter({'meter': Meter()}).execute(skill) == (200, {'ok': True, 'channel': 7})
