"""Pruebas de interfaz HTTP con backend falso, sin ROS 2."""

import json
from pathlib import Path
import threading
import unittest
from urllib.error import HTTPError
from urllib.request import Request, urlopen

from robot_skill_bridge.catalog import load_catalog
from robot_skill_bridge.http_api import create_server
from robot_skill_bridge.motion import parse_motion
from robot_skill_bridge.perception import parse_perception
from robot_skill_bridge.skill_result import SkillResult
from robot_skill_bridge.target import parse_target
from robot_skill_bridge.trajectory_profile import parse_trajectory

CONFIG = Path(__file__).resolve().parents[1] / 'config' / 'pepper_skills.yaml'


class FakeBackend:
    def __init__(self):
        self.called = []

    def execute(self, skill, parameters=None):
        self.called.append(skill.name)
        return SkillResult(True, 'succeeded', skill.name)


class HttpContractTests(unittest.TestCase):
    def setUp(self):
        self.fake = FakeBackend()
        catalog = load_catalog(CONFIG, {
            'follow_joint_trajectory': parse_trajectory, 'perception': parse_perception,
            'base_motion': parse_motion, 'target_location': parse_target,
        })
        self.server = create_server('127.0.0.1', 0, catalog, self.fake)
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()
        self.address = f'http://127.0.0.1:{self.server.server_address[1]}'

    def tearDown(self):
        self.server.shutdown()
        self.server.server_close()
        self.thread.join(3)

    def req(self, path, obj=None):
        data = None if obj is None else json.dumps(obj).encode('utf-8')
        request = Request(self.address + path, data=data,
                          headers={'Content-Type': 'application/json'})
        try:
            with urlopen(request, timeout=3) as reply:
                return reply.status, json.load(reply)
        except HTTPError as exc:
            return exc.code, json.load(exc)

    def test_skills_list(self):
        status, body = self.req('/v1/skills')
        self.assertEqual(status, 200)
        self.assertEqual(body['skills'][0]['name'], 'wave_right')

    def test_execute_allowed_skill(self):
        status, body = self.req('/v1/skills/execute', {'skill': 'wave_right'})
        self.assertEqual(status, 200)
        self.assertTrue(body['ok'])
        self.assertEqual(self.fake.called, ['wave_right'])

    def test_rejects_unknown_skill(self):
        status, _ = self.req('/v1/skills/execute', {'skill': 'execute_shell'})
        self.assertEqual(status, 404)
        self.assertFalse(self.fake.called)

    def test_rejects_joint_injection(self):
        status, _ = self.req('/v1/skills/execute',
                             {'skill': 'wave_right', 'joints': ['WheelFL']})
        self.assertEqual(status, 400)
        self.assertFalse(self.fake.called)

    def test_accepts_bounded_motion_parameters_and_publishes_the_schema(self):
        code, body = self.req('/v1/skills')
        motion = next(s for s in body['skills'] if s['name'] == 'move_relative')
        self.assertEqual(motion['parameters']['required'], ['distance_m', 'direction_deg'])
        self.assertEqual(code, 200)
        code, result = self.req('/v1/skills/execute', {
            'skill': 'move_relative', 'parameters': {'distance_m': .25, 'direction_deg': 90.}})
        self.assertEqual(code, 200)
        self.assertTrue(result['ok'])

    def test_rejects_motion_outside_the_public_limits(self):
        for parameters in ({'distance_m': 10., 'direction_deg': 0.},
                           {'distance_m': .25}, {'distance_m': .25, 'direction_deg': True}):
            code, _ = self.req('/v1/skills/execute', {
                'skill': 'move_relative', 'parameters': parameters})
            self.assertEqual(code, 400)
        self.assertFalse(self.fake.called)

    def test_backend_failure_keeps_its_terminal_status(self):
        self.fake.execute = lambda skill, parameters=None: SkillResult(False, 'timeout')
        code, result = self.req('/v1/skills/execute', {'skill': 'wave_right'})
        self.assertEqual((code, result['status']), (504, 'timeout'))

    def test_in_flight_skill_excludes_another_request(self):
        entered, release = threading.Event(), threading.Event()

        def execute(skill, parameters=None):
            entered.set()
            release.wait(2.)
            return SkillResult(True, 'succeeded')

        self.fake.execute = execute
        results = []
        thread = threading.Thread(target=lambda: results.append(
            self.req('/v1/skills/execute', {'skill': 'wave_right'}),
        ))
        thread.start()
        try:
            self.assertTrue(entered.wait(1.))
            code, result = self.req('/v1/skills/execute', {'skill': 'wave_right'})
            self.assertEqual((code, result['status']), (409, 'busy'))
        finally:
            release.set()
            thread.join(3.)
        self.assertEqual(results[0][0], 200)

    def test_perception_can_be_read_while_a_motion_is_running(self):
        entered, release = threading.Event(), threading.Event()

        def execute(skill, parameters=None):
            if skill.kind == 'action':
                entered.set()
                release.wait(2.)
            return SkillResult(True, 'observed' if skill.kind == 'perception' else 'succeeded')

        self.fake.execute = execute
        thread = threading.Thread(target=lambda: self.req(
            '/v1/skills/execute', {'skill': 'wave_right'},
        ))
        thread.start()
        try:
            self.assertTrue(entered.wait(1.))
            status, result = self.req('/v1/skills/execute', {'skill': 'look_front'})
            self.assertEqual((status, result['status']), (200, 'observed'))
        finally:
            release.set()
            thread.join(3.)
