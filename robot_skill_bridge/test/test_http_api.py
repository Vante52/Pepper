"""Pruebas de interfaz HTTP con backend falso, sin ROS 2."""

import json
import threading
import unittest
from pathlib import Path
from urllib.error import HTTPError
from urllib.request import Request, urlopen

from robot_skill_bridge.catalog import load_catalog
from robot_skill_bridge.http_api import create_server
from robot_skill_bridge.trajectory_profile import parse_trajectory

CONFIG = Path(__file__).resolve().parents[1] / 'config' / 'pepper_skills.yaml'


class FakeBackend:
    def __init__(self):
        self.called = []

    def execute(self, skill):
        self.called.append(skill.name)
        return 200, {'ok': True, 'status': 'succeeded', 'skill': skill.name}


class HttpContractTests(unittest.TestCase):
    def setUp(self):
        self.fake = FakeBackend()
        catalog = load_catalog(CONFIG, {'follow_joint_trajectory': parse_trajectory})
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

    def test_backend_failure_keeps_its_terminal_status(self):
        self.fake.execute = lambda skill: (504, {'ok': False, 'status': 'timeout'})
        code, result = self.req('/v1/skills/execute', {'skill': 'wave_right'})
        self.assertEqual((code, result['status']), (504, 'timeout'))

    def test_in_flight_skill_excludes_another_request(self):
        entered, release = threading.Event(), threading.Event()

        def execute(skill):
            entered.set()
            release.wait(2.)
            return 200, {'ok': True, 'status': 'succeeded'}

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
