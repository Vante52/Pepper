"""Pruebas del adaptador IA sin Ollama ni ROS 2 instalados."""

import json
import unittest
from unittest.mock import patch

from host_demo import demo_ollama

SKILLS = [{'name': 'wave_right', 'description': 'Saludar con brazo derecho'}]


class OllamaAdapterTests(unittest.TestCase):
    def test_selects_allowed_skill(self):
        with patch.object(demo_ollama, 'get_json', return_value={
            'message': {'content': json.dumps({'intent': 'execute_skill', 'skill': 'wave_right'})},
        }) as request:
            self.assertEqual(demo_ollama.interpret('Saluda con el brazo derecho', SKILLS),
                             'wave_right')
            arguments = request.call_args.args[1]
            schema = arguments['format']
            self.assertEqual(schema['properties']['skill']['enum'], ['wave_right', 'none'])

    def test_unsupported_executes_nothing(self):
        with patch.object(demo_ollama, 'get_json', return_value={
            'message': {'content': '{"intent":"unsupported","skill":"none"}'},
        }):
            self.assertIsNone(demo_ollama.interpret('Vuela al techo', SKILLS))

    def test_rejects_unlisted_llm_skill(self):
        with patch.object(demo_ollama, 'get_json', return_value={
            'message': {'content': '{"intent":"execute_skill","skill":"run_script"}'},
        }):
            with self.assertRaises(RuntimeError):
                demo_ollama.interpret('Ejecuta un script', SKILLS)


if __name__ == '__main__':
    unittest.main()


class ProviderContractTests(unittest.TestCase):
    def test_another_robot_catalog_needs_no_provider_change(self):
        skills = [{'name': 'inspect_part', 'description': 'Inspeccionar pieza'}]
        with patch.object(demo_ollama, 'get_json', return_value={
            'message': {'content': '{"intent":"execute_skill","skill":"inspect_part"}'},
        }) as request:
            self.assertEqual(demo_ollama.interpret('Inspecciona la pieza', skills), 'inspect_part')
            payload = request.call_args.args[1]
            self.assertNotIn('wave_right', json.dumps(payload))
            self.assertNotIn('controller', json.dumps(payload))

    def test_invalid_provider_outputs_are_rejected(self):
        for output in ['not JSON', '[]', '{"skill":"wave_right"}',
                       '{"intent":"unsupported","skill":"wave_right"}',
                       '{"intent":"execute_skill","skill":"none"}',
                       '{"intent":"execute_skill","skill":["wave_right"]}']:
            with self.subTest(output=output), patch.object(demo_ollama, 'get_json', return_value={
                'message': {'content': output},
            }):
                with self.assertRaises(RuntimeError):
                    demo_ollama.interpret('Saluda', SKILLS)

    def test_provider_transport_failure_does_not_execute(self):
        with patch.object(demo_ollama, 'get_json', side_effect=RuntimeError('offline')):
            with self.assertRaises(RuntimeError):
                demo_ollama.interpret('Saluda', SKILLS)
