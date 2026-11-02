"""Agente único multimodal, memoria y reactividad sin Ollama ni ROS."""

import base64
import copy
import json
from pathlib import Path

from host_demo.clients import OllamaModel
from host_demo.multimodal_agent import ConversationMemory, MultimodalAgent, load_config

import pytest

CONFIG = Path(__file__).with_name('agent_config.json')


def decision(person=None, skill='none', source='none', reply='Estoy aquí para ayudarte.'):
    return {'reply': reply, 'observations': {'person_visible': person},
            'action': {'skill': skill, 'source': source}}


class Bridge:
    def __init__(self, image=True):
        self.image = image
        self.called = []
        self.stamp = 100.
        self.result = {'skill': 'wave_right', 'ok': True, 'status': 'succeeded',
                       'verification': {'private_joint_name': [1., 2.]}}

    def discover(self):
        skills = [{'name': 'wave_right', 'description': 'Saludar', 'kind': 'action'}]
        if self.image:
            skills.append({'name': 'look_front', 'description': 'Observar', 'kind': 'perception'})
        return skills

    def execute(self, name):
        self.called.append(name)
        if name == 'look_front':
            return {'ok': True, 'status': 'observed', 'observation': {
                'image_base64': base64.b64encode(b'test-image').decode(),
                'media_type': 'image/png',
                'captured_at': self.stamp, 'source': 'test',
                'frame_id': 'optical',
            }}
        return self.result


class Model:
    def __init__(self, decisions):
        self.decisions = iter(decisions)
        self.calls = []

    def generate(self, messages, schema):
        self.calls.append((copy.deepcopy(messages), copy.deepcopy(schema)))
        if set(schema['properties']) == {'reply'}:
            return {'reply': 'He recibido el resultado de la habilidad.'}
        return next(self.decisions)


def agent(decisions, image=True, memory=None, clock=lambda: 100.):
    bridge, model = Bridge(image), Model(decisions)
    return MultimodalAgent(bridge, model, load_config(CONFIG), memory, clock), bridge, model


def test_qwen_is_the_only_model_for_text_and_images():
    requests = []

    def transport(url, payload, timeout):
        requests.append(payload)
        return {'message': {'content': '{"reply":"Hola"}'}}

    model = OllamaModel(transport=transport)
    model.generate([{'role': 'user', 'content': 'Hola'}], {})
    model.generate([{'role': 'user', 'content': '¿Qué ves?', 'images': ['encoded']}], {})
    assert {r['model'] for r in requests} == {'qwen3.5:4b'}
    assert all(r['think'] is False for r in requests)
    assert requests[1]['messages'][-1]['images'] == ['encoded']


def test_person_observation_produces_action_and_verified_result():
    instance, bridge, model = agent([decision(True, 'wave_right', 'observation')])
    output = instance.turn('¿Qué hay frente a ti?')
    assert output['observations'] == {'person_visible': True}
    assert output['action'] == {'skill': 'wave_right', 'source': 'observation'}
    assert bridge.called == ['look_front', 'wave_right']
    assert output['result']['ok']
    assert model.calls[0][0][-1]['images']
    assert 'private_joint_name' not in json.dumps(model.calls)


def test_person_staying_visible_does_not_repeat_even_after_cooldown():
    now = [100.]
    instance, bridge, _ = agent([decision(True), decision(True)], clock=lambda: now[0])
    instance.turn('Mira')
    now[0], bridge.stamp = 200., 200.
    output = instance.turn('Mira')
    assert output['action']['skill'] == 'none'
    assert bridge.called.count('wave_right') == 1


def test_person_reappearing_rearms_after_absence_and_cooldown():
    now = [100.]
    instance, bridge, _ = agent([decision(True), decision(False), decision(False), decision(True)],
                                clock=lambda: now[0])
    for index in range(4):
        now[0] = bridge.stamp = 100. + 30. * index
        instance.turn('Observa', watch=True)
    assert bridge.called.count('wave_right') == 2


def test_unknown_observation_does_not_rearm():
    instance, bridge, _ = agent([decision(True), decision(None), decision(None), decision(True)])
    for _ in range(4):
        instance.turn('Observa', watch=True)
    assert bridge.called.count('wave_right') == 1


def test_explicit_greeting_still_works_without_camera_and_can_repeat():
    instance, bridge, _ = agent([decision(None, 'wave_right', 'user_request') for _ in range(2)],
                                image=False)
    instance.turn('Saluda')
    instance.turn('Saluda de nuevo')
    assert bridge.called == ['wave_right', 'wave_right']


def test_stale_image_cannot_trigger_or_rearm():
    instance, bridge, _ = agent([decision(True)])
    bridge.stamp = 1.
    output = instance.turn('Mira')
    assert output['action']['skill'] == 'none'
    assert not output['observation']['fresh_for_action']
    assert bridge.called == ['look_front']


def test_perception_failure_allows_honest_conversation_but_no_detection():
    instance, bridge, _ = agent([decision(None, reply='No tengo una imagen disponible.')])
    original = bridge.execute

    def execute(name):
        if name == 'look_front':
            raise RuntimeError('camera unavailable')
        return original(name)

    bridge.execute = execute
    output = instance.turn('¿Qué ves?')
    assert output['observations']['person_visible'] is None
    assert output['perception_error'] == 'camera unavailable'
    assert not output.get('result')


def test_failed_automatic_action_is_not_automatically_retried():
    instance, bridge, _ = agent([decision(True), decision(True)])
    bridge.result = {'skill': 'wave_right', 'ok': False, 'status': 'controller_unavailable'}
    first = instance.turn('Observa', watch=True)
    second = instance.turn('Observa', watch=True)
    assert first['result']['ok'] is False and second['action']['skill'] == 'none'
    assert bridge.called.count('wave_right') == 1


def test_memory_persists_conversation_and_latch_without_images(tmp_path):
    path = tmp_path / 'memory.json'
    memory = ConversationMemory(path, max_turns=2)
    instance, _, _ = agent([decision(True, reply='Hola, Ana.')], memory=memory)
    instance.turn('Me llamo Ana')
    restored = ConversationMemory(path, max_turns=2)
    second, bridge, model = agent([decision(True, reply='Recuerdo que te llamas Ana.')],
                                  memory=restored)
    output = second.turn('¿Cómo me llamo?')
    assert output['action']['skill'] == 'none'
    assert bridge.called == ['look_front']
    assert 'Me llamo Ana' in json.dumps(model.calls[0])
    assert 'image_base64' not in path.read_text() and 'private_joint_name' not in path.read_text()
    restored.clear()
    assert json.loads(path.read_text()) == {'history': [], 'rules': {}}


@pytest.mark.parametrize('edit', [
    lambda d: d.update(extra='unexpected'),
    lambda d: d.update(action={'skill': 'shell', 'source': 'user_request'}),
    lambda d: d.update(observations={'person_visible': 1}),
    lambda d: d.update(action={'skill': 'wave_right', 'source': 'none'}),
])
def test_invalid_model_decisions_never_execute(edit):
    output = decision(True)
    edit(output)
    instance, bridge, _ = agent([output])
    with pytest.raises(ValueError):
        instance.turn('Hola')
    assert bridge.called == ['look_front']


def test_no_image_cannot_claim_a_person():
    instance, bridge, _ = agent([decision(True)], image=False)
    with pytest.raises(ValueError):
        instance.turn('¿Qué ves?')
    assert not bridge.called


def test_watch_cannot_invent_a_user_command():
    instance, bridge, _ = agent([decision(True, 'wave_right', 'user_request')])
    with pytest.raises(ValueError):
        instance.turn('Observa', watch=True)
    assert bridge.called == ['look_front']


def test_invalid_json_is_repaired_only_by_same_model_before_any_action():
    requests = []
    responses = iter(['```json\n{}\n```', '{"reply":"Hola"}'])

    def transport(url, payload, timeout):
        requests.append(copy.deepcopy(payload))
        return {'message': {'content': next(responses)}}

    result = OllamaModel(transport=transport).generate(
        [{'role': 'user', 'content': 'Hola'}], {'type': 'object'},
    )
    assert result == {'reply': 'Hola'}
    assert len(requests) == 2
    assert {request['model'] for request in requests} == {'qwen3.5:4b'}
    assert requests[1]['messages'][-1]['role'] == 'user'


def test_repeated_invalid_json_aborts():
    def transport(url, payload, timeout):
        return {'message': {'content': 'no JSON'}}

    with pytest.raises(RuntimeError, match='JSON válido'):
        OllamaModel(transport=transport).generate([], {})
