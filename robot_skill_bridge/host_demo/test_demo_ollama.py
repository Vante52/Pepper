"""Selección de modelos, percepción bajo demanda y reactividad sin servicios externos."""

import base64
import copy
import json
from pathlib import Path

from host_demo.clients import OllamaModel
from host_demo.multimodal_agent import ConversationMemory, MultimodalAgent, load_config

import pytest

CONFIG = Path(__file__).with_name('agent_config.json')


def decision(person=None, skill='none', source='none', reply='Estoy aquí para ayudarte.',
             perception_required=False):
    return {'reply': reply, 'perception_required': perception_required,
            'observations': {'person_visible': person},
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
        self.visual_decision = None

    def generate(self, messages, schema):
        self.calls.append((copy.deepcopy(messages), copy.deepcopy(schema)))
        if set(schema['properties']) == {'description', 'person_visible'}:
            self.visual_decision = next(self.decisions)
            return {'description': self.visual_decision['reply'],
                    'person_visible': self.visual_decision['observations']['person_visible']}
        if self.visual_decision is not None:
            result, self.visual_decision = self.visual_decision, None
            return result
        return next(self.decisions)


def agent(decisions, image=True, memory=None, clock=lambda: 100.):
    bridge, model = Bridge(image), Model(decisions)
    return MultimodalAgent(bridge, model, load_config(CONFIG), memory, clock), bridge, model


def test_text_and_image_requests_use_the_configured_models():
    requests = []

    def transport(url, payload, timeout):
        requests.append(payload)
        return {'message': {'content': '{"reply":"Hola"}'}}

    model = OllamaModel(transport=transport)
    model.generate([{'role': 'user', 'content': 'Hola'}], {})
    model.generate([{'role': 'user', 'content': '¿Qué ves?', 'images': ['encoded']}], {})
    assert [r['model'] for r in requests] == ['qwen2.5:7b', 'qwen3-vl:2b-instruct']
    assert all(r['think'] is False for r in requests)
    assert requests[1]['messages'][-1]['images'] == ['encoded']


def test_person_observation_produces_action_and_verified_result():
    instance, bridge, model = agent([decision(True, 'wave_right', 'observation')])
    output = instance.turn('¿Qué hay frente a ti?', watch=True)
    assert output['observations'] == {'person_visible': True}
    assert output['action'] == {'skill': 'wave_right', 'source': 'observation'}
    assert bridge.called == ['look_front', 'wave_right']
    assert output['result']['ok']
    assert model.calls[0][0][-1]['images']
    assert 'private_joint_name' not in json.dumps(model.calls)


def test_person_staying_visible_does_not_repeat_even_after_cooldown():
    now = [100.]
    instance, bridge, _ = agent([decision(True), decision(True)], clock=lambda: now[0])
    instance.turn('Mira', watch=True)
    now[0], bridge.stamp = 200., 200.
    output = instance.turn('Mira', watch=True)
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
    output = instance.turn('Mira', watch=True)
    assert output['action']['skill'] == 'none'
    assert not output['observation']['fresh_for_action']
    assert bridge.called == ['look_front']


def test_perception_failure_allows_honest_conversation_but_no_detection():
    instance, bridge, _ = agent([decision(perception_required=True)])
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
    instance.turn('Me llamo Ana', watch=True)
    restored = ConversationMemory(path, max_turns=2)
    second, bridge, model = agent([decision(None, reply='Recuerdo que te llamas Ana.')],
                                  memory=restored)
    output = second.turn('¿Cómo me llamo?')
    assert output['action']['skill'] == 'none'
    assert bridge.called == []
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
        instance.turn('Hola', watch=True)
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
    assert {request['model'] for request in requests} == {'qwen2.5:7b'}
    assert requests[1]['messages'][-1]['role'] == 'user'


def test_repeated_invalid_json_aborts():
    def transport(url, payload, timeout):
        return {'message': {'content': 'no JSON'}}

    with pytest.raises(RuntimeError, match='JSON válido'):
        OllamaModel(transport=transport).generate([], {})


def test_normal_conversation_never_acquires_or_sends_an_image():
    instance, bridge, model = agent([decision(reply='Hola, Ana.')])
    output = instance.turn('Hola, me llamo Ana')
    assert bridge.called == []
    assert len(model.calls) == 1
    assert not any(m.get('images') for m in model.calls[0][0])
    assert output['observations']['person_visible'] is None


def test_direct_greeting_does_not_read_camera_even_when_it_is_available():
    instance, bridge, model = agent([decision(None, 'wave_right', 'user_request')])
    instance.turn('Saluda con el brazo derecho')
    assert bridge.called == ['wave_right']
    assert not any(m.get('images') for messages, _ in model.calls for m in messages)


def test_visual_question_acquires_an_image_only_after_text_interpretation():
    instance, bridge, model = agent([
        decision(perception_required=True), decision(False, reply='Veo un cono.'),
    ])
    output = instance.turn('¿Qué hay delante de ti?')
    assert bridge.called == ['look_front']
    assert len(model.calls) == 3
    assert not any(m.get('images') for m in model.calls[0][0])
    assert model.calls[1][0][-1]['images']
    assert not any(m.get('images') for m in model.calls[2][0])
    assert set(model.calls[1][1]['properties']) == {'description', 'person_visible'}
    assert output['reply'] == 'Veo un cono.'


def test_visual_request_without_camera_does_not_call_the_vision_model_or_act():
    instance, bridge, model = agent([
        decision(None, 'wave_right', 'user_request', perception_required=True),
    ], image=False)
    output = instance.turn('Saluda si ves a una persona')
    assert bridge.called == []
    assert len(model.calls) == 1
    assert output['action']['skill'] == 'none'
    assert 'imagen válida' in output['reply']


def test_configured_model_names_are_used_for_the_whole_turn():
    config = load_config(CONFIG)
    bridge, requests = Bridge(), []
    replies = iter([decision(perception_required=True),
                    {'description': 'Un cono.', 'person_visible': False},
                    {'reply': 'La imagen muestra un cono.', 'action': {'skill': 'none'}}])

    def transport(url, payload, timeout):
        requests.append(copy.deepcopy(payload))
        return {'message': {'content': json.dumps(next(replies))}}

    model = OllamaModel(config['model'], vision_model=config['vision_model'], transport=transport)
    instance = MultimodalAgent(bridge, model, config, clock=lambda: 100.)
    instance.turn('¿Qué ves?')
    assert [r['model'] for r in requests] == [
        'qwen2.5:7b', 'qwen3-vl:2b-instruct', 'qwen2.5:7b',
    ]
    assert set(requests[-1]['format']['properties']) == {'reply', 'action'}
    assert bridge.called == ['look_front']


def test_only_visual_facts_can_trigger_an_automatic_greeting():
    config, bridge = load_config(CONFIG), Bridge()
    replies = iter([
        decision(perception_required=True),
        {'description': 'Escena vacía.', 'person_visible': False},
        decision(True, 'wave_right', 'observation'),
    ])

    def transport(url, payload, timeout):
        return {'message': {'content': json.dumps(next(replies))}}

    model = OllamaModel(config['model'], vision_model=config['vision_model'], transport=transport)
    output = MultimodalAgent(bridge, model, config, clock=lambda: 100.).turn('¿Qué ves?')
    assert output['observations']['person_visible'] is False
    assert output['action']['skill'] == 'none'
    assert bridge.called == ['look_front']


def test_slow_image_analysis_can_answer_without_triggering_a_stale_action():
    instance, bridge, _ = agent([
        decision(perception_required=True), decision(False, reply='La imagen muestra un cono.'),
    ])
    bridge.stamp = 1.
    output = instance.turn('Describe la imagen')
    assert output['reply'] == 'La imagen muestra un cono.'
    assert output['observation']['fresh_for_action'] is False
    assert output['action']['skill'] == 'none'
    assert bridge.called == ['look_front']


def test_action_requiring_perception_is_blocked_if_analysis_has_become_stale():
    instance, bridge, _ = agent([
        decision(perception_required=True), decision(True, 'wave_right', 'user_request'),
    ])
    bridge.stamp = 1.
    output = instance.turn('Saluda si ves a alguien')
    assert output['action']['skill'] == 'none'
    assert bridge.called == ['look_front']


def test_pending_perception_with_no_skill_ignores_an_inconsistent_source():
    instance, bridge, model = agent([
        decision(source='user_request', perception_required=True),
        decision(False, reply='La imagen muestra un cono.'),
    ])
    output = instance.turn('qué ves?')
    assert output['reply'] == 'La imagen muestra un cono.'
    assert output['action'] == {'skill': 'none', 'source': 'none'}
    assert bridge.called == ['look_front']
    assert len(model.calls) == 3


def test_no_skill_is_normalized_but_unauthorized_actions_are_rejected():
    from host_demo.multimodal_agent import validate_decision
    for source in ('none', 'user_request', 'observation'):
        output = validate_decision(decision(source=source), ['wave_right'], False, False)
        assert output['action'] == {'skill': 'none', 'source': 'none'}
    with pytest.raises(ValueError):
        validate_decision(decision(skill='wave_right'), ['wave_right'], False, False)
    with pytest.raises(ValueError):
        validate_decision(decision(skill='unknown', source='user_request'),
                          ['wave_right'], False, False)


def test_interactive_session_continues_after_a_bad_model_decision(monkeypatch, capsys):
    from unittest.mock import Mock
    from host_demo import demo_ollama
    turns = Mock(side_effect=[
        ValueError('Acción inválida o no disponible'), decision(reply='Hola'),
    ])
    monkeypatch.setattr(demo_ollama, 'MultimodalAgent', Mock(return_value=Mock(turn=turns)))
    monkeypatch.setattr(demo_ollama, 'HttpSkillClient', Mock())
    monkeypatch.setattr(demo_ollama, 'OllamaModel', Mock())
    monkeypatch.setattr('sys.argv', ['demo_ollama.py', '--no-memory'])
    inputs = iter(['qué ves?', 'hola', '/exit'])
    monkeypatch.setattr('builtins.input', lambda _: next(inputs))
    assert demo_ollama.main() == 0
    captured = capsys.readouterr()
    assert 'No pude completar este turno' in captured.err
    assert 'Pepper > Hola' in captured.out and turns.call_count == 2


def test_explicit_perception_skips_text_routing_and_still_uses_the_vision_pipeline():
    instance, bridge, model = agent([decision(False, reply='Veo el suelo.')])
    output = instance.turn('qué ves?', perceive=True)
    assert output['reply'] == 'Veo el suelo.'
    assert bridge.called == ['look_front']
    assert len(model.calls) == 2 and model.calls[0][0][-1]['images']
    assert not any(message.get('images') for message in model.calls[1][0])


def test_interactive_look_requests_explicit_perception(monkeypatch):
    from unittest.mock import Mock
    from host_demo import demo_ollama
    turn = Mock(return_value=decision(reply='Veo un cono'))
    monkeypatch.setattr(demo_ollama, 'MultimodalAgent', Mock(return_value=Mock(turn=turn)))
    monkeypatch.setattr(demo_ollama, 'HttpSkillClient', Mock())
    monkeypatch.setattr(demo_ollama, 'OllamaModel', Mock())
    monkeypatch.setattr('sys.argv', ['demo_ollama.py', '--no-memory'])
    inputs = iter(['/look qué hay delante?', '/exit'])
    monkeypatch.setattr('builtins.input', lambda _: next(inputs))
    assert demo_ollama.main() == 0
    turn.assert_called_once_with('qué hay delante?', perceive=True)


@pytest.mark.parametrize('result,expected', [
    ({'ok': True, 'status': 'succeeded'}, '¡Listo, ya saludé!'),
    ({'ok': False, 'status': 'timeout'}, 'No pude completar el saludo.'),
    ({'status': 'unknown'}, 'No pude completar el saludo.'),
    ({'ok': 'true', 'status': 'succeeded'}, 'No pude completar el saludo.'),
    ({'ok': 1, 'status': 'succeeded'}, 'No pude completar el saludo.'),
    (None, 'No pude completar el saludo.'),
])
def test_skill_reply_uses_the_result_without_a_second_model_call(result, expected):
    instance, bridge, model = agent([decision(None, 'wave_right', 'user_request')])
    bridge.result = result
    output = instance.turn('Saluda')
    assert output['reply'] == expected
    assert bridge.called == ['wave_right']
    assert len(model.calls) == 1


def test_transport_failure_has_a_result_based_reply_and_is_never_retried():
    instance, bridge, model = agent([decision(None, 'wave_right', 'user_request')])
    calls = []

    def execute(name):
        calls.append(name)
        raise RuntimeError('offline')

    bridge.execute = execute
    output = instance.turn('Saluda')
    assert output['result'] == {'ok': False, 'status': 'transport_error'}
    assert output['reply'] == 'No pude completar el saludo.'
    assert calls == ['wave_right'] and len(model.calls) == 1


def test_initial_model_request_has_no_schema_or_example_copied_into_messages():
    requests = []
    messages = [{'role': 'system', 'content': 'Devuelve JSON.'},
                {'role': 'user', 'content': 'Hola'}]
    schema = {'type': 'object', 'properties': {'reply': {'type': 'string'}}}

    def transport(url, payload, timeout):
        requests.append(copy.deepcopy(payload))
        return {'message': {'content': '{"reply":"Hola"}'}}

    OllamaModel(transport=transport).generate(messages, schema)
    assert len(requests) == 1
    assert requests[0]['messages'] == messages and requests[0]['format'] == schema


def test_prompt_context_is_bounded_and_keeps_only_conversation_text():
    memory = ConversationMemory()
    for index in range(6):
        memory.record(f'Pregunta {index}', decision(reply=f'Respuesta {index}'))
    original = copy.deepcopy(memory.data)
    context = memory.context(max_turns=2, max_chars=1000)
    assert len(context) == 4
    assert context[0]['content'] == 'Pregunta 4' and context[-1]['content'] == 'Respuesta 5'
    assert 'observations' not in json.dumps(context) and 'action' not in json.dumps(context)
    assert sum(len(m['content']) for m in memory.context(6, 15)) <= 15
    assert memory.context(0, 1000) == [] and memory.context(4, 0) == []
    assert memory.data == original


def test_normal_prompt_has_no_examples_or_reactive_configuration_and_only_public_skill_summary():
    instance, bridge, model = agent([decision(reply='Hola')])
    original_discover = bridge.discover

    def discover():
        skills = original_discover()
        skills[0]['parameters'] = {'private_config': 'must not reach the model'}
        return skills

    bridge.discover = discover
    instance.turn('Hola')
    messages, _ = model.calls[0]
    assert len(messages) == 2
    prompt = messages[0]['content']
    assert 'private_config' not in prompt and 'cooldown_sec' not in prompt
    assert 'reset_after_absent' not in prompt and 'Reglas automáticas' not in prompt
    assert 'wave_right' in prompt and 'Saludar' in prompt


def test_model_does_not_generate_an_action_origin_and_the_host_assigns_it():
    config, bridge, requests = load_config(CONFIG), Bridge(), []

    def transport(url, payload, timeout):
        requests.append(copy.deepcopy(payload))
        return {'message': {'content': json.dumps({
            'perception_required': False, 'reply': 'Voy a saludar.',
            'action': {'skill': 'wave_right'},
        })}}

    model = OllamaModel(config['model'], transport=transport, vision_model=config['vision_model'])
    output = MultimodalAgent(bridge, model, config).turn('Saluda')
    assert output['action'] == {'skill': 'wave_right', 'source': 'user_request'}
    assert output['reply'] == '¡Listo, ya saludé!'
    assert len(requests) == 1
    assert 'source' not in requests[0]['format']['properties']['action']['properties']


def test_watch_only_uses_configured_rules_even_if_model_suggests_a_skill():
    config, bridge = load_config(CONFIG), Bridge()
    replies = iter([
        {'description': 'Escena sin personas.', 'person_visible': False},
        {'perception_required': False, 'reply': 'Veo el suelo.',
         'action': {'skill': 'wave_right'}},
    ])

    def transport(url, payload, timeout):
        return {'message': {'content': json.dumps(next(replies))}}

    model = OllamaModel(config['model'], transport=transport, vision_model=config['vision_model'])
    output = MultimodalAgent(bridge, model, config, clock=lambda: 100.).turn('Observa', watch=True)
    assert output['action']['skill'] == 'none'
    assert bridge.called == ['look_front']
