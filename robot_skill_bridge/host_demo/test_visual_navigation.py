"""Composición genérica de visión/acción con proveedores deterministas."""

import base64
import copy
import math
from pathlib import Path

from host_demo.clients import HttpSkillClient
from host_demo.multimodal_agent import load_config, MultimodalAgent
from host_demo.visual_navigation import approach_plan, navigation_capability, VisualApproach
import pytest


CONFIG = Path(__file__).with_name('agent_config.json')


def navigation_config():
    return load_config(CONFIG)['visual_navigation']


def found(status='found'):
    return {'status': status, 'confidence': .9, 'bbox_2d': [400, 200, 600, 800]}


class Bridge:
    def __init__(self, distances=(1.2, .81), metric=True):
        self.distances = iter(distances)
        self.calls = []
        self.metric = metric
        self.motion_ok = True

    def discover(self):
        return [
            {'name': 'move_relative', 'description': 'Mover sin visión', 'parameters': {
                'type': 'object', 'additionalProperties': False,
                'required': ['distance_m', 'direction_deg'], 'properties': {
                    'distance_m': {'type': 'number', 'minimum': 0., 'maximum': 5.},
                    'direction_deg': {'type': 'number', 'minimum': -180., 'maximum': 180.}}}},
            {'name': 'turn_relative', 'description': 'Girar sin visión', 'parameters': {
                'type': 'object', 'additionalProperties': False,
                'required': ['angle_deg'], 'properties': {
                    'angle_deg': {'type': 'number', 'minimum': -360., 'maximum': 360.}}}},
            {'name': 'look_front', 'description': 'Imagen', 'kind': 'perception'},
            {'name': 'locate_target', 'description': 'Medir', 'kind': 'perception',
             'parameters': {'required': ['observation_id']}},
        ]

    def execute(self, name, parameters=None):
        self.calls.append((name, copy.deepcopy(parameters)))
        if name == 'look_front':
            observation = {'image_base64': base64.b64encode(b'image').decode(),
                           'media_type': 'image/png', 'captured_at': 100.}
            if self.metric:
                observation['observation_id'] = 'token'
            return {'ok': True, 'observation': observation}
        if name == 'locate_target':
            distance = next(self.distances)
            return {'ok': True, 'target': {
                'x_m': distance, 'y_m': 0., 'distance_m': distance, 'bearing_deg': 0.,
                'frame_id': 'base', 'observation_id': 'token'}}
        return {'ok': self.motion_ok, 'status': 'succeeded' if self.motion_ok else 'stalled'}


class Model:
    def __init__(self, responses):
        self.responses, self.calls = iter(responses), []

    def generate(self, messages, schema):
        self.calls.append((copy.deepcopy(messages), copy.deepcopy(schema)))
        return next(self.responses)


@pytest.mark.parametrize('description', ['un cono rojo', 'la esfera morada', 'un cilindro azul',
                                         'la caja con una estrella'])
def test_free_text_target_is_passed_to_vision_without_color_or_shape_rules(description):
    bridge, model = Bridge(), Model([found(), found()])
    approach = VisualApproach(bridge, model, navigation_config(),
                              clock=lambda: 100.)
    result = approach.execute({'target': description})
    assert result['ok'] and result['final_distance_m'] == .81
    assert [name for name, _ in bridge.calls] == [
        'look_front', 'locate_target', 'move_relative', 'look_front', 'locate_target']
    assert all(messages[-1]['content'] == description for messages, _ in model.calls)
    movement = bridge.calls[2][1]
    assert movement['distance_m'] == pytest.approx(.4) and movement['direction_deg'] == 0.


def test_search_turns_in_bounded_steps_then_reobserves_before_translation():
    bridge, model = Bridge((.8,)), Model([found('missing'), found()])
    result = VisualApproach(bridge, model, navigation_config(),
                            clock=lambda: 100.).execute({'target': 'esfera morada'})
    assert result['ok']
    assert bridge.calls[1] == ('turn_relative', {'angle_deg': 45.})
    assert not any(name == 'move_relative' for name, _ in bridge.calls)


@pytest.mark.parametrize('status', ['ambiguous', 'found'])
def test_ambiguity_and_low_confidence_never_produce_motion(status):
    bridge, detection = Bridge(), found(status)
    detection['confidence'] = .2
    result = VisualApproach(bridge, Model([detection]),
                            navigation_config(), clock=lambda: 100.).execute(
                                {'target': 'cilindro'})
    assert not result['ok'] and result['status'] == 'target_ambiguous'
    assert [name for name, _ in bridge.calls] == ['look_front']


def test_recorded_rgb_image_cannot_authorize_metric_navigation():
    bridge = Bridge(metric=False)
    result = VisualApproach(bridge, Model([found()]),
                            navigation_config(), clock=lambda: 100.).execute(
                                {'target': 'cilindro'})
    assert not result['ok'] and result['status'] == 'target_unavailable'
    assert len(bridge.calls) == 1


def test_stale_detection_stops_without_locating_or_moving():
    bridge = Bridge()
    result = VisualApproach(bridge, Model([found()]),
                            navigation_config(), clock=lambda: 200.).execute(
                                {'target': 'cilindro'})
    assert not result['ok'] and len(bridge.calls) == 1


def test_a_spurious_full_width_box_cannot_authorize_translation():
    bridge, detection = Bridge(), found()
    detection['bbox_2d'] = [0, 570, 1000, 999]
    result = VisualApproach(bridge, Model([detection]),
                            navigation_config(), clock=lambda: 100.).execute(
                                {'target': 'esfera morada'})
    assert not result['ok'] and len(bridge.calls) == 1


def test_lost_target_stops_after_the_last_verified_step():
    bridge = Bridge()
    result = VisualApproach(bridge, Model([found(), found('missing')]),
                            navigation_config(), clock=lambda: 100.).execute(
                                {'target': 'cilindro'})
    assert not result['ok'] and result['status'] == 'target_lost'
    assert sum(name == 'move_relative' for name, _ in bridge.calls) == 1


def test_failed_step_never_retries_or_claims_arrival():
    bridge = Bridge()
    bridge.motion_ok = False
    result = VisualApproach(bridge, Model([found()]),
                            navigation_config(), clock=lambda: 100.).execute(
                                {'target': 'cilindro'})
    assert not result['ok'] and result['status'] == 'stalled'
    assert len(bridge.calls) == 3


def test_not_found_stops_after_one_search_revolution():
    bridge = Bridge()
    result = VisualApproach(bridge, Model([found('missing')]*9),
                            navigation_config(), clock=lambda: 100.).execute(
                                {'target': 'cilindro'})
    assert not result['ok'] and result['status'] == 'target_not_found'
    assert sum(p['angle_deg'] for n, p in bridge.calls if n == 'turn_relative') == 360.


def test_direct_move_uses_arguments_and_never_acquires_images_or_reinfers():
    bridge = Bridge()
    model = Model([{'reply': 'Voy.', 'perception_required': False,
                    'action': {'skill': 'move_relative', 'parameters': {
                        'distance_m': .5, 'direction_deg': 90.}}}])
    agent = MultimodalAgent(bridge, model, load_config(CONFIG), clock=lambda: 100.)
    output = agent.turn('Muévete medio metro a la izquierda')
    assert output['result']['ok']
    assert bridge.calls == [('move_relative', {'distance_m': .5, 'direction_deg': 90.})]
    assert len(model.calls) == 1
    assert not any(m.get('images') for m in model.calls[0][0])


def test_agent_composes_target_action_without_a_post_execution_inference():
    bridge = Bridge()
    model = Model([{'reply': 'Busco el objetivo.', 'perception_required': False,
                    'action': {'skill': 'approach_object', 'parameters': {
                        'target': 'la esfera morada'}}}, found(), found()])
    output = MultimodalAgent(bridge, model, load_config(CONFIG), clock=lambda: 100.).turn(
        'Busca la esfera morada y acércate')
    assert output['result']['ok'] and len(model.calls) == 3
    assert output['reply'].startswith('Listo, me acerqué')
    assert all(call[0][-1].get('images') for call in model.calls[1:])


def test_http_arguments_are_an_additive_change():
    calls = []
    client = HttpSkillClient('http://example', lambda url, payload, timeout: calls.append(payload))
    client.execute('wave_right')
    client.execute('move_relative', {'distance_m': .2, 'direction_deg': 0.})
    assert calls == [{'skill': 'wave_right'}, {'skill': 'move_relative', 'parameters': {
        'distance_m': .2, 'direction_deg': 0.}}]


def test_lost_http_response_requests_cancel_without_repeating_the_motion():
    calls = []

    def transport(url, payload, timeout):
        calls.append(url)
        if url.endswith('/execute'):
            raise RuntimeError('connection lost')
        return {'ok': True, 'status': 'cancel_requested'}

    client = HttpSkillClient('http://example', transport)
    with pytest.raises(RuntimeError, match='connection lost'):
        client.execute('move_relative', {'distance_m': .2, 'direction_deg': 0.})
    assert calls == ['http://example/v1/skills/execute', 'http://example/v1/skills/cancel']


def test_watch_parameters_from_the_model_cannot_override_a_configured_reactive_action():
    from host_demo.test_demo_ollama import agent, decision
    proposed = decision(True, 'wave_right', 'observation')
    proposed['action']['parameters'] = {'distance_m': 5.}
    instance, bridge, _ = agent([proposed])
    output = instance.turn('Observa', watch=True)
    assert output['action'] == {'skill': 'wave_right', 'source': 'observation'}
    assert bridge.called == ['look_front', 'wave_right']


def test_discriminated_schema_couples_each_skill_to_its_own_arguments():
    from host_demo.multimodal_agent import decision_schema
    actions = [s for s in Bridge().discover() if s.get('kind', 'action') == 'action']
    actions.append(navigation_capability(navigation_config()))
    variants = decision_schema([s['name'] for s in actions], actions)[
        'properties']['action']['oneOf']
    branches = {v['properties']['skill']['enum'][0]: v for v in variants}
    assert set(branches['move_relative']['properties']['parameters']['properties']) == {
        'distance_m', 'direction_deg'}
    assert set(branches['approach_object']['properties']['parameters']['properties']) == {
        'target', 'stop_distance_m'}
    assert 'parameters' in branches['move_relative']['required']
    assert branches['none']['properties']['parameters']['properties'] == {}


def test_unknown_model_arguments_request_clarification_without_a_transport_call():
    bridge = Bridge()
    model = Model([{'reply': 'Avanzo.', 'perception_required': False,
                    'action': {'skill': 'move_relative', 'parameters': {'linear_x': .5}}}])
    output = MultimodalAgent(bridge, model, load_config(CONFIG)).turn('muevete hacia adelante')
    assert output['action']['skill'] == 'none' and output.get('decision_error')
    assert not bridge.calls


@pytest.mark.parametrize('point,expected,bearing', [
    ((3., 4.), (2.52, 3.36), 53.130102354156),
    ((0., -2.), (0., -1.2), -90.), ((-2., 0.), (-1.2, 0.), 180.),
])
def test_complete_goal_is_computed_from_measured_xy(point, expected, bearing):
    plan = approach_plan({'x_m': point[0], 'y_m': point[1], 'frame_id': 'robot'}, .8)
    assert tuple(plan['goal_in_initial_base'].values()) == pytest.approx(expected)
    assert plan['command']['direction_deg'] == pytest.approx(bearing)
    assert plan['command']['distance_m'] == pytest.approx(math.hypot(*point)-.8)


@pytest.mark.parametrize('target', [
    None, {}, {'x_m': float('nan'), 'y_m': 1.},
    {'x_m': True, 'y_m': 1.}, {'x_m': 0., 'y_m': 0.}])
def test_invalid_sensor_positions_cannot_produce_a_movement_plan(target):
    with pytest.raises(ValueError):
        approach_plan(target, .8)


def test_exactly_one_full_movement_is_sent_after_the_plan_is_computed():
    bridge, plans = Bridge((3., .82)), []
    original = bridge.execute

    def execute(name, parameters=None):
        if name == 'move_relative':
            assert len(plans) == 1 and parameters == plans[0]['command']
        return original(name, parameters)

    bridge.execute = execute
    result = VisualApproach(bridge, Model([found(), found()]), navigation_config(),
                            clock=lambda: 100., on_plan=plans.append).execute({'target': 'cubo'})
    movements = [p for name, p in bridge.calls if name == 'move_relative']
    assert result['ok'] and len(movements) == 1
    assert movements[0] == {'distance_m': 2.2, 'direction_deg': 0.}
    assert result['plan']['goal_in_initial_base'] == {'x_m': 2.2, 'y_m': 0.}


def test_a_goal_beyond_the_budget_is_rejected_without_truncating_it():
    bridge, config = Bridge((3.,)), navigation_config()
    config['max_travel_m'] = 1.
    result = VisualApproach(bridge, Model([found()]), config, clock=lambda: 100.).execute(
        {'target': 'objeto'})
    assert not result['ok'] and result['status'] == 'travel_limit'
    assert result['plan']['command']['distance_m'] == 2.2
    assert not any(name == 'move_relative' for name, _ in bridge.calls)


def test_final_range_error_does_not_trigger_additional_movements():
    bridge = Bridge((2., 1.3))
    result = VisualApproach(bridge, Model([found(), found()]), navigation_config(),
                            clock=lambda: 100.).execute({'target': 'objeto'})
    assert not result['ok'] and result['status'] == 'verification_failed'
    assert result['final_distance_m'] == 1.3
    assert sum(name == 'move_relative' for name, _ in bridge.calls) == 1
