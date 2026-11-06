"""Contratos y proveedores intercambiables de percepción."""

import base64
from concurrent.futures import ThreadPoolExecutor
from io import BytesIO
import threading
import time
from unittest.mock import Mock

from PIL import Image
import pytest
from robot_skill_bridge.catalog import Skill
from robot_skill_bridge.image_file_provider import ImageFileProvider
from robot_skill_bridge.perception import MockPerceptionProvider, Observation, PerceptionBackend


def test_image_file_normalizes_and_limits_image_size(tmp_path):
    path = tmp_path / 'image.jpg'
    Image.new('RGB', (1200, 800), 'red').save(path)
    observation = ImageFileProvider(path, max_dimension=320).observe()
    with Image.open(BytesIO(observation.image)) as image:
        assert image.size == (320, 213)
        assert image.mode == 'RGB'
    assert observation.media_type == 'image/png'
    assert time.time() - observation.captured_at < 1.
    assert observation.sensor_stamp is None  # No inventar fecha de captura de un archivo.


def test_mock_provider_and_backend_follow_the_same_public_contract():
    observation = Observation(b'image', 'image/png', 100., 'mock')
    backend = PerceptionBackend({'front': MockPerceptionProvider([observation])})
    skill = Skill('look', 'Observar', 'perception', 'front', 'perception')
    result = backend.execute(skill)
    output = result.to_dict()
    assert result.ok and result.status == 'observed'
    assert base64.b64decode(output['observation']['image_base64']) == b'image'
    assert not backend.execute(skill).ok  # Sin observaciones, no inventa otra.


def test_missing_file_is_an_explicit_perception_error(tmp_path):
    backend = PerceptionBackend({'front': ImageFileProvider(tmp_path / 'missing.png')})
    result = backend.execute(Skill('look', 'Observar', 'perception', 'front'))
    assert not result.ok and result.status == 'perception_unavailable'


def test_ros_provider_requires_a_new_frame_and_preserves_optical_frame():
    pytest.importorskip('rclpy')
    from sensor_msgs.msg import Image as RosImage
    from robot_skill_bridge.ros2_camera_provider import ROS2CameraProvider
    node = Mock()
    provider = ROS2CameraProvider(node, '/camera', timeout_sec=.2)
    message = RosImage()
    message.header.frame_id = 'camera_optical'
    message.header.stamp.sec = 42
    message.width, message.height, message.step = 2, 1, 6
    message.encoding, message.data = 'rgb8', [255, 0, 0, 0, 255, 0]
    started = threading.Event()

    original_wait = provider.condition.wait_for

    def wait_for(predicate, timeout):
        started.set()
        return original_wait(predicate, timeout)

    provider.condition.wait_for = wait_for
    with ThreadPoolExecutor() as executor:
        future = executor.submit(provider.observe)
        assert started.wait(1.)
        # Esperar a que observe libere el mutex al entrar en la condición.
        with provider.condition:
            provider._receive(message)
        observation = future.result(timeout=1.)
    assert observation.frame_id == 'camera_optical' and observation.sensor_stamp == 42.
    assert observation.source == '/camera'
    with pytest.raises(RuntimeError, match='nueva'):
        provider.observe()  # No reutilizar indefinidamente el último frame.


def test_perception_via_existing_http_skill_contract(tmp_path):
    from urllib.request import Request, urlopen
    import json
    from robot_skill_bridge.catalog import BackendRouter
    from robot_skill_bridge.http_api import create_server
    path = tmp_path / 'scene.png'
    Image.new('RGB', (40, 30), 'green').save(path)
    skill = Skill('look', 'Observar escena', 'perception', 'front', 'perception')
    router = BackendRouter({'perception': PerceptionBackend({'front': ImageFileProvider(path)})})
    server = create_server('127.0.0.1', 0, {'look': skill}, router)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    base = f'http://127.0.0.1:{server.server_address[1]}'
    try:
        with urlopen(base + '/v1/skills', timeout=3) as response:
            assert json.load(response)['skills'][0]['kind'] == 'perception'
        request = Request(base + '/v1/skills/execute', data=b'{"skill":"look"}',
                          headers={'Content-Type': 'application/json'})
        with urlopen(request, timeout=3) as response:
            result = json.load(response)
        assert result['status'] == 'observed'
        data = base64.b64decode(result['observation']['image_base64'])
        with Image.open(BytesIO(data)) as image:
            assert image.size == (40, 30)
    finally:
        server.shutdown()
        server.server_close()
        thread.join(3)
