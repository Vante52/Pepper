"""Composición de proveedores configurados; ROS solo se importa si se necesita."""

from pathlib import Path

import yaml

from .catalog import number
from .image_file_provider import ImageFileProvider


def create_perception_providers(node, config_path):
    config_path = Path(config_path)
    config = yaml.safe_load(config_path.read_text(encoding='utf-8'))
    definitions = config.get('perception_providers', {})
    if not isinstance(definitions, dict):
        raise ValueError('perception_providers debe ser un mapa')
    providers = {}
    for name, data in definitions.items():
        if not isinstance(data, dict):
            raise ValueError(f'Proveedor inválido: {name}')
        size = data.get('max_dimension', 640)
        if type(size) is not int or not 32 <= size <= 2048:
            raise ValueError(f'{name}: max_dimension debe estar entre 32 y 2048')
        if data.get('type') == 'image_file':
            path = data.get('path')
            if not isinstance(path, str) or not path:
                raise ValueError(f'{name}: falta path')
            path = Path(path).expanduser()
            if not path.is_absolute():
                path = config_path.parent / path
            providers[name] = ImageFileProvider(path, size)
        elif data.get('type') == 'ros2_camera':
            from .ros2_camera_provider import ROS2CameraProvider
            topic = data.get('topic')
            if not isinstance(topic, str) or not topic.startswith('/'):
                raise ValueError(f'{name}: topic debe ser absoluto')
            timeout = number(data.get('timeout_sec', 3.), 'timeout_sec', positive=True)
            providers[name] = ROS2CameraProvider(node, topic, timeout, size)
        else:
            raise ValueError(f'{name}: tipo de proveedor no registrado')
    return providers
