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
        elif data.get('type') in ('ros2_camera', 'ros2_target_camera'):
            from .ros2_camera_provider import ROS2CameraProvider
            topic = data.get('topic')
            if not isinstance(topic, str) or not topic.startswith('/'):
                raise ValueError(f'{name}: topic debe ser absoluto')
            timeout = number(data.get('timeout_sec', 3.), 'timeout_sec', positive=True)
            if data['type'] == 'ros2_camera':
                providers[name] = ROS2CameraProvider(node, topic, timeout, size)
            else:
                from .ros2_target_provider import ROS2TargetCameraProvider
                arguments = {}
                for key in ('info_topic', 'points_topic', 'odom_topic', 'base_frame'):
                    value = data.get(key)
                    if not isinstance(value, str) or not value.strip():
                        raise ValueError(f'{name}: falta {key}')
                    arguments[key] = value
                for key, default in [('sync_tolerance_sec', .1), ('max_context_age_sec', 60.)]:
                    arguments[key] = number(data.get(key, default), key, positive=True)
                for key in ('target_min_z_m', 'target_max_z_m'):
                    if key in data:
                        arguments[key] = number(data[key], key)
                providers[name] = ROS2TargetCameraProvider(
                    node, topic, timeout_sec=timeout, max_dimension=size, **arguments)
        else:
            raise ValueError(f'{name}: tipo de proveedor no registrado')
    return providers
