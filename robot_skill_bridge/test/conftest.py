"""Las pruebas que mueven el robot requieren una opción explícita."""

from pathlib import Path

import pytest


def pytest_addoption(parser):
    parser.addoption('--run-robot-integration', action='store_true', default=False)
    parser.addoption('--bridge-url', default='http://127.0.0.1:8765')
    parser.addoption('--skills-config', default=str(
        Path(__file__).resolve().parents[1] / 'config/pepper_skills.yaml',
    ))


def pytest_collection_modifyitems(config, items):
    if not config.getoption('--run-robot-integration'):
        for item in items:
            if 'integration' in item.keywords:
                item.add_marker(pytest.mark.skip(
                    reason='Use --run-robot-integration con el robot activo'))
