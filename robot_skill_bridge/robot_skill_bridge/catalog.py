"""Catálogo genérico: metadatos públicos y validadores de adaptadores inyectados."""

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Protocol

import yaml


@dataclass(frozen=True)
class Skill:
    name: str
    description: str
    backend: str
    profile: Any
    kind: str = 'action'


class SkillBackend(Protocol):
    def execute(self, skill: Skill) -> tuple[int, dict]:
        """Devuelve código HTTP y resultado terminal estructurado."""
        ...


def number(value, label, positive=False):
    if isinstance(value, bool) or not isinstance(value, (float, int)):
        raise ValueError(f'{label}: se necesita un número')
    value = float(value)
    if not math.isfinite(value) or (positive and value <= 0):
        raise ValueError(f'{label}: valor no válido')
    return value


def load_catalog(path, validators):
    """Los adaptadores interpretan perfiles; el núcleo solo conoce habilidades."""
    config = yaml.safe_load(Path(path).read_text(encoding='utf-8'))
    if (not isinstance(config, dict) or type(config.get('version')) is not int
            or config['version'] != 1):
        raise ValueError('Catálogo: version debe ser 1')
    entries = config.get('skills')
    if not isinstance(entries, dict) or not entries:
        raise ValueError('Catálogo: skills debe ser un mapa no vacío')
    catalog = {}
    for name, data in entries.items():
        if not isinstance(name, str) or not name.isidentifier() or not isinstance(data, dict):
            raise ValueError(f'Habilidad inválida: {name!r}')
        description, backend = data.get('description'), data.get('backend')
        if not isinstance(description, str) or not description.strip():
            raise ValueError(f'{name}: falta description')
        if not isinstance(backend, str) or backend not in validators:
            raise ValueError(f'{name}: backend no registrado: {backend!r}')
        profile = validators[backend](name, data)
        kind = data.get('kind', 'action')
        if kind not in ('action', 'perception'):
            raise ValueError(f'{name}: kind debe ser action o perception')
        catalog[name] = Skill(name, description, backend, profile, kind)
    return catalog


class BackendRouter:
    """Despacha por contrato; no importa implementaciones de hardware ni de IA."""

    def __init__(self, backends):
        self.backends = backends

    def execute(self, skill):
        return self.backends[skill.backend].execute(skill)
