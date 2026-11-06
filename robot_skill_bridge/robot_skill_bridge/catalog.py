"""Catálogo genérico: metadatos públicos y validadores de adaptadores inyectados."""

from dataclasses import dataclass, field
import math
from pathlib import Path
import threading
from typing import Any, Protocol

import yaml

from .skill_result import SkillResult


@dataclass(frozen=True)
class Skill:
    name: str
    description: str
    backend: str
    profile: Any
    kind: str = 'action'
    parameters: dict = field(default_factory=lambda: {
        'type': 'object', 'properties': {}, 'additionalProperties': False})
    public: bool = True


class SkillBackend(Protocol):
    def execute(self, skill: Skill, parameters=None, cancel=None) -> SkillResult:
        """Devuelve un resultado de dominio terminal."""
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
        schema = data.get('parameters', {'type': 'object', 'properties': {},
                                         'additionalProperties': False})
        validate_schema(schema)
        public = data.get('public', True)
        if type(public) is not bool:
            raise ValueError(f'{name}: public debe ser booleano')
        catalog[name] = Skill(name, description, backend, profile, kind, schema, public)
    return catalog


def validate_schema(schema):
    """Subconjunto explícito de JSON Schema para argumentos escalares de habilidades."""
    if (not isinstance(schema, dict) or schema.get('type') != 'object'
            or schema.get('additionalProperties') is not False
            or not isinstance(schema.get('properties'), dict)):
        raise ValueError('parameters debe ser un esquema object cerrado')
    if set(schema.get('required', []))-schema['properties'].keys():
        raise ValueError('required contiene parámetros no declarados')
    for name, spec in schema['properties'].items():
        if not isinstance(name, str) or not isinstance(spec, dict):
            raise ValueError('Propiedad de parameters inválida')
        if spec.get('type') not in ('number', 'integer', 'string', 'boolean'):
            raise ValueError('Solo se admiten parámetros escalares')
        if 'default' in spec:
            _argument(spec['default'], spec, name)


def _argument(value, spec, name):
    kind = spec['type']
    valid = {'number': type(value) in (int, float), 'integer': type(value) is int,
             'string': isinstance(value, str), 'boolean': type(value) is bool}[kind]
    if not valid:
        raise ValueError(f'{name}: tipo incorrecto')
    if kind in ('number', 'integer'):
        value = number(value, name)
        if value < spec.get('minimum', -math.inf) or value > spec.get('maximum', math.inf):
            raise ValueError(f'{name}: fuera del intervalo permitido')
    if 'enum' in spec and value not in spec['enum']:
        raise ValueError(f'{name}: valor no permitido')
    return value


def validate_parameters(skill, parameters=None):
    parameters = {} if parameters is None else parameters
    if not isinstance(parameters, dict) or set(parameters)-skill.parameters['properties'].keys():
        raise ValueError('Parámetros desconocidos o formato incorrecto')
    values = dict(parameters)
    for name, spec in skill.parameters['properties'].items():
        if name not in values and 'default' in spec:
            values[name] = spec['default']
        if name in values:
            values[name] = _argument(values[name], spec, name)
    if set(skill.parameters.get('required', []))-values.keys():
        raise ValueError('Faltan parámetros requeridos')
    return values


class BackendRouter:
    """Despacha, compone y cancela sin conocer hardware, simulador ni proveedor IA."""

    def __init__(self, backends, catalog=None):
        self.backends, self.catalog = backends, catalog or {}
        self._busy, self._cancel = threading.Lock(), threading.Event()
        self._closed = False

    def execute(self, skill, parameters=None):
        if self._closed:
            return SkillResult(False, 'unavailable', skill.name, 'El ejecutor está cerrándose')
        try:
            parameters = validate_parameters(skill, parameters)
        except ValueError as exc:
            return SkillResult(False, 'invalid_parameters', skill.name, str(exc))
        locked = skill.kind == 'action'
        if locked and not self._busy.acquire(blocking=False):
            return SkillResult(False, 'busy', skill.name)
        if locked:
            self._cancel.clear()
        try:
            return self.dispatch(skill, parameters, self._cancel)
        finally:
            if locked:
                self._busy.release()

    def dispatch(self, skill, parameters, cancel):
        if skill.kind == 'action' and cancel.is_set():
            return SkillResult(False, 'canceled', skill.name)
        return self.backends[skill.backend].execute(skill, parameters, cancel)

    def cancel(self):
        self._cancel.set()
        return SkillResult(True, 'cancel_requested', detail='Cancelación solicitada')

    def close(self):
        self._closed = True
        self._cancel.set()
        for backend in self.backends.values():
            close = getattr(backend, 'close', None)
            if callable(close):
                close()
        if self._busy.acquire(timeout=3.):
            self._busy.release()
