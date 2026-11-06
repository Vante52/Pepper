"""Composición acotada de habilidades, sin dependencias de ROS o del modelo IA."""

from .skill_result import SkillResult


def parse_sequence(name, data):
    steps = data.get('steps')
    if not isinstance(steps, list) or not 1 <= len(steps) <= 12:
        raise ValueError(f'{name}: steps debe contener entre 1 y 12 pasos')
    for step in steps:
        if (not isinstance(step, dict) or set(step)-{'skill', 'parameters', 'pass_parameters'}
                or not isinstance(step.get('skill'), str)
                or not isinstance(step.get('parameters', {}), dict)
                or type(step.get('pass_parameters', False)) is not bool):
            raise ValueError(f'{name}: paso de secuencia inválido')
    return steps


def validate_sequences(catalog):
    def visit(name, chain):
        if name not in catalog:
            raise ValueError(f'Secuencia: habilidad inexistente {name}')
        if name in chain or len(chain) > 4:
            raise ValueError('Secuencia cíclica o demasiado profunda')
        if catalog[name].backend == 'sequence':
            for step in catalog[name].profile:
                visit(step['skill'], (*chain, name))
    for name in catalog:
        visit(name, ())


class SequenceBackend:
    def __init__(self, router):
        self.router = router

    def execute(self, skill, parameters=None, cancel=None):
        from .catalog import validate_parameters
        results = []
        for step in skill.profile:
            selected = self.router.catalog[step['skill']]
            arguments = dict(step.get('parameters', {}))
            if step.get('pass_parameters'):
                arguments.update(parameters or {})
            try:
                arguments = validate_parameters(selected, arguments)
            except ValueError as exc:
                return SkillResult(False, 'invalid_parameters', skill.name, str(exc))
            result = self.router.dispatch(selected, arguments, cancel)
            results.append(result.to_dict())
            if not result.ok:
                return SkillResult(False, result.status, skill.name, result.detail,
                                   {'steps': results, 'failed_step': selected.name})
        return SkillResult(True, 'succeeded', skill.name,
                           'Secuencia completada y verificada', {'steps': results})
