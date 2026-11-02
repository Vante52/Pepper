#!/usr/bin/env python3
"""Adaptador experimental: texto -> intención Ollama -> contrato HTTP de habilidades.

NO importa ROS 2. Cualquier otro modelo puede sustituir esta implementación.
"""

import argparse
import json
import os
import sys
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

BRIDGE = os.getenv('SKILL_BRIDGE_URL', 'http://127.0.0.1:8765')
OLLAMA = os.getenv('OLLAMA_URL', 'http://127.0.0.1:11434/api/chat')
MODEL = os.getenv('OLLAMA_MODEL', 'qwen2.5:7b')


class HttpFailure(RuntimeError):
    def __init__(self, code, result):
        self.code, self.result = code, result
        super().__init__(f'HTTP {code}: {result}')


def get_json(url, payload=None, timeout=15):
    data = None if payload is None else json.dumps(payload, ensure_ascii=False).encode('utf-8')
    req = Request(url, data=data, headers={'Content-Type': 'application/json'})
    try:
        with urlopen(req, timeout=timeout) as response:
            return json.loads(response.read().decode('utf-8'))
    except HTTPError as exc:
        detail = exc.read().decode('utf-8', errors='replace')
        try:
            result = json.loads(detail)
        except ValueError:
            result = {'ok': False, 'status': 'http_error', 'detail': detail}
        raise HttpFailure(exc.code, result) from exc
    except (URLError, TimeoutError) as exc:
        raise RuntimeError(f'No se pudo conectar con {url}: {exc}') from exc


def interpret(text, skills):
    """El LLM solo clasifica en un nombre permitido o 'none'."""
    permitted = [item['name'] for item in skills]
    if not permitted:
        raise RuntimeError('El puente no expone habilidades')
    schema = {
        'type': 'object',
        'properties': {
            'intent': {'type': 'string', 'enum': ['execute_skill', 'unsupported']},
            'skill': {'type': 'string', 'enum': permitted + ['none']},
        },
        'required': ['intent', 'skill'],
        'additionalProperties': False,
    }
    definitions = '\n'.join(f"- {x['name']}: {x['description']}" for x in skills)
    payload = {
        'model': MODEL,
        'stream': False,
        'format': schema,
        'messages': [
            {'role': 'system', 'content': (
                'Eres un enrutador de habilidades robóticas. '
                'Responde SOLO JSON válido según el schema. '
                'No controles motores ni inventes habilidades. '
                'Para una instrucción de movimiento conocida, usa intent=execute_skill '
                'y skill exactamente igual al nombre permitido. '
                'Si no es una solicitud de habilidad disponible, '
                'usa intent=unsupported y skill=none.\n'
                'Capacidades disponibles:\n' + definitions
            )},
            {'role': 'user', 'content': text},
        ],
        'options': {'temperature': 0, 'num_predict': 90},
    }
    response = get_json(OLLAMA, payload, timeout=90)
    if not isinstance(response, dict) or not isinstance(response.get('message'), dict):
        raise RuntimeError('Respuesta del proveedor sin message válido')
    content = response['message'].get('content', '')
    try:
        output = json.loads(content)
    except (json.JSONDecodeError, TypeError) as exc:
        raise RuntimeError(f'LLM devolvió contenido no JSON: {content!r}') from exc
    if not isinstance(output, dict) or set(output) != {'intent', 'skill'}:
        raise RuntimeError(f'Contrato LLM inválido: {output!r}')
    intent, skill = output['intent'], output['skill']
    if intent == 'unsupported' and skill == 'none':
        return None
    if intent != 'execute_skill' or skill not in permitted:
        raise RuntimeError(f'Intención inconsistente o no permitida: {output!r}')
    return skill


def main():
    parser = argparse.ArgumentParser(description='Demo agnóstica de ROS 2: LLM -> habilidad')
    parser.add_argument('text', nargs='?', help='Instrucción natural')
    parser.add_argument('--skill', help='Evitar Ollama y probar directamente la interfaz')
    args = parser.parse_args()
    try:
        skills = get_json(BRIDGE + '/v1/skills')['skills']
        if args.skill:
            allowed = [item['name'] for item in skills]
            if args.skill not in allowed:
                raise RuntimeError(
                    f'Habilidad no registrada: {args.skill}. Disponibles: {allowed}')
            selected = args.skill
        else:
            text = args.text or input('Instrucción > ')
            selected = interpret(text, skills)
        if selected is None:
            print('No hay una habilidad disponible para esa instrucción; no se ejecutó nada.')
            return 0
        print(f'[Intención validada] skill={selected}')
        response = get_json(BRIDGE + '/v1/skills/execute', {'skill': selected}, timeout=90)
        print(json.dumps(response, ensure_ascii=False, indent=2))
        return 0 if response.get('ok') else 1
    except HttpFailure as exc:
        print(json.dumps(exc.result, ensure_ascii=False, indent=2), file=sys.stderr)
        return 1
    except (RuntimeError, ValueError, KeyError) as exc:
        print(f'[ERROR] {exc}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
