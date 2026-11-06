"""Adaptadores HTTP de habilidades y selección de modelos de texto e imagen."""

import json
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen


class HttpFailure(RuntimeError):
    def __init__(self, code, result):
        self.code, self.result = code, result
        super().__init__(f'HTTP {code}: {result}')


def get_json(url, payload=None, timeout=15):
    data = None if payload is None else json.dumps(payload, ensure_ascii=False).encode('utf-8')
    request = Request(url, data=data, headers={'Content-Type': 'application/json'})
    try:
        with urlopen(request, timeout=timeout) as response:
            return json.load(response)
    except HTTPError as exc:
        detail = exc.read().decode('utf-8', errors='replace')
        try:
            result = json.loads(detail)
        except ValueError:
            result = {'ok': False, 'status': 'http_error', 'detail': detail}
        raise HttpFailure(exc.code, result) from exc
    except (URLError, TimeoutError) as exc:
        raise RuntimeError(f'No se pudo conectar con {url}: {exc}') from exc


class HttpSkillClient:
    def __init__(self, base_url, transport=get_json):
        self.base_url, self.transport = base_url.rstrip('/'), transport

    def discover(self):
        result = self.transport(self.base_url + '/v1/skills')
        if not isinstance(result, dict):
            raise ValueError('Catálogo HTTP inválido')
        skills = result.get('skills')
        if (not isinstance(skills, list) or not all(
            isinstance(s, dict) and isinstance(s.get('name'), str)
            and isinstance(s.get('description'), str)
            and s.get('kind', 'action') in ('action', 'perception') for s in skills
        )):
            raise ValueError('Catálogo HTTP inválido')
        return skills

    def execute(self, skill):
        return self.transport(self.base_url + '/v1/skills/execute', {'skill': skill}, timeout=180)


class OllamaModel:
    def __init__(self, model='qwen2.5:7b', url='http://127.0.0.1:11434/api/chat',
                 transport=get_json, thinking=False, vision_model='qwen3-vl:2b-instruct'):
        self.model, self.url, self.transport = model, url, transport
        self.thinking = thinking
        self.vision_model = vision_model

    def generate(self, messages, schema):
        messages = [dict(message) for message in messages]
        payload = {
            'model': self.vision_model if any(m.get('images') for m in messages) else self.model,
            'messages': messages, 'format': schema,
            'stream': False, 'think': self.thinking,
            'options': {'temperature': 0, 'num_predict': 256, 'num_ctx': 4096},
        }
        for attempt in range(2):
            response = self.transport(self.url, payload, timeout=180)
            if not isinstance(response, dict) or not isinstance(response.get('message'), dict):
                raise RuntimeError('Respuesta del modelo sin message válido')
            content = response['message'].get('content')
            try:
                return json.loads(content)
            except (TypeError, ValueError) as exc:
                if attempt or not isinstance(content, str):
                    raise RuntimeError('El modelo no devolvió JSON válido') from exc
                # Repara la sintaxis con el mismo modelo, sin ejecutar ninguna habilidad.
                payload['messages'] = [
                    *messages, {'role': 'assistant', 'content': content},
                    {'role': 'user', 'content': 'Corrige solo el formato. Devuelve un objeto JSON '
                     'sin Markdown conforme a este esquema: '
                     + json.dumps(schema, ensure_ascii=False, separators=(',', ':'))},
                ]
