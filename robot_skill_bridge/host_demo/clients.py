"""Adaptadores HTTP de habilidades y del único proveedor de IA utilizado."""

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
    def __init__(self, model='qwen3.5:4b', url='http://127.0.0.1:11434/api/chat',
                 transport=get_json, thinking=False):
        self.model, self.url, self.transport = model, url, transport
        self.thinking = thinking

    def generate(self, messages, schema):
        messages = [dict(message) for message in messages]
        contract = ('\nResponde SOLO JSON válido, sin Markdown. Empieza por { y termina por }. '
                    'Esquema: '
                    + json.dumps(schema))
        contract += '\nForma de la respuesta (rellena los valores): ' + json.dumps(example(schema))
        if messages and messages[0]['role'] == 'system':
            messages[0]['content'] += contract
        else:
            messages.insert(0, {'role': 'system', 'content': contract})
        payload = {
            'model': self.model, 'messages': messages, 'format': schema,
            'stream': False, 'think': self.thinking,
            'options': {'temperature': 0, 'num_predict': 1024, 'num_ctx': 8192},
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
                    {'role': 'user', 'content': 'Salida inválida. Devuelve solo el objeto JSON '
                     'del esquema, sin explicaciones ni Markdown.'},
                ]


def example(schema):
    if 'enum' in schema:
        return schema['enum'][-1]
    if schema.get('type') == 'object':
        return {key: example(value) for key, value in schema.get('properties', {}).items()}
    if isinstance(schema.get('type'), list) and 'null' in schema['type']:
        return None
    return 'respuesta breve'
