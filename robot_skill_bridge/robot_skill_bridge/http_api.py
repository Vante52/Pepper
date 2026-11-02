"""Contrato externo independiente de Ollama/Whisper/ROS: HTTP + JSON."""

import json
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from .catalog import SkillBackend


def create_server(host, port, catalog, backend: SkillBackend):
    busy = threading.Lock()  # Una habilidad a la vez en este MVP.

    class Handler(BaseHTTPRequestHandler):
        def _json(self, status, obj):
            body = json.dumps(obj, ensure_ascii=False).encode('utf-8')
            self.send_response(status)
            self.send_header('Content-Type', 'application/json; charset=utf-8')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            try:
                self.wfile.write(body)
            except (BrokenPipeError, ConnectionResetError):
                # Desconectar HTTP no implica haber cancelado una acción robótica.
                pass

        def do_GET(self):
            if self.path != '/v1/skills':
                return self._json(404, {'ok': False, 'status': 'not_found'})
            self._json(200, {'ok': True, 'skills': [
                {'name': skill.name, 'description': skill.description}
                for skill in catalog.values()
            ]})

        def do_POST(self):
            if self.path != '/v1/skills/execute':
                return self._json(404, {'ok': False, 'status': 'not_found'})
            try:
                size = int(self.headers.get('Content-Length', '0'))
                if not 0 < size <= 8192:
                    raise ValueError('Tamaño JSON inválido')
                request = json.loads(self.rfile.read(size).decode('utf-8'))
                if (not isinstance(request, dict) or set(request) != {'skill'}
                        or not isinstance(request['skill'], str)):
                    raise ValueError('Se espera exclusivamente {"skill": "nombre"}')
            except (ValueError, UnicodeDecodeError) as exc:
                return self._json(400, {'ok': False, 'status': 'bad_request',
                                        'detail': str(exc)})
            name = request['skill']
            skill = catalog.get(name)
            if skill is None:
                return self._json(404, {'ok': False, 'status': 'unknown_skill',
                                        'detail': 'Habilidad no autorizada'})
            if not busy.acquire(blocking=False):
                return self._json(409, {'ok': False, 'status': 'busy',
                                        'detail': 'Otra habilidad está en ejecución'})
            try:
                status, result = backend.execute(skill)
            except Exception as exc:
                status, result = 500, {'ok': False, 'status': 'bridge_error',
                                       'detail': str(exc)}
            finally:
                busy.release()
            self._json(status, result)

        def log_message(self, fmt, *args):
            pass

    server = ThreadingHTTPServer((host, port), Handler)
    server.daemon_threads = True
    return server
