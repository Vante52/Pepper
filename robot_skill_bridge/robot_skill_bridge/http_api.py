"""Contrato externo independiente de Ollama/Whisper/ROS: HTTP + JSON."""

from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import threading

from .catalog import SkillBackend, validate_parameters
from .skill_result import SkillResult


def http_status(result):
    if result.ok:
        return 200
    return {'invalid_parameters': 400, 'unknown_skill': 404, 'busy': 409,
            'cancel_pending': 409, 'rejected': 422, 'timeout': 504,
            'controller_unavailable': 503, 'controller_inactive': 503,
            'state_unavailable': 503, 'perception_unavailable': 503,
            'target_not_found': 422, 'target_lost': 422, 'obstacle': 422,
            'target_unavailable': 503, 'clock_stopped': 409, 'clock_mismatch': 409, 'stalled': 422,
            'canceled': 409, 'bridge_error': 500}.get(result.status, 502)


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
                {'name': skill.name, 'description': skill.description, 'kind': skill.kind,
                 'parameters': skill.parameters}
                for skill in catalog.values() if skill.public
            ]})

        def do_POST(self):
            if self.path == '/v1/skills/cancel':
                result = backend.cancel()
                return self._json(http_status(result), result.to_dict())
            if self.path != '/v1/skills/execute':
                return self._json(404, {'ok': False, 'status': 'not_found'})
            try:
                size = int(self.headers.get('Content-Length', '0'))
                if not 0 < size <= 8192:
                    raise ValueError('Tamaño JSON inválido')
                request = json.loads(self.rfile.read(size).decode('utf-8'))
                if (not isinstance(request, dict) or 'skill' not in request
                        or set(request)-{'skill', 'parameters'}
                        or not isinstance(request['skill'], str)):
                    raise ValueError('Se espera skill y, opcionalmente, parameters')
            except (ValueError, UnicodeDecodeError) as exc:
                return self._json(400, {'ok': False, 'status': 'bad_request',
                                        'detail': str(exc)})
            name = request['skill']
            skill = catalog.get(name)
            if skill is None or not skill.public:
                return self._json(404, {'ok': False, 'status': 'unknown_skill',
                                        'detail': 'Habilidad no autorizada'})
            try:
                parameters = validate_parameters(skill, request.get('parameters'))
            except ValueError as exc:
                return self._json(400, {'ok': False, 'status': 'invalid_parameters',
                                        'detail': str(exc)})
            holds_motion_lock = skill.kind == 'action'
            if holds_motion_lock and not busy.acquire(blocking=False):
                return self._json(409, {'ok': False, 'status': 'busy',
                                        'detail': 'Otra habilidad está en ejecución'})
            try:
                result = backend.execute(skill, parameters)
                if not isinstance(result, SkillResult):
                    raise TypeError('El backend debe devolver SkillResult')
            except Exception as exc:
                result = SkillResult(False, 'bridge_error', skill.name, str(exc))
            finally:
                if holds_motion_lock:
                    busy.release()
            self._json(http_status(result), result.to_dict())

        def log_message(self, fmt, *args):
            pass

    server = ThreadingHTTPServer((host, port), Handler)
    server.daemon_threads = True
    return server
