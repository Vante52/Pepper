"""Adaptador ROS 2: acción estándar, estado del controlador y evidencia medida.

El executor ROS gira en su propio hilo. El hilo HTTP espera eventos; nunca gira
un segundo executor sobre el mismo nodo.
"""

import threading
import time

from action_msgs.msg import GoalStatus

from builtin_interfaces.msg import Duration

from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance

from controller_manager_msgs.srv import ListControllers

from rclpy.action import ActionClient

from trajectory_msgs.msg import JointTrajectoryPoint
from .joint_observer import JointObserver


def to_duration(seconds):
    whole = int(seconds)
    nanos = round((seconds - whole) * 1_000_000_000)
    if nanos == 1_000_000_000:
        whole, nanos = whole + 1, 0
    return Duration(sec=whole, nanosec=nanos)


class TrajectoryBackend:
    def __init__(self, node):
        self.node = node
        self.clients = {}
        self.managers = {}
        self.observers = {}
        self._cancel_pending = threading.Event()
        self._busy = threading.Lock()

    def execute(self, skill):
        if not self._busy.acquire(blocking=False):
            return 409, {'ok': False, 'status': 'busy', 'skill': skill.name}
        try:
            return self._execute(skill)
        except Exception as exc:
            return 502, {'ok': False, 'status': 'ros_error', 'skill': skill.name,
                         'detail': str(exc)}
        finally:
            self._busy.release()

    def _controller_error(self, profile):
        manager = profile.controller_manager.rstrip('/') + '/list_controllers'
        if manager not in self.managers:
            self.managers[manager] = self.node.create_client(ListControllers, manager)
        client = self.managers[manager]
        if not client.wait_for_service(timeout_sec=3.0):
            return 'controller_unavailable', f'Servicio no disponible: {manager}'
        future = client.call_async(ListControllers.Request())
        finished = threading.Event()
        future.add_done_callback(lambda _: finished.set())
        if not finished.wait(3.0):
            return 'controller_unavailable', f'Sin respuesta de {manager}'
        controllers = {c.name: c for c in future.result().controller}
        controller = controllers.get(profile.controller)
        if controller is None or controller.state != 'active':
            return 'controller_inactive', f'{profile.controller} no está activo'
        return None

    def _execute(self, skill):
        profile = skill.profile

        def failure(code, status, detail, **extra):
            return code, {'ok': False, 'skill': skill.name, 'status': status,
                          'detail': detail, **extra}

        if self._cancel_pending.is_set():
            return failure(409, 'cancel_pending', 'Objetivo anterior sin terminación confirmada')
        error = self._controller_error(profile)
        if error:
            return failure(503, *error)
        if profile.action not in self.clients:
            self.clients[profile.action] = ActionClient(
                self.node, FollowJointTrajectory, profile.action,
            )
        client = self.clients[profile.action]
        if not client.wait_for_server(timeout_sec=3.0):
            return failure(503, 'controller_unavailable', f'Acción ausente: {profile.action}')
        if profile.joint_states_topic not in self.observers:
            self.observers[profile.joint_states_topic] = JointObserver(
                self.node, profile.joint_states_topic,
            )
        observer = self.observers[profile.joint_states_topic]
        initial = observer.wait(profile.joints, time.monotonic(), profile.state_timeout_sec)
        if initial is None:
            return failure(503, 'state_unavailable',
                           'No llegan estados recientes de todos los joints')
        evidence = observer.begin(initial)
        try:
            code, result = self._run_action(skill, client)
            if not result.get('ok'):
                return code, result
            positions = observer.wait(profile.joints, time.monotonic(), profile.state_timeout_sec)
            if positions is None:
                return failure(502, 'verification_failed',
                               'Sin estados recientes tras el resultado',
                               accepted=True, action_status=result['action_status'],
                               error_code=result['error_code'])
            with observer.condition:
                verified, measurements = evidence.evaluate(positions, profile)
            result['verification'] = measurements
            result['ok'] = verified
            if not verified:
                result.update(status='verification_failed',
                              detail='Movimiento o posición final fuera del contrato configurado')
                return 502, result
            result['detail'] = 'Acción completada y movimiento verificado mediante joint_states'
            return code, result
        finally:
            observer.end()

    def _run_action(self, skill, client):
        profile = skill.profile
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(profile.joints)
        for wp in profile.waypoints:
            point = JointTrajectoryPoint()
            point.positions = list(wp.positions)
            # Velocidad nula en los puntos: arranque, inversiones y fin suaves.
            point.velocities = [0.0] * len(profile.joints)
            point.time_from_start = to_duration(wp.time)
            goal.trajectory.points.append(point)
        for joint in profile.joints:
            tolerance = JointTolerance()
            tolerance.name = joint
            tolerance.position = profile.goal_tolerance_rad
            goal.goal_tolerance.append(tolerance)
        goal.goal_time_tolerance = to_duration(profile.goal_time_tolerance_sec)

        finished = threading.Event()
        cancel_requested = threading.Event()
        reply = {}
        shared = {'handle': None}
        mutex = threading.Lock()

        def done(code, status, detail, terminal=True, **extra):
            with mutex:
                if finished.is_set():
                    return
                reply.update({'ok': code == 200, 'status': status, 'detail': detail,
                              'skill': skill.name, 'accepted': shared['handle'] is not None,
                              **extra})
                reply['_http_status'] = code
                if terminal:
                    self._cancel_pending.clear()
                else:
                    self._cancel_pending.set()
                finished.set()

        def got_result(future):
            try:
                wrapped = future.result()
                action_status = int(wrapped.status)
                result = wrapped.result
                success = (action_status == GoalStatus.STATUS_SUCCEEDED
                           and result.error_code == FollowJointTrajectory.Result.SUCCESSFUL)
                state = ('succeeded' if success else
                         'canceled' if action_status == GoalStatus.STATUS_CANCELED else 'aborted')
                done(200 if success else 502, state,
                     result.error_string or (
                         'Trayectoria completada' if success else 'Acción fallida'),
                     action_status=action_status, error_code=int(result.error_code))
            except Exception as exc:
                done(502, 'ros_error', str(exc), terminal=False)

        def got_acceptance(future):
            try:
                handle = future.result()
                if handle is None or not handle.accepted:
                    done(422, 'rejected', 'El controlador rechazó la trayectoria')
                    return
                with mutex:
                    shared['handle'] = handle
                handle.get_result_async().add_done_callback(got_result)
                if cancel_requested.is_set():
                    handle.cancel_goal_async()
            except Exception as exc:
                done(502, 'ros_error', str(exc), terminal=False)

        try:
            client.send_goal_async(goal).add_done_callback(got_acceptance)
        except Exception as exc:
            # Ante un fallo de transporte no sabemos si se entregó el objetivo.
            done(502, 'ros_error', str(exc), terminal=False)

        if not finished.wait(timeout=profile.execution_timeout_sec):
            timed_out = False
            with mutex:
                if not finished.is_set():
                    self._cancel_pending.set()
                    cancel_requested.set()
                    handle = shared['handle']
                    timed_out = True
            if timed_out:
                if handle is not None:
                    handle.cancel_goal_async()
                return 504, {
                    'ok': False, 'skill': skill.name, 'status': 'timeout',
                    'accepted': handle is not None,
                    'detail': 'Tiempo real agotado. Cancelación solicitada, no confirmada.',
                }
        with mutex:
            result = dict(reply)
        return result.pop('_http_status'), result
