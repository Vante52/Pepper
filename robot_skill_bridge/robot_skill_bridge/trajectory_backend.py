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
from .skill_result import SkillResult


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

    def execute(self, skill, parameters=None, cancel=None):
        if not self._busy.acquire(blocking=False):
            return SkillResult(False, 'busy', skill.name)
        try:
            return self._execute(skill, cancel)
        except Exception as exc:
            return SkillResult(False, 'ros_error', skill.name, str(exc))
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

    def _execute(self, skill, cancel=None):
        profile = skill.profile

        def failure(status, detail, **extra):
            return SkillResult(False, status, skill.name, detail, extra)

        if self._cancel_pending.is_set():
            return failure('cancel_pending', 'Objetivo anterior sin terminación confirmada')
        error = self._controller_error(profile)
        if error:
            return failure(*error)
        if profile.action not in self.clients:
            self.clients[profile.action] = ActionClient(
                self.node, FollowJointTrajectory, profile.action,
            )
        client = self.clients[profile.action]
        if not client.wait_for_server(timeout_sec=3.0):
            return failure('controller_unavailable', f'Acción ausente: {profile.action}')
        if profile.joint_states_topic not in self.observers:
            self.observers[profile.joint_states_topic] = JointObserver(
                self.node, profile.joint_states_topic,
            )
        observer = self.observers[profile.joint_states_topic]
        initial = observer.wait(profile.joints, time.monotonic(), profile.state_timeout_sec)
        if initial is None:
            return failure('state_unavailable',
                           'No llegan estados recientes de todos los joints')
        evidence = observer.begin(initial)
        try:
            outcome = self._run_action(skill, client, cancel)
            if not outcome.ok:
                return outcome
            result = outcome.to_dict()
            positions = observer.wait(profile.joints, time.monotonic(), profile.state_timeout_sec)
            if positions is None:
                return failure('verification_failed',
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
                return SkillResult.from_dict(result)
            result['detail'] = 'Acción completada y movimiento verificado mediante joint_states'
            return SkillResult.from_dict(result)
        finally:
            observer.end()

    def _run_action(self, skill, client, cancel=None):
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

        def done(status, detail, ok=False, terminal=True, **extra):
            with mutex:
                if finished.is_set():
                    return
                reply.update({'ok': ok, 'status': status, 'detail': detail,
                              'skill': skill.name, 'accepted': shared['handle'] is not None,
                              **extra})
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
                done(state,
                     result.error_string or (
                         'Trayectoria completada' if success else 'Acción fallida'),
                     ok=success, action_status=action_status, error_code=int(result.error_code))
            except Exception as exc:
                done('ros_error', str(exc), terminal=False)

        def got_acceptance(future):
            try:
                handle = future.result()
                if handle is None or not handle.accepted:
                    done('rejected', 'El controlador rechazó la trayectoria')
                    return
                with mutex:
                    shared['handle'] = handle
                handle.get_result_async().add_done_callback(got_result)
                if cancel_requested.is_set():
                    handle.cancel_goal_async()
            except Exception as exc:
                done('ros_error', str(exc), terminal=False)

        try:
            client.send_goal_async(goal).add_done_callback(got_acceptance)
        except Exception as exc:
            # Ante un fallo de transporte no sabemos si se entregó el objetivo.
            done('ros_error', str(exc), terminal=False)

        deadline = time.monotonic()+profile.execution_timeout_sec
        cancel_deadline = None
        while not finished.wait(.02):
            if cancel is not None and cancel.is_set() and not cancel_requested.is_set():
                cancel_requested.set()
                self._cancel_pending.set()
                cancel_deadline = time.monotonic()+3.
                with mutex:
                    handle = shared['handle']
                if handle is not None:
                    handle.cancel_goal_async()
            if time.monotonic() >= min(deadline, cancel_deadline or deadline):
                with mutex:
                    self._cancel_pending.set()
                    cancel_requested.set()
                    handle = shared['handle']
                if handle is not None:
                    handle.cancel_goal_async()
                state = 'cancel_pending' if cancel_deadline else 'timeout'
                return SkillResult(False, state, skill.name,
                                   'Cancelación solicitada, sin terminación confirmada.',
                                   {'accepted': handle is not None})
        with mutex:
            return SkillResult.from_dict(reply)
