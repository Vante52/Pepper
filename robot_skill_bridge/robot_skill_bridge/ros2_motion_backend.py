"""Movimiento por odometría y cmd_vel; no consulta Gazebo ni proveedores de IA."""

import math
import threading
import time

from controller_manager_msgs.srv import ListControllers
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data

from .motion import feedback_velocity, relative_goal, wrap_angle
from .skill_result import SkillResult


class ROS2MotionBackend:
    def __init__(self, node):
        self.node = node
        self.condition = threading.Condition()
        self.states, self.subscriptions, self.managers = {}, {}, {}
        self.closed = threading.Event()

    def _receive(self, topic, message):
        p, q = message.pose.pose.position, message.pose.pose.orientation
        values = (p.x, p.y, q.x, q.y, q.z, q.w,
                  message.twist.twist.linear.x, message.twist.twist.linear.y,
                  message.twist.twist.angular.z)
        if (not all(math.isfinite(v) for v in values)
                or abs(sum(v*v for v in values[2:6]) - 1.) > .01):
            return
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        stamp = message.header.stamp.sec + message.header.stamp.nanosec*1e-9
        with self.condition:
            self.states[topic] = ((p.x, p.y, yaw), stamp, time.monotonic(),
                                  (message.header.frame_id, message.child_frame_id), values[6:])
            self.condition.notify_all()

    def _controller_active(self, profile):
        service = profile.controller_manager.rstrip('/') + '/list_controllers'
        if service not in self.managers:
            self.managers[service] = self.node.create_client(ListControllers, service)
        client = self.managers[service]
        if not client.wait_for_service(timeout_sec=2.):
            return False
        future = client.call_async(ListControllers.Request())
        finished = threading.Event()
        future.add_done_callback(lambda _: finished.set())
        return (finished.wait(2.) and any(
            c.name == profile.controller and c.state == 'active'
            for c in future.result().controller))

    def execute(self, skill, parameters=None, cancel=None):
        profile = skill.profile
        publisher = None
        try:
            if not self._controller_active(profile):
                return SkillResult(False, 'controller_inactive', skill.name,
                                   f'{profile.controller} no está activo o no responde')
            if profile.odom_topic not in self.subscriptions:
                self.subscriptions[profile.odom_topic] = self.node.create_subscription(
                    Odometry, profile.odom_topic,
                    lambda msg: self._receive(profile.odom_topic, msg), qos_profile_sensor_data)
            with self.condition:
                available = self.condition.wait_for(
                    lambda: (profile.odom_topic in self.states and time.monotonic()
                             - self.states[profile.odom_topic][2] < profile.state_timeout_sec),
                    timeout=profile.state_timeout_sec)
            if not available:
                return SkillResult(False, 'state_unavailable', skill.name,
                                   f'Sin odometría reciente en {profile.odom_topic}')
            state_stamp = self.states[profile.odom_topic][1]
            if abs(self.node.get_clock().now().nanoseconds*1e-9 - state_stamp) > 3600.:
                return SkillResult(False, 'clock_mismatch', skill.name,
                                   'El reloj del bridge y la odometría usan tiempos distintos. '
                                   'En simulación inicie el bridge con use_sim_time:=true; '
                                   'en hardware, compruebe la sincronización de relojes.')
            if self.node.count_publishers(profile.cmd_vel_topic):
                return SkillResult(False, 'busy', skill.name,
                                   'Otro emisor usa cmd_vel; cierre el teleop o utilice un mux')
            # Solo existe durante la habilidad: el guard no ve emisores inactivos permanentes.
            publisher = self.node.create_publisher(Twist, profile.cmd_vel_topic, 1)
            deadline = time.monotonic() + 2.
            while publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
                time.sleep(.05)
            if publisher.get_subscription_count() == 0:
                return SkillResult(False, 'controller_unavailable', skill.name,
                                   'No hay receptor de cmd_vel (¿está iniciado el guard?)')
            return self._run(skill, parameters or {}, publisher, cancel)
        except Exception as exc:
            return SkillResult(False, 'ros_error', skill.name, str(exc))
        finally:
            if publisher is not None:
                for _ in range(3):
                    publisher.publish(Twist())
                    time.sleep(.03)
                self.node.destroy_publisher(publisher)

    def _run(self, skill, parameters, publisher, cancel):
        profile = skill.profile
        initial, initial_stamp, _, frames, _ = self.states[profile.odom_topic]
        if not all(frames):
            return SkillResult(False, 'state_unavailable', skill.name,
                               'Frames de odometría vacíos')
        rotation = math.radians(parameters.get('angle_deg', 0.))
        goal = relative_goal(initial, parameters.get('distance_m', 0.),
                             parameters.get('direction_deg', 0.))
        started = last_clock_progress = last_motion = time.monotonic()
        last_stamp, previous_yaw, accumulated_yaw = initial_stamp, initial[2], 0.
        previous_pose = initial
        settled_since = None
        status, detail = 'timeout', 'Se agotó el tiempo real de ejecución'
        error = math.hypot(initial[0]-goal[0], initial[1]-goal[1])
        while time.monotonic() - started < profile.execution_timeout_sec:
            now = time.monotonic()
            if self.closed.is_set() or (cancel is not None and cancel.is_set()):
                status, detail = 'canceled', 'Movimiento cancelado'
                break
            pose, stamp, received, current_frames, twist = self.states[profile.odom_topic]
            clock_age = self.node.get_clock().now().nanoseconds*1e-9 - stamp
            if abs(clock_age) > 3600.:
                status, detail = 'clock_mismatch', 'El reloj del bridge no coincide con odometría'
                break
            if (current_frames != frames or not -.1 <= clock_age <= profile.state_timeout_sec
                    or now - received > profile.state_timeout_sec or stamp < last_stamp):
                status = 'state_unavailable'
                detail = 'Odometría caducada, reiniciada o frame cambiado'
                break
            if self.node.count_publishers(profile.cmd_vel_topic) > 1:
                status, detail = 'busy', 'Otro emisor comenzó a publicar en cmd_vel'
                break
            if stamp > last_stamp:
                accumulated_yaw += wrap_angle(pose[2] - previous_yaw)
                last_clock_progress, last_stamp, previous_yaw = now, stamp, pose[2]
            if now - last_clock_progress > profile.state_timeout_sec:
                status, detail = 'clock_stopped', 'El reloj no avanza; pulse Play antes de mover'
                break
            yaw_error = (rotation - accumulated_yaw if profile.mode == 'rotate'
                         else wrap_angle(initial[2] - pose[2]))
            velocity, reached = feedback_velocity(pose, goal, yaw_error, profile)
            error = math.hypot(pose[0] - goal[0], pose[1] - goal[1])
            if reached:
                publisher.publish(Twist())
                stopped = math.hypot(twist[0], twist[1]) < .02 and abs(twist[2]) < .03
                if stopped:
                    settled_since = stamp if settled_since is None else settled_since
                    if stamp - settled_since >= .2:
                        verification = {
                            'source': profile.odom_topic, 'position_error_m': error,
                            'yaw_error_rad': yaw_error,
                            'travelled_m': math.hypot(pose[0]-initial[0], pose[1]-initial[1]),
                            'turned_rad': accumulated_yaw,
                        }
                        return SkillResult(True, 'succeeded', skill.name,
                                           'Objetivo alcanzado y parada verificada por odometría',
                                           {'verification': verification})
                else:
                    settled_since = None
            else:
                settled_since = None
                message = Twist()
                message.linear.x, message.linear.y, message.angular.z = velocity
                publisher.publish(message)
            motion = math.hypot(pose[0]-previous_pose[0], pose[1]-previous_pose[1])
            if motion > .002 or abs(wrap_angle(pose[2]-previous_pose[2])) > .005:
                last_motion, previous_pose = now, pose
            if not reached and now-last_motion > profile.stall_timeout_sec:
                status, detail = 'stalled', 'No se detecta avance; se detuvo el comando'
                break
            time.sleep(.05)
        return SkillResult(False, status, skill.name, detail,
                           {'verification': {'source': profile.odom_topic,
                                             'position_error_m': error}})

    def close(self):
        self.closed.set()
