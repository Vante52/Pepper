"""Verificación de posiciones y movimiento sin dependencias de ROS."""


class MotionEvidence:
    """Compara mediciones, sin asumir que publicar objetivos implica movimiento."""

    def __init__(self, initial):
        self.initial = dict(initial)
        self.maximum_motion = {joint: 0.0 for joint in initial}
        self.samples = 0

    def observe(self, positions):
        for joint, start in self.initial.items():
            self.maximum_motion[joint] = max(
                self.maximum_motion[joint], abs(positions[joint] - start),
            )
        self.samples += 1

    def evaluate(self, positions, profile):
        errors = {joint: abs(positions[joint] - target)
                  for joint, target in zip(profile.joints, profile.waypoints[-1].positions)}
        verified = (self.samples > 0 and max(errors.values()) <= profile.goal_tolerance_rad
                    and max(self.maximum_motion.values()) >= profile.min_motion_rad)
        return verified, {
            'samples': self.samples,
            'initial_positions': self.initial,
            'final_positions': positions,
            'final_errors_rad': errors,
            'maximum_motion_rad': dict(self.maximum_motion),
            'goal_tolerance_rad': profile.goal_tolerance_rad,
            'min_motion_rad': profile.min_motion_rad,
        }
