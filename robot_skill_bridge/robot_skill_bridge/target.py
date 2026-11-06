"""Localización métrica de una región observada; no interpreta descripciones."""

from typing import Protocol

import numpy as np

from .skill_result import SkillResult


class TargetProvider(Protocol):
    def locate(self, observation_id: str, bounds: tuple[float, float, float, float]) -> dict:
        """Devuelve una posición medida en la base para esa observación concreta."""
        ...


def parse_target(name, data):
    provider = data.get('provider')
    if (not isinstance(provider, str) or not provider or data.get('kind') != 'perception'):
        raise ValueError(f'{name}: requiere provider y kind=perception')
    return provider


def rigid_transform(points, translation, quaternion):
    """Transforma coordenadas reales, no solo la etiqueta del frame."""
    x, y, z, w = quaternion
    if not np.isfinite([x, y, z, w, *translation]).all():
        raise ValueError('TF no finita')
    if abs(x*x+y*y+z*z+w*w-1.) > .01:
        raise ValueError('Cuaternión TF no unitario')
    rotation = np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ])
    return np.asarray(points) @ rotation.T + translation


def target_from_region(optical_points, base_points, camera, bounds, min_points=12,
                       height_range=(-np.inf, np.inf)):
    """Proyecta profundidad en RGB y estima un grupo compacto dentro de la caja."""
    bounds = np.asarray(bounds, dtype=float)
    if (bounds.shape != (4,) or not np.isfinite(bounds).all()
            or (bounds < 0).any() or (bounds > 1).any()
            or bounds[0] >= bounds[2] or bounds[1] >= bounds[3]):
        raise ValueError('Caja normalizada inválida')
    optical, base = np.asarray(optical_points), np.asarray(base_points)
    if optical.shape != base.shape or optical.ndim != 2 or optical.shape[1] != 3:
        raise ValueError('Puntos inválidos')
    fx, fy, cx, cy, width, height = camera
    if not np.isfinite(camera).all() or min(fx, fy, width, height) <= 0:
        raise ValueError('Calibración inválida')
    finite = np.isfinite(optical).all(axis=1) & np.isfinite(base).all(axis=1)
    # La caja central reduce mezcla con fondo y suelo sin depender de color o forma.
    left, top, right, bottom = bounds
    inset_x, inset_y = .2*(right-left), .2*(bottom-top)
    safe_z = np.where(optical[:, 2] > 0, optical[:, 2], np.nan)
    u, v = fx*optical[:, 0]/safe_z+cx, fy*optical[:, 1]/safe_z+cy
    mask = (finite & (optical[:, 2] > .1) & (optical[:, 2] < 5.)
            & (base[:, 2] >= height_range[0]) & (base[:, 2] <= height_range[1])
            & (u >= (left+inset_x)*width) & (u <= (right-inset_x)*width)
            & (v >= (top+inset_y)*height) & (v <= (bottom-inset_y)*height))
    selected = np.flatnonzero(mask)
    if len(selected) < min_points:
        raise ValueError('No hay profundidad suficiente dentro del objetivo')
    depths = optical[selected, 2]
    # Se exige soporte de profundidad compacto; no se inventa un rango a partir de RGB.
    near = float(np.percentile(depths, 25))
    cluster = selected[np.abs(depths-near) <= .15]
    if len(cluster) < min_points:
        raise ValueError('Profundidad ambigua o dispersa')
    point = np.median(base[cluster], axis=0)
    distance = float(np.hypot(point[0], point[1]))
    if not .25 <= distance <= 4.5:
        raise ValueError('Objetivo fuera del intervalo de profundidad utilizable')
    return {'x_m': float(point[0]), 'y_m': float(point[1]), 'z_m': float(point[2]),
            'distance_m': distance, 'bearing_deg': float(np.degrees(np.arctan2(
                point[1], point[0]))), 'depth_samples': len(cluster)}


class TargetBackend:
    def __init__(self, providers):
        self.providers = providers

    def execute(self, skill, parameters=None, cancel=None):
        try:
            provider = self.providers[skill.profile]
            target = provider.locate(parameters['observation_id'], tuple(
                parameters[key] for key in ('left', 'top', 'right', 'bottom')))
            return SkillResult(True, 'located', skill.name, data={'target': target})
        except (RuntimeError, ValueError, KeyError, AttributeError) as exc:
            return SkillResult(False, 'target_unavailable', skill.name, str(exc))
