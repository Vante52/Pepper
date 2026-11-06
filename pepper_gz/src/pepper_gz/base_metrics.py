"""Métricas SE(2) con timestamps de simulación y verdad terreno independiente."""

import math

import numpy as np


def series(rows, angle_column=None):
    """Ordena muestras, conserva el último duplicado y desenvuelve orientación."""
    data = np.asarray(rows, dtype=float)
    if data.ndim != 2 or len(data) < 2 or not np.isfinite(data).all():
        raise ValueError('Se requieren al menos dos muestras finitas por fuente')
    data = data[np.argsort(data[:, 0], kind='stable')].copy()
    data = data[np.r_[np.diff(data[:, 0]) > 1e-9, True]]
    if len(data) < 2:
        raise ValueError('Las muestras deben cubrir más de un instante')
    if angle_column is not None:
        data[:, angle_column] = np.unwrap(data[:, angle_column])
    return data


def interpolate(data, times):
    """Interpola dentro del intervalo cubierto por una fuente ordenada."""
    return np.column_stack([np.interp(times, data[:, 0], data[:, i])
                            for i in range(1, data.shape[1])])


def relative_pose(start, poses):
    """Expresa poses XY/yaw respecto al origen y orientación inicial de su fuente."""
    poses = np.asarray(poses, dtype=float)
    delta = poses[:, :2]-start[:2]
    c, s = math.cos(start[2]), math.sin(start[2])
    xy = delta @ np.array([[c, -s], [s, c]])
    return np.column_stack([xy, poses[:, 2]-start[2]])


def command_reference(commands, start, end):
    """Integra Twist aplicado con retención de orden cero y exponencial SE(2)."""
    data = series(commands)
    times = np.r_[start, data[(data[:, 0] > start) & (data[:, 0] < end), 0], end]
    pose, path = np.zeros(3), 0.
    for a, b in zip(times, times[1:]):
        index = max(0, np.searchsorted(data[:, 0], a, side='right')-1)
        x, y, yaw = data[index, 1:4]
        dt, angle = b-a, yaw*(b-a)
        if abs(yaw) < 1e-12:
            dx, dy = x*dt, y*dt
        else:
            dx = (x*math.sin(angle)+y*(math.cos(angle)-1.))/yaw
            dy = (x*(1.-math.cos(angle))+y*math.sin(angle))/yaw
        c, s = math.cos(pose[2]), math.sin(pose[2])
        pose += [c*dx-s*dy, s*dx+c*dy, angle]
        path += math.hypot(x, y)*dt
    return pose, float(path)


def trajectory_metrics(truth_rows, odom_rows, command_rows, start, stop, end, hz=50.):
    """Compara GZ [t,x,y,z,r,p,y] con odom [t,x,y,z,y,vx,vy,wz]."""
    truth, odom = series(truth_rows, 6), series(odom_rows, 4)
    start = max(start, truth[0, 0], odom[0, 0])
    end = min(end, truth[-1, 0], odom[-1, 0])
    if end-start < .2:
        raise ValueError('Intervalo común insuficiente para medir movimiento')
    grid = np.arange(start, end, 1./hz)
    grid = np.r_[grid, end]
    gt, od = interpolate(truth, grid), interpolate(odom, grid)
    gt_pose, od_pose = gt[:, [0, 1, 5]], od[:, [0, 1, 3]]
    gt_relative = relative_pose(gt_pose[0], gt_pose)
    od_relative = relative_pose(od_pose[0], od_pose)
    reference, reference_path = command_reference(command_rows, start, end)
    position_errors = od_relative[:, :2]-gt_relative[:, :2]
    yaw_errors = od_relative[:, 2]-gt_relative[:, 2]
    # Diferencia central de 100 ms evita derivar directamente el ruido de cada paso.
    h = .05
    velocity_times = grid[(grid >= start+h) & (grid <= end-h)]
    before, after = interpolate(truth, velocity_times-h), interpolate(truth, velocity_times+h)
    middle = interpolate(truth, velocity_times)
    # Compara ambas velocidades con la misma ventana de 100 ms.
    offsets = np.linspace(-h, h, 101)
    sample_times = velocity_times[:, None]+offsets[None, :]
    od_samples = interpolate(odom, sample_times.ravel())[:, 4:7].reshape(-1, 101, 3)
    od_velocity = (od_samples[:, 1:-1].sum(axis=1)
                   + .5*(od_samples[:, 0]+od_samples[:, -1]))/100.
    world_v = (after[:, :2]-before[:, :2])/(2.*h)
    c, s = np.cos(middle[:, 5]), np.sin(middle[:, 5])
    gt_velocity = np.column_stack([c*world_v[:, 0]+s*world_v[:, 1],
                                   -s*world_v[:, 0]+c*world_v[:, 1],
                                   (after[:, 5]-before[:, 5])/(2.*h)])
    velocity_error = od_velocity-gt_velocity
    # Los ajustes se basan en segmentos en marcha, sin arranque ni parada.
    steady = (velocity_times > start+.35) & (velocity_times < stop-.15)
    lin = steady & (np.linalg.norm(od_velocity[:, :2], axis=1) > .02)
    ang = steady & (np.abs(od_velocity[:, 2]) > .05)
    fit = {
        'linear_dot': float(np.sum(od_velocity[lin, :2]*gt_velocity[lin, :2])),
        'linear_sq': float(np.sum(od_velocity[lin, :2]**2)),
        'yaw_dot': float(np.sum(od_velocity[ang, 2]*gt_velocity[ang, 2])),
        'yaw_sq': float(np.sum(od_velocity[ang, 2]**2)),
    }
    error = float(np.linalg.norm(position_errors[-1]))
    summary = {
        'start': float(start), 'stop': float(stop), 'end': float(end),
        'reference_pose': reference.tolist(), 'reference_path_m': reference_path,
        'gazebo_pose_delta': gt_relative[-1].tolist(),
        'odom_pose_delta': od_relative[-1].tolist(),
        'endpoint_error_m': error,
        'endpoint_error_fraction': error/reference_path if reference_path > 1e-6 else None,
        'orientation_error_rad': float(yaw_errors[-1]),
        'position_rmse_m': float(np.sqrt(np.mean(np.sum(position_errors**2, axis=1)))),
        'orientation_rmse_rad': float(np.sqrt(np.mean(yaw_errors**2))),
        'linear_velocity_rmse_mps': float(np.sqrt(np.mean(np.sum(
            velocity_error[:, :2]**2, axis=1)))),
        'angular_velocity_rmse_rps': float(np.sqrt(np.mean(velocity_error[:, 2]**2))),
        'tracking_endpoint_error_m': float(np.linalg.norm(gt_relative[-1, :2]-reference[:2])),
        'tracking_yaw_error_rad': float(gt_relative[-1, 2]-reference[2]),
        'max_base_tilt_rad': float(np.max(np.hypot(gt[:, 3], gt[:, 4]))),
        'base_height_range_m': float(np.ptp(gt[:, 2])),
        'ground_truth_max_gap_s': float(np.max(np.diff(truth[
            (truth[:, 0] >= start) & (truth[:, 0] <= end), 0]))),
        'odom_max_gap_s': float(np.max(np.diff(odom[
            (odom[:, 0] >= start) & (odom[:, 0] <= end), 0]))),
        'fit_sums': fit,
    }
    aligned = np.column_stack([grid, gt_relative, od_relative])
    velocities = np.column_stack([velocity_times, gt_velocity, od_velocity])
    return summary, aligned, velocities


def fitted_parameters(results, wheel_radius, robot_radius):
    """Estima radios efectivos; el llamador decide si tienen justificación física."""
    sums = {key: sum(r['fit_sums'][key] for r in results)
            for key in ('linear_dot', 'linear_sq', 'yaw_dot', 'yaw_sq')}
    linear = sums['linear_dot']/sums['linear_sq'] if sums['linear_sq'] > 1e-10 else None
    angular = sums['yaw_dot']/sums['yaw_sq'] if sums['yaw_sq'] > 1e-10 else None
    return {
        'linear_gain_truth_over_odom': linear, 'angular_gain_truth_over_odom': angular,
        'suggested_effective_wheel_radius': wheel_radius*linear if linear is not None else None,
        'suggested_effective_robot_radius': (robot_radius*linear/angular
                                             if linear is not None and angular else None),
    }
