# Validación de base

Estado: **failed**.

Pose real: GZ Transport; odometría: controlador. Tiempo de simulación.
Velocidades comparadas con una ventana común de 100 ms.

| Ensayo | Repetición | Error posición (cm) | Error yaw (°) | RMSE velocidad (m/s) | RMSE giro (rad/s) | Criterio |
| --- | --- | ---: | ---: | ---: | ---: | --- |
| forward | 1 | 3.910 | 0.398 | 0.0660 | 0.0387 | endpoint_error_m, linear_velocity_rmse_mps |
| backward | 1 | 3.651 | -4.303 | 0.0891 | 0.2007 | endpoint_error_m, tracking_endpoint_error_m, orientation_error_rad, tracking_yaw_error_rad, linear_velocity_rmse_mps, angular_velocity_rmse_rps |
| left | 1 | 5.651 | -8.573 | 0.0511 | 0.1407 | endpoint_error_m, tracking_endpoint_error_m, orientation_error_rad, tracking_yaw_error_rad, linear_velocity_rmse_mps, angular_velocity_rmse_rps |
| right | 1 | 4.606 | 6.416 | 0.0383 | 0.1283 | endpoint_error_m, tracking_endpoint_error_m, orientation_error_rad, tracking_yaw_error_rad, linear_velocity_rmse_mps, angular_velocity_rmse_rps |
| turn_left | 1 | 1.172 | 0.991 | 0.0087 | 0.0181 | OK |
| turn_right | 1 | 1.126 | -0.333 | 0.0080 | 0.0093 | OK |
| arc_left | 1 | 2.341 | 1.305 | 0.0235 | 0.0337 | OK |
| arc_right | 1 | 3.275 | -1.626 | 0.0247 | 0.0355 | endpoint_error_m, tracking_endpoint_error_m |

Los CSV comprimidos conservan las fuentes originales y los timestamps.
summary.json incluye seguimiento de la orden, estabilidad y huecos de muestreo.
manifest.json contiene configuración, versiones y ajuste diagnóstico de radios.
Las sugerencias de ajuste no se aplican automáticamente.
