# Validación de base

Estado: **failed**.

Pose real: GZ Transport; odometría: controlador. Tiempo de simulación.
Velocidades comparadas con una ventana común de 100 ms.

| Ensayo | Repetición | Error posición (cm) | Error yaw (°) | RMSE velocidad (m/s) | RMSE giro (rad/s) | Criterio |
| --- | --- | ---: | ---: | ---: | ---: | --- |
| forward | 1 | 9.562 | 0.000 | 0.0342 | 0.0002 | endpoint_error_m, tracking_endpoint_error_m, linear_velocity_rmse_mps |
| backward | 1 | 8.926 | -0.000 | 0.0326 | 0.0002 | endpoint_error_m, tracking_endpoint_error_m, linear_velocity_rmse_mps |
| left | 1 | 2.649 | -1.895 | 0.0203 | 0.0523 | tracking_endpoint_error_m, tracking_yaw_error_rad, angular_velocity_rmse_rps |
| right | 1 | 1.437 | 3.738 | 0.0141 | 0.0390 | tracking_endpoint_error_m, orientation_error_rad, tracking_yaw_error_rad |
| turn_left | 1 | 5.172 | 19.395 | 0.0284 | 0.1221 | endpoint_error_m, tracking_endpoint_error_m, orientation_error_rad, tracking_yaw_error_rad, angular_velocity_rmse_rps |
| turn_right | 1 | 4.554 | -21.701 | 0.0317 | 0.1339 | endpoint_error_m, tracking_endpoint_error_m, orientation_error_rad, tracking_yaw_error_rad, linear_velocity_rmse_mps, angular_velocity_rmse_rps |
| arc_left | 1 | 13.864 | 12.834 | 0.0421 | 0.0833 | endpoint_error_m, tracking_endpoint_error_m, orientation_error_rad, tracking_yaw_error_rad, linear_velocity_rmse_mps, angular_velocity_rmse_rps |
| arc_right | 1 | 6.368 | -9.827 | 0.0265 | 0.0783 | endpoint_error_m, tracking_endpoint_error_m, orientation_error_rad, tracking_yaw_error_rad, angular_velocity_rmse_rps |

Los CSV comprimidos conservan las fuentes originales y los timestamps.
summary.json incluye seguimiento de la orden, estabilidad y huecos de muestreo.
manifest.json contiene configuración, versiones y ajuste diagnóstico de radios.
Las sugerencias de ajuste no se aplican automáticamente.
