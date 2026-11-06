# Validación de base

Estado: **failed**.

Pose real: GZ Transport; odometría: controlador. Tiempo de simulación.
Velocidades comparadas con una ventana común de 100 ms.

| Ensayo | Repetición | Error posición (cm) | Error yaw (°) | RMSE velocidad (m/s) | RMSE giro (rad/s) | Criterio |
| --- | --- | ---: | ---: | ---: | ---: | --- |
| forward | 1 | 0.523 | -0.048 | 0.0083 | 0.0003 | OK |
| backward | 1 | 0.096 | 0.008 | 0.0092 | 0.0027 | OK |
| left | 1 | 1.235 | -2.813 | 0.0102 | 0.0384 | OK |
| right | 1 | 1.068 | 2.509 | 0.0087 | 0.0354 | OK |
| turn_left | 1 | 0.912 | 0.416 | 0.0032 | 0.0031 | OK |
| turn_right | 1 | 0.922 | -0.186 | 0.0033 | 0.0038 | OK |
| arc_left | 1 | 1.058 | 0.004 | 0.0078 | 0.0109 | OK |
| arc_right | 1 | 1.105 | 0.468 | 0.0070 | 0.0069 | OK |
| forward | 2 | 0.381 | -0.651 | 0.0081 | 0.0065 | OK |
| backward | 2 | 0.860 | 0.053 | 0.0093 | 0.0089 | OK |
| left | 2 | 1.337 | -2.831 | 0.0134 | 0.0475 | OK |
| right | 2 | 1.273 | 3.253 | 0.0131 | 0.0465 | orientation_error_rad, tracking_yaw_error_rad |
| turn_left | 2 | 0.801 | 0.216 | 0.0055 | 0.0058 | OK |
| turn_right | 2 | 0.987 | -0.195 | 0.0058 | 0.0046 | OK |
| arc_left | 2 | 0.999 | -0.130 | 0.0058 | 0.0089 | OK |
| arc_right | 2 | 1.194 | 0.493 | 0.0064 | 0.0069 | OK |

Los CSV comprimidos conservan las fuentes originales y los timestamps.
summary.json incluye seguimiento de la orden, estabilidad y huecos de muestreo.
manifest.json contiene configuración, versiones y ajuste diagnóstico de radios.
Las sugerencias de ajuste no se aplican automáticamente.
