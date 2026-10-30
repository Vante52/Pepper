# pepper_gz

Simulación de los sensores de Pepper con ROS 2 Jazzy y Gazebo Harmonic.
El paquete incluye la descripción física de `pepper_description` y añade
sensores mediante extensiones de Gazebo, sin crear links, joints ni geometrías.

## Iniciar

Desde la raíz de `pepper_ws`:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select pepper_description pepper_gz
source install/setup.bash
ros2 launch pepper_gz sensors.launch.py
```

Para ejecutar sin interfaz gráfica, con renderizado de cámaras mediante EGL:

```bash
ros2 launch pepper_gz sensors.launch.py gui:=false
```

El lanzamiento inicia `minimal_world` con Bullet Featherstone y los sistemas
`Sensors` e `Imu`, inserta `pepper`, publica los estados articulares de Gazebo
para `robot_state_publisher` y abre un único puente Gazebo → ROS. Todos los
nodos ROS de la simulación utilizan `use_sim_time=true`.

El comprobador de arranque espera los tópicos y mediciones, revisa sus tipos,
resoluciones, cabeceras y frames TF y termina con un mensaje de confirmación.
Si la comprobación falla, se cierra ese lanzamiento. No se inicia una GUI de
estados articulares.

Las articulaciones tienen dinámica libre. Este lanzamiento no incorpora
control postural: Pepper puede flexionarse y caer. La inserción utiliza
`spawn_z:=0.607`, que coloca el origen de `base_link` cinco milímetros por
encima de su altura nominal inicial sobre el suelo.

## Sensores y parámetros

La configuración está en [src/config/sensors.yaml](src/config/sensors.yaml).
**Todos sus valores son provisionales de simulación; no son especificaciones
verificadas del hardware.** El perfil predeterminado usa ruido de amplitud cero.

| Sensor | Nombre en Gazebo | Frame de montaje | Frecuencia provisional |
| --- | --- | --- | --- |
| RGB superior | `camera_top_rgb` | `CameraTop_frame` | 15 Hz |
| RGB inferior | `camera_bottom_rgb` | `CameraBottom_frame` | 15 Hz |
| Profundidad | `camera_depth` | `CameraDepth_frame` | 15 Hz |
| IMU de base | `imu_base` | `ImuBaseAccelerometer_frame` | 100 Hz |
| IMU de torso | `imu_torso` | `ImuTorsoAccelerometer_frame` | 100 Hz |

Las cámaras RGB usan 640 × 480 píxeles y formato `R8G8B8`; en ROS publican
`rgb8`. La cámara de profundidad usa 320 × 240 y publica `32FC1`, con distancias
en metros. El campo de visión horizontal provisional es 1.0471975512 rad
(60°) para las tres cámaras. Los planos de recorte provisionales son
0.05–20 m para RGB y 0.1–5 m para profundidad.

El modelo de cámara es pinhole ideal. `CameraInfo` se genera a partir de esa
configuración y no contiene una calibración del Pepper real.

Los parámetros `noise_mean` y `noise_stddev` configuran el ruido gaussiano de
imagen de Gazebo. Para las IMU, `acceleration_mean` y `acceleration_stddev`
están en m/s²; `angular_velocity_mean` y `angular_velocity_stddev`, en rad/s.
Cada eje recibe ruido independiente con los mismos parámetros. Los sesgos y
el sesgo dinámico se fijan explícitamente a cero en el Xacro. Las amplitudes
por defecto son cero: es un perfil ideal para comprobar la integración.
La orientación de la IMU es la orientación simulada respecto al mundo, sin
un modelo de error de estimación ni un algoritmo de fusión del hardware.

Se puede editar el YAML o pasar una copia:

```bash
ros2 launch pepper_gz sensors.launch.py sensors_config:=/ruta/sensors.yaml
```

Mantener `profile: provisional_simulation` mientras los valores no estén
contrastados con el hardware. Si se cambian tópicos o frames, actualizar
también [src/config/sensors_bridge.yaml](src/config/sensors_bridge.yaml)
y mantener la correspondencia con los frames físicos existentes.

## Tópicos ROS

Todos los puentes del YAML son unidireccionales `GZ_TO_ROS`.

| Tópico ROS | Tipo | `header.frame_id` |
| --- | --- | --- |
| `/pepper/camera/top/image_raw` | `sensor_msgs/msg/Image` | `CameraTop_optical_frame` |
| `/pepper/camera/top/camera_info` | `sensor_msgs/msg/CameraInfo` | `CameraTop_optical_frame` |
| `/pepper/camera/bottom/image_raw` | `sensor_msgs/msg/Image` | `CameraBottom_optical_frame` |
| `/pepper/camera/bottom/camera_info` | `sensor_msgs/msg/CameraInfo` | `CameraBottom_optical_frame` |
| `/pepper/camera/depth/image_raw` | `sensor_msgs/msg/Image` | `CameraDepth_optical_frame` |
| `/pepper/camera/depth/camera_info` | `sensor_msgs/msg/CameraInfo` | `CameraDepth_optical_frame` |
| `/pepper/camera/depth/points` | `sensor_msgs/msg/PointCloud2` | `CameraDepth_optical_frame` |
| `/pepper/imu/base` | `sensor_msgs/msg/Imu` | `ImuBaseAccelerometer_frame` |
| `/pepper/imu/torso` | `sensor_msgs/msg/Imu` | `ImuTorsoAccelerometer_frame` |
| `/joint_states` | `sensor_msgs/msg/JointState` | Sin frame espacial |
| `/clock` | `rosgraph_msgs/msg/Clock` | No tiene cabecera |

Las imágenes, información de cámara, IMU y estados articulares usan QoS
`SENSOR_DATA`; `/clock` usa `CLOCK`. El sello de tiempo se conserva desde
Gazebo. Los consumidores deben utilizar tiempo simulado.

No se añaden láseres ni puentes `LaserScan`: no hay un láser simulado
implementado en este lanzamiento.

### Coordenadas de la nube de profundidad

En Gazebo Sensors 8.2.2, la nube nativa usa X hacia delante, Y hacia la izquierda
y Z hacia arriba. Cambiar solamente su `frame_id` a un frame óptico daría
coordenadas incorrectas.

El puente recibe el tópico Gazebo `/pepper/camera/depth/image_raw/points` y lo
expone internamente como `/pepper/camera/depth/_points_sensor`, con frame
`CameraDepth_frame`. El nodo `depth_points_optical` aplica la TF existente y
publica `/pepper/camera/depth/points` con X hacia la derecha, Y hacia abajo y
Z hacia delante. Conserva sello de tiempo, campos, padding y organización 2D.

### IMU del torso

Se crea **una sola IMU virtual** en `ImuTorsoAccelerometer_frame`.
`linear_acceleration` corresponde a ese origen. El frame
`ImuTorsoGyrometer_frame` está en otro punto del mismo torso rígido y tiene
la misma orientación. La velocidad angular es igual en ambos puntos y se
expresa en los mismos ejes, por lo que `angular_velocity` representa también
la lectura del giróscopo en esos ejes.

El mensaje completo usa `ImuTorsoAccelerometer_frame`: un `sensor_msgs/Imu`
solo admite una cabecera. No se asigna al giróscopo la aceleración lineal
medida en otro punto ni se crea una segunda IMU redundante.

## Reutilizar componentes existentes

El lanzamiento comprueba la partición de Gazebo y el dominio ROS antes de
arrancar. Rechaza un segundo servidor en la misma `GZ_PARTITION` y publicadores
ROS que entren en conflicto. La inserción desactiva el renombrado automático
para impedir un segundo Pepper con otro nombre.

| Argumento | Por defecto | Uso |
| --- | --- | --- |
| `gui` | `true` | Abrir la interfaz de Gazebo. |
| `start_gazebo` | `true` | Iniciar `minimal_world`. |
| `spawn_robot` | `true` | Insertar el modelo `pepper`. |
| `publish_tf` | `true` | Iniciar `robot_state_publisher`. |
| `start_bridge` | `true` | Iniciar el puente y el conversor de nube. |
| `check_sensors` | `true` | Comprobar mediciones al arrancar. |
| `spawn_z` | `0.607` | Altura de inserción de `base_link`, en metros. |
| `sensors_config` | YAML del paquete | Parámetros de los sensores. |
| `bridge_config` | YAML del paquete | Correspondencia de tópicos y tipos. |

Para reutilizar un `minimal_world` que ya tenga los sistemas de sensores:

```bash
ros2 launch pepper_gz sensors.launch.py start_gazebo:=false
```

Desactivar también `spawn_robot`, `publish_tf` o `start_bridge` si esos
componentes ya están en ejecución. Reutilizar un puente requiere que también
esté funcionando la conversión de la nube y que sus tópicos coincidan con el
contrato configurado.

Para una instancia aislada de otra simulación:

```bash
export GZ_PARTITION=pepper_sensors
export ROS_DOMAIN_ID=182
ros2 launch pepper_gz sensors.launch.py gui:=false
```

Usar esas mismas variables en las terminales desde las que se inspeccionen
los tópicos. Elegir otro dominio si 182 ya está ocupado.

## Comprobar los sensores

La comprobación de arranque también puede ejecutarse por separado:

```bash
ros2 run pepper_gz check_sensors --ros-args -p use_sim_time:=true
```

Para inspeccionar Gazebo directamente:

```bash
gz topic -l
gz topic -i -t /pepper/camera/top/image_raw
gz topic -i -t /pepper/camera/depth/image_raw/points
gz topic -i -t /pepper/imu/base
```

Los nombres y tipos de estos tópicos se verifican en ejecución; no se infieren
del nombre del sensor. El reloj se toma de `/world/minimal_world/clock`, que
permanece disponible aunque Gazebo no publique su alias global `/clock`.

Las pruebas del modelo no necesitan abrir una interfaz gráfica:

```bash
colcon test --packages-select pepper_gz
colcon test-result --verbose
```

La reducción de uniones fijas mueve las cámaras al link `Head`, la IMU de
base a `base_link` y la del torso a `torso`. Las pruebas comprueban que los
cinco sensores conservan la pose de sus frames de montaje, sin duplicados.
Los frames originales siguen publicados por `robot_state_publisher`.

Harmonic puede avisar que `gz_frame_id` no pertenece al esquema SDF. Es una
extensión reconocida por Gazebo Sensors 8 para las IMU y se conserva en la
conversión. Las cámaras utilizan `camera/optical_frame_id`.

Referencias: [sistemas de sensores de Harmonic](https://gazebosim.org/docs/harmonic/sensors/),
[configuración del puente en Jazzy](https://github.com/gazebosim/ros_gz/blob/jazzy/ros_gz_bridge/src/bridge_config.cpp)
y [implementación de la cámara de profundidad](https://github.com/gazebosim/gz-sensors/blob/gz-sensors8/src/DepthCameraSensor.cc).

## Visualizar la nube en RViz2

Con la simulación activa y el mismo `ROS_DOMAIN_ID`, ejecutar:

```bash
rviz2 -d "$(ros2 pkg prefix pepper_gz)/share/pepper_gz/rviz/sensors.rviz" \
  --ros-args -p use_sim_time:=true
```

La vista incluye el robot y `/pepper/camera/depth/points`, con frame fijo
`base_link`, QoS Best Effort y puntos verdes. La cámara comienza a diez metros
para abarcar el rango de profundidad; ajustar el zoom según la escena.
