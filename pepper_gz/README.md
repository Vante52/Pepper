# Pepper en Gazebo

ROS 2 Jazzy y Gazebo Harmonic con Bullet Featherstone. `pepper_description`
conserva geometría, cinemática y límites. Este paquete incorpora sensores,
contactos de simulación, `gz_ros2_control` y teleoperación.

## Compilar y arrancar

Desde la raíz del workspace:

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src/pepper_gz src/pepper_description --ignore-src -r -y
colcon build --packages-select pepper_description pepper_gz
source install/setup.bash
ros2 launch pepper_gz simulation.launch.py paused:=false
```

`paused:=false` ejecuta la física y activa los controladores seleccionados en
`config/controllers.yaml`. El plugin `libgz_ros2_control-system.so` crea el único
`controller_manager`, dentro de Gazebo. No se inicia `ros2_control_node`, RViz ni
`joint_state_publisher_gui`.

El arranque predeterminado, sin `paused:=false`, deja Gazebo pausado y configura
los controladores con `--inactive`. Después de pulsar **Play**, ejecuta el comando
que imprime el launch para activar el grupo seleccionado. Usa el servicio
`SwitchController` con timeout tipado; la lista se obtiene del YAML. Los spawners
esperan hasta 120 segundos al manager, y la inserción espera al servicio de
Gazebo con un plazo de 60 segundos reales configurable mediante `spawn_timeout`.

`world:=minimal_world.sdf` selecciona un mundo instalado; también se acepta otro
nombre instalado o una ruta absoluta. `spawn_z:=0.602` configura la altura
inicial. `gui:=false` inicia el servidor con renderizado sin ventana. El launch
prepara `GZ_SIM_RESOURCE_PATH` conservando las rutas existentes y las mallas
instaladas de `pepper_description`.

## Teclado

En otra terminal, con los mismos `setup.bash` y `ROS_DOMAIN_ID`:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run pepper_gz keyboard_teleop --ros-args -p use_sim_time:=true
```

El foco debe estar en esa terminal. Mantén pulsadas las teclas para repetir.
No ejecutes otro publicador de `/cmd_vel` al mismo tiempo.

| Tecla | Movimiento |
| --- | --- |
| `W` / flecha arriba | Base adelante |
| `S` / flecha abajo | Base atrás |
| `A` / flecha izquierda | Girar base a la izquierda |
| `D` / flecha derecha | Girar base a la derecha |
| `Q` / `E` | Desplazamiento lateral izquierda / derecha |
| Espacio | Detener la base inmediatamente |
| `K` / `Ñ` | Cabeza izquierda / derecha |
| `O` / `L` | Cabeza arriba / abajo |
| `Tab` | Seleccionar siguiente controlador articular |
| `[` / `]` | Seleccionar anterior / siguiente articulación |
| `-` / `+` (también `=`) | Disminuir / aumentar la articulación seleccionada |
| `Ctrl+C` | Detener la base y salir |

Se aceptan minúsculas y mayúsculas. `ñ` se decodifica como UTF-8. El selector
articular permite manejar postura y ambos brazos; sus grupos se descubren en
`controllers.yaml`, sin listas de joints en el launch.

La base publica a 30 Hz. Una pulsación aislada mantiene la orden hasta
`key_timeout` (0.5 s reales); después publica cero. El retraso inicial de
repetición del teclado del sistema puede provocar una pausa breve. La velocidad
predeterminada es 0.50 m/s y el giro 1.25 rad/s. El teclado escala la orden
completa cuando una rueda alcanzaría su límite URDF: el desplazamiento lateral
queda aproximadamente en 0.44 m/s. En una simulación más lenta que
el tiempo real, mantén la tecla varios segundos para observar el desplazamiento.

Las teclas articulares envían incrementos de 0.15 rad mediante trayectorias de
0.4 s. El objetivo está limitado por el URDF, por la velocidad articular y por
un adelanto máximo de 1.0 rad respecto al estado medido, acotado también por la
velocidad URDF y la duración. Requieren descripción,
estados recientes y controlador activo. Al dejar de pulsar, la articulación
mantiene el último objetivo. No pulses simultáneamente estas teclas mientras
otro cliente envía objetivos al mismo controlador.

## Configuración e interfaces

- `controllers.yaml`: tipos, recursos y política de activación.
- `initial_positions.yaml`: posiciones iniciales individuales; las cinco
  articulaciones de cabeza y postura empiezan en cero válido. Los brazos usan
  posiciones válidas para sus límites, que no admiten cero en todos los joints.
- `keyboard_teleop.yaml`: teclas, tópicos, pasos y velocidades del teclado.
- `base_teleop.yaml`: límites y watchdog del adaptador `cmd_vel_guard`.
- `wheel_contacts.yaml`: rodillos pasivos provisionales para las ruedas omni.
- `sensors.yaml` y `sensors_bridge.yaml`: sensores y puente Gazebo → ROS.

Para otro teclado o configuración usa:

```bash
ros2 run pepper_gz keyboard_teleop --ros-args -p use_sim_time:=true \
  -p config_file:=/ruta/keyboard_teleop.yaml \
  -p controllers_config:=/ruta/controllers.yaml
```

El launch también acepta `controllers_config`, `initial_positions_config`,
`teleop_config`, `wheel_contacts_config`, `sensors_config` y `bridge_config`.
Las rutas predeterminadas se resuelven desde el paquete instalado.

```text
keyboard_teleop -> /cmd_vel (Twist)
               -> cmd_vel_guard -> /base_controller/cmd_vel (TwistStamped)
               -> /<controlador>/joint_trajectory (JointTrajectory)
```

El [controlador omni estándar](https://control.ros.org/jazzy/doc/ros2_controllers/omni_wheel_drive_controller/doc/userdoc.html)
comanda `WheelFL`, `WheelB` y `WheelFR` en velocidad. Su odometría usa los estados
articulares, no la posición real de Gazebo. `enable_odom_tf` está deshabilitado
para reservar la autoridad TF a localización.

El adaptador limita el módulo XY a 0.50 m/s y el giro a 1.25 rad/s, y publica cero
tras 0.3 s reales sin entrada, incluso con `/clock` congelado. Solo publica con
la base activa, descarta órdenes de otra activación y detiene si hay más de un
publicador en `/cmd_vel`. Para varios emisores usa un multiplexor con salida
única. El controlador añade su propio watchdog de 0.3 s en tiempo simulado.

### Contacto físico

Las esferas rígidas originales no reproducen el rodillo transversal libre de
una rueda omni. Durante la conversión a SDF se reemplazan sus colisiones por
12 cápsulas pasivas por rueda, con articulaciones tangentes a la llanta.
Las mallas, los joints actuados y el árbol TF original permanecen intactos.
La masa, el centro de masa y el tensor de inercia agregados se conservan al
repartirlos entre cubo y rodillos. Los cuerpos adicionales existen solo en
Gazebo. No se cambia el motor físico ni se impone una velocidad al chasis.

Los tamaños y fricciones de rodillos son provisionales, no especificaciones
verificadas del hardware. La cinemática omni aproxima los centros originales,
ligeramente asimétricos, mediante una circunferencia ideal con separación de
120 grados. La odometría puede diferir de la pose física por esa aproximación
y por deslizamiento; requiere calibración y localización para navegar.

Las articulaciones `mimic` se conservan sin interfaces de comando propias,
como indica [gz_ros2_control](https://control.ros.org/jazzy/doc/gz_ros2_control/doc/index.html).
Los dedos no son actuadores independientes. No se eliminan relaciones `mimic`
ni se cambia la física para ocultar una incompatibilidad; los avisos reales de
Bullet deben distinguirse del resultado del control articular.

## Comprobar

```bash
ros2 control list_controllers
ros2 control list_hardware_interfaces
ros2 topic echo /cmd_vel
ros2 topic echo /base_controller/cmd_vel
ros2 topic echo /joint_states --once
```

Con simulación ejecutando, los controladores seleccionados deben aparecer como
`active`. Las interfaces de posición de cabeza, postura y brazos, y las de
velocidad de ruedas, deben estar reclamadas. Comprueba tanto estados articulares
como movimiento físico en Gazebo; la odometría por sí sola no lo demuestra.

```bash
colcon test --packages-select pepper_gz --ctest-args -R 'control_config|teleop'
colcon test-result --verbose
```
