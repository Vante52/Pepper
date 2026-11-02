# Robot Skill Bridge

Conecta un agente conversacional con percepción y habilidades robóticas por
HTTP/JSON. El agente usa **Qwen3.5 4B** para conversar, interpretar imágenes y
seleccionar habilidades. El movimiento lo ejecutan los controladores ROS 2.

```text
Conversación + memoria → Qwen3.5 4B → nombre de habilidad → HTTP → adaptador ROS 2
                              ↑                                     ↓
                         imagen actual ← PerceptionProvider ← cámara / archivo
```

La integración visión–lenguaje–acción es discreta: el modelo reconoce una escena
y solicita habilidades predefinidas. No genera posiciones articulares ni es una
política entrenada de control continuo.

## Arranque

Desde la raíz del workspace:

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select robot_skill_bridge
source install/setup.bash
```

Terminal 1:

```bash
source install/setup.bash
ros2 launch pepper_gz simulation.launch.py paused:=false
```

Terminal 2:

```bash
source install/setup.bash
ros2 run robot_skill_bridge skill_bridge --ros-args -p use_sim_time:=true
```

Terminal 3, con Ollama ejecutándose y el modelo instalado:

```bash
ollama list
# Si falta el modelo:
ollama pull qwen3.5:4b

python3 src/robot_skill_bridge/host_demo/demo_ollama.py '¿Qué hay frente a ti?'
```

Sin argumento de texto se abre una conversación interactiva:

```bash
python3 src/robot_skill_bridge/host_demo/demo_ollama.py
```

Puedes decir «Me llamo Ana», preguntar después «¿Cómo me llamo?» y pedir «Saluda
con el brazo derecho». `/exit` termina y `/reset` borra conversación y estado de
las reglas automáticas.

## Percepción

`PerceptionProvider.observe() -> Observation` es el contrato común. No incluye
inferencias de personas ni depende del proveedor de IA.

| Implementación | Fuente | Uso |
| --- | --- | --- |
| `ROS2CameraProvider` | `sensor_msgs/msg/Image` | Cámara real o simulada |
| `ImageFileProvider` | Archivo de imagen | Pruebas reproducibles |
| `MockPerceptionProvider` | Secuencia de observaciones inyectadas | Pruebas sin ROS ni IA |

El perfil `config/pepper_skills.yaml` registra `look_front`, de tipo
`perception`, con la cámara `/pepper/camera/top/image_raw`. El adaptador solicita
un frame nuevo en cada adquisición, convierte a RGB y PNG, conserva el
`header.frame_id` original y entrega como máximo 640 píxeles en el lado mayor.
No añade frames ni transforma coordenadas. Usa QoS de datos de sensores.

`captured_at` es la hora real de adquisición del proveedor; `sensor_stamp`
conserva el tiempo ROS de la imagen. Un archivo no tiene `sensor_stamp`: su
fecha de captura real es desconocida. El timestamp de adquisición de un archivo
no convierte una foto grabada en una escena en vivo.

Para probar percepción directamente, sin IA:

```bash
curl -sS http://127.0.0.1:8765/v1/skills
curl -sS http://127.0.0.1:8765/v1/skills/execute \
  -H 'Content-Type: application/json' -d '{"skill":"look_front"}' > /tmp/observation.json
```

La respuesta contiene `observation.image_base64`, `media_type`, `captured_at`,
`source`, `frame_id` y `sensor_stamp`. La demo envía la imagen a Ollama; no imprime
ni guarda sus bytes en el historial. Si falla la cámara, el agente puede seguir
conversando, pero debe declarar `person_visible: null`. Una respuesta que afirme
haber visto algo sin imagen se rechaza.

## Saludar al ver una persona

Cada turno obtiene una imagen y evalúa las reglas del agente. Para observar sin
esperar preguntas:

```bash
python3 src/robot_skill_bridge/host_demo/demo_ollama.py --watch --interval 5
```

Con una persona visible, la decisión tiene esta forma:

```json
{
  "reply": "Veo una persona frente a mí.",
  "observations": {"person_visible": true},
  "action": {"skill": "wave_right", "source": "observation"}
}
```

La regla está en `host_demo/agent_config.json`, fuera del núcleo y de la
trayectoria de Pepper. Solicita `wave_right` una vez al detectar una persona.
Mantenerla visible no repite el saludo, aunque venza el cooldown. Se rearma tras
**dos observaciones sin persona** y exige **60 segundos** desde el último
intento. Una observación desconocida no rearma. Los fallos no provocan reintentos
automáticos mientras la persona permanezca visible. Una orden explícita de
saludar sigue permitida independientemente de esta regla.

Solo una observación con edad inferior a `max_observation_age_sec` (60 segundos
por defecto, incluida la inferencia) puede disparar o rearmar una regla. Una
inferencia lenta puede describir la imagen obtenida y suprimir el movimiento por
caducidad. Configura este límite según la aplicación; `--interval` es la pausa
entre ciclos, no una frecuencia garantizada de percepción.

El mundo `minimal_world` está vacío: hay que añadir una persona o un objetivo
visual para obtener `person_visible: true`. El clasificador también puede
reconocer personas en fotografías o pantallas; no distingue de forma garantizada
una persona presente de su representación. No se hace seguimiento de identidades:
el bloqueo se aplica a presencia en la escena, no a cada individuo.

## Modelo, personalidad y memoria

`host_demo/agent_config.json` selecciona el único modelo de todos los turnos y
contiene personalidad, reglas y tamaño de memoria. La misma instancia del
adaptador verbaliza el resultado real después de ejecutar una acción. No hay
un segundo modelo de conversación, detección o movimiento.

La memoria conserva los últimos 12 turnos y el estado de los disparadores en:

```text
~/.local/state/pepper_agent/memory.json
```

Se respeta `XDG_STATE_HOME`. `--memory-file /ruta/sesion.json` selecciona otra
sesión y `--no-memory` conserva memoria únicamente durante el proceso. La memoria
no guarda imágenes, trayectorias ni articulaciones. No es una base de memoria
semántica ilimitada; al superar el límite se descartan los turnos antiguos.

`SKILL_BRIDGE_URL` cambia la URL del puente y `OLLAMA_URL` el endpoint `/api/chat`.
`--agent-config /ruta/agent.json` permite otra personalidad o configuración.
El cliente no importa ROS 2 ni el paquete Python del bridge.

En Ollama 0.21.0 se observó que `think=false` puede ignorar `format` con Qwen3.5
([incidencia oficial](https://github.com/ollama/ollama/issues/14645)). El adaptador
incluye el esquema y un ejemplo en el prompt; si la sintaxis no es JSON, admite
una única corrección con **el mismo modelo**. Después valida la decisión de forma
estricta antes de actuar. Si vuelve a fallar, no se ejecuta nada. `thinking` puede
activarse en la configuración, con mayor latencia; no se expone su razonamiento.

## Separación entre módulos

- `catalog.py` y `http_api.py`: metadatos y contratos genéricos; no conocen ROS,
  Gazebo, Pepper ni Ollama. Los adaptadores se inyectan desde `main.py`.
- `perception.py`: observación, proveedor y backend de adquisición. La detección
  semántica corresponde al agente, no a este contrato.
- `ros2_camera_provider.py`: único adaptador de percepción que importa ROS 2.
- `trajectory_backend.py`: ejecuta `FollowJointTrajectory`, exige el controlador
  activo y verifica movimiento y posición final con `/joint_states`.
- `config/pepper_skills.yaml`: tópicos, acciones, articulaciones, límites y
  trayectorias específicos del robot.
- `host_demo/`: agente, memoria, política reactiva y adaptadores HTTP/Ollama.

`GET /v1/skills` publica `name`, `description` y `kind` (`action` o `perception`).
`POST /v1/skills/execute` sigue aceptando exclusivamente `{"skill":"nombre"}`.
No recibe trayectorias ni imágenes del LLM. Las acciones se serializan; consultar
percepción no ocupa el bloqueo de movimiento. El servidor escucha en loopback.

Para probar el movimiento sin IA:

```bash
python3 src/robot_skill_bridge/host_demo/demo_ollama.py --skill wave_right
```

El saludo dura 4,5 segundos de simulación, con 0,5 segundos de margen para
alcanzar la postura final. Su duración real depende de la velocidad de Gazebo.

Un resultado `succeeded` exige aceptación, éxito de la acción y mediciones dentro
de tolerancia. Un timeout solicita cancelar y bloquea nuevos movimientos hasta
conocer el resultado terminal; no afirma que el robot ya se detuvo.

## Usar imágenes grabadas

Copia `config/pepper_skills.yaml` a otro archivo y sustituye solamente el proveedor:

```yaml
perception_providers:
  front_camera:
    type: image_file
    path: /ruta/escena.png
    max_dimension: 640
```

Conserva `skills` y arranca:

```bash
ros2 run robot_skill_bridge skill_bridge --ros-args \
  -p config_file:=/ruta/perfil_archivo.yaml -p use_sim_time:=true
```

Las rutas relativas de imágenes se resuelven respecto al YAML. Cambiar de fuente
no requiere modificar el agente ni HTTP. Un archivo con una persona puede
activar el saludo en la simulación: usa una sesión de memoria separada para tests.

## Pruebas

```bash
source install/setup.bash
colcon test --packages-select robot_skill_bridge --event-handlers console_direct+
colcon test-result --test-result-base build/robot_skill_bridge --verbose
```

Las pruebas de percepción y del agente usan archivos y mocks. Comprueban
conversación, persistencia, detección estructurada, caducidad, rearme, bloqueo de
repetición, rechazo de contratos inválidos y errores de adquisición. No llaman
a Ollama ni mueven el robot. Las pruebas ROS necesitan las dependencias Jazzy.

Con la simulación y el puente activos, esta prueba **mueve el robot** y verifica
las habilidades de trayectoria del catálogo:

```bash
python3 -m pytest src/robot_skill_bridge/test/test_robot_integration.py \
  --run-robot-integration --bridge-url http://127.0.0.1:8765 -v -s
```

Para una prueba visual acotada del agente real:

```bash
python3 src/robot_skill_bridge/host_demo/demo_ollama.py \
  --memory-file /tmp/pepper_vision_test.json --watch --iterations 2 --interval 1
```

Con la misma persona continuamente visible debe emitir como máximo un saludo.
La percepción del modelo es probabilística; las pruebas deterministas verifican
la política y los contratos, no garantizan exactitud de reconocimiento en todas
las escenas. La geometría, la cinemática y la física permanecen en los paquetes
existentes; este paquete no añade planificación de colisiones.
