# Robot Skill Bridge

Conecta un agente conversacional con percepción y habilidades robóticas por
HTTP/JSON. El agente usa **Qwen2.5 7B** para conversar y seleccionar habilidades,
y **Qwen3-VL 2B Instruct** únicamente para analizar imágenes solicitadas. El movimiento lo ejecutan los controladores ROS 2.

```text
Conversación + memoria → Qwen2.5 7B → habilidad → HTTP → adaptador ROS 2
                              ↓ solo si necesita visión
                       imagen nueva → Qwen3-VL 2B → observaciones → Qwen2.5 7B
```

El agente selecciona habilidades con argumentos acotados. Puede combinar observaciones
y un destino métrico para buscar un objeto y acercarse a él. Los lazos de control
usan odometría y profundidad; el modelo no genera velocidades de ruedas ni posiciones
articulares.

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
ros2 launch robot_skill_bridge skill_bridge.launch.py
```

Terminal 3, con Ollama ejecutándose y el modelo instalado:

```bash
ollama list
# Si faltan los modelos:
ollama pull qwen2.5:7b
ollama pull qwen3-vl:2b-instruct

python3 src/robot_skill_bridge/host_demo/demo_ollama.py '¿Qué hay frente a ti?'
```

Sin argumento de texto se abre una conversación interactiva:

```bash
python3 src/robot_skill_bridge/host_demo/demo_ollama.py
```

Puedes decir «Me llamo Ana», preguntar después «¿Cómo me llamo?» y pedir «Saluda
con el brazo derecho». «Qué ves?» solicita una observación nueva cuando Qwen2.5
identifica la necesidad de visión. `/look` o `/look qué hay delante?` pide la
imagen explícitamente y omite esa interpretación inicial.
`/exit` termina y `/reset` borra conversación y estado de las reglas automáticas.
Un fallo de decisión se muestra y permite seguir escribiendo. Los errores reales
de adquisición se muestran como `[Percepción: ...]`.


El launch del bridge configura `use_sim_time=true` por defecto. Para hardware se
puede iniciar con `use_sim_time:=false`. El ejecutable `ros2 run` por sí solo conserva
el valor estándar de ROS 2 (`false`); si se utiliza en simulación hay que pasar
`--ros-args -p use_sim_time:=true`. Un `clock_mismatch` indica que los timestamps del
bridge y los sensores u odometría no pertenecen al mismo reloj. El bridge se inicia
por separado de Gazebo; este launch no duplica la simulación ni los controladores.

La decisión estructurada asocia cada habilidad a su propio esquema de argumentos.
Los errores de ejecución muestran también su `detail` en la terminal.

## Percepción

`PerceptionProvider.observe() -> Observation` es el contrato común de imágenes. No incluye
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
La descripción de imágenes no añade frames. El proveedor métrico asociado conserva
una instantánea de nube, calibración, TF y odometría para localizar regiones
de esa misma imagen. Usa QoS de datos de sensores.

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

## Mover la base y aproximarse a objetos

Con la simulación en Play, los controladores activos y el puente iniciado, prueba:

```text
Avanza medio metro.
Desplázate 30 centímetros a la izquierda.
Retrocede un metro.
Gira 90 grados a la derecha.
Busca el cono rojo y acércate a él.
Dirígete hacia la esfera morada.
Acércate al cilindro azul y quédate a un metro.
```

| Habilidad | Argumentos | Comprobación |
| --- | --- | --- |
| `move_relative` | `distance_m` (0–5), `direction_deg` (−180–180) | Posición y parada por odometría |
| `turn_relative` | `angle_deg` (−360–360) | Giro acumulado y parada por odometría |
| `approach_object` | `target` (descripción), `stop_distance_m` (0,6–2; defecto 0,8) | Objetivo visible y distancia por profundidad |

Las direcciones se refieren a la orientación inicial de la base: 0° delante,
90° izquierda, −90° derecha y 180° atrás. El desplazamiento mantiene esa
orientación. Los giros positivos son hacia la izquierda. Se usan metros y grados
en el contrato público, con conversión explícita a radianes en el adaptador.

`approach_object` es una habilidad compuesta del agente, registrada por
`visual_navigation` en `host_demo/agent_config.json`; no es un backend remoto que
conozca Ollama. Solo se ofrece al modelo cuando están disponibles las habilidades
remotas configuradas. Sus nombres se resuelven desde ese mapa. La forma y el
color del objetivo son texto libre, sin un clasificador limitado a conos.

1. La cámara obtiene una imagen nueva mientras la base está detenida.
2. El modelo visual devuelve una caja, coincidencia y confianza. El adaptador
   normaliza las coordenadas al contrato 0–1 de las habilidades.
   Si hay varios objetos indistinguibles o dudas, no se mueve.
3. `locate_target` usa el `observation_id` de esa imagen. Transforma los XYZ de la
   nube desde su frame real mediante TF, los proyecta en la cámara RGB con
   `CameraInfo` y estima la posición de la región en el frame de la base.
4. Calcula la distancia y el destino completos antes de moverse. Para un punto
   medido `(x,y)`, obtiene `d=hypot(x,y)` y una dirección `atan2(y,x)`. Si la separación
   deseada es `s`, el destino relativo es `(x,y) * max(0,(d-s)/d)`.
5. Envía **un único** `move_relative` con distancia `max(0,d-s)` y la dirección
   calculada. La base omnidireccional conserva su orientación y el adaptador ROS 2
   alcanza el punto con realimentación de odometría.
6. Al terminar, vuelve a medir el objeto para comprobar la separación. Esta
   comprobación no envía movimientos adicionales. Una diferencia fuera de tolerancia
   se informa como `verification_failed`; no se oculta ni se corrige automáticamente.

Por ejemplo: si el objeto se mide a 3,2 m y se pide quedar a 0,8 m, se envía un
solo desplazamiento de 2,4 m. El modelo visual únicamente identifica la región;
la distancia procede de los sensores y el cálculo se realiza en código.
La terminal muestra `[Movimiento calculado: ...]` antes de enviar el comando.
El resultado incluye `plan`, el destino XY, el comando enviado y `motion` con
las mediciones del ejecutor.

Si inicialmente no ve el objeto, busca con giros de 45° hasta completar como
máximo una vuelta. Una vez localizado, calcula el destino completo y ejecuta
un desplazamiento. Los límites de 24 observaciones de búsqueda y 5 m de recorrido,
junto con un presupuesto de 600 s reales reevaluado entre operaciones, están en
`visual_navigation`. Si el destino excede ese recorrido, se rechaza entero.
`goal_tolerance_m: 0.08` configura la tolerancia de la comprobación final por
profundidad. `step_distance_m` ya no interviene en la aproximación.

Las órdenes de distancia y las conversaciones normales no adquieren imágenes.
La búsqueda visual usa Qwen3-VL únicamente cuando necesita observar; no hace una
inferencia conversacional durante el desplazamiento ni al terminar. La respuesta final
se construye desde el resultado de ejecución. `--watch` conserva las reglas de
observación automática y no inicia aproximaciones por su cuenta.

Para probar desplazamientos sin IA:

```bash
python3 src/robot_skill_bridge/host_demo/demo_ollama.py --skill move_relative \
  --parameters '{"distance_m":0.25,"direction_deg":0}'
python3 src/robot_skill_bridge/host_demo/demo_ollama.py --skill turn_relative \
  --parameters '{"angle_deg":45}'
curl -sS http://127.0.0.1:8765/v1/skills/cancel -X POST
```

El adaptador publica `Twist` en `/cmd_vel`, pasando por el guard existente y sus
límites comunes de ruedas. Solo crea el publicador mientras ejecuta una habilidad.
Cierra otros teleops: dos emisores simultáneos detienen la base. Una pausa del
reloj, pérdida de odometría, falta de avance, cancelación o timeout provoca una
orden cero. El éxito requiere posición dentro de 1,5 cm, orientación dentro de
0,025 rad y velocidad prácticamente cero durante 0,2 s de simulación.
El perfil usa 0,15 m/s y 0,5 rad/s, dentro del intervalo probado en la calibración
de la base. La tolerancia de odometría no garantiza ese error en posición física.

La distancia final se mide desde el origen de la base hasta la región observada
del objeto; no es el espacio libre entre sus carcasas.

La aproximación necesita imagen y nube con tiempos separados como máximo 0,1 s,
calibración RGB rectificada sin distorsión y TF disponible. El perfil de suelo plano descarta puntos fuera del intervalo de altura
`target_min_z_m: -0.45` a `target_max_z_m: 1.5`, expresado en la base situada en
la cadera; así evita interpretar el suelo como un objeto. Cambia este intervalo
si cambias el frame de referencia o la escena. La nube de Gazebo
actual usa `CameraDepth_frame`; sus XYZ se transforman realmente antes de
proyectarlos en `CameraTop_optical_frame`. No se supone que profundidad y RGB
compartan resolución ni centro óptico. Se conservan hasta cuatro contextos
métricos durante 60 s. Si la base cambia de pose durante el análisis, el contexto
se invalida. Una imagen de archivo puede describirse, pero no autoriza una
aproximación métrica.

El formato visual es `bbox_2d=[x_min,y_min,x_max,y_max]`, con
`coordinate_scale: 1000` para el modelo actual, siguiendo el
[ejemplo oficial de grounding de Qwen3-VL](https://github.com/QwenLM/Qwen3-VL/blob/main/cookbooks/2d_grounding.ipynb).
Otro proveedor puede usar otra escala configurada; el bridge siempre recibe
coordenadas normalizadas 0–1. La confianza declarada por el modelo es un filtro
configurable, no una probabilidad calibrada ni una garantía de reconocimiento.

Este es control local en un área despejada. No incluye mapa, planificación de
trayectorias ni evitación de obstáculos; la cámara de profundidad tiene un campo
visible limitado. Para desplazarse por habitaciones con obstáculos, el backend
de aproximación deberá delegar el objetivo a navegación, por ejemplo Nav2.

### Escena de prueba

El paquete incluye un cono rojo, un cilindro azul y una esfera morada en un soporte.
Después de compilar, prepara los recursos **antes de iniciar Gazebo**:

```bash
source install/setup.bash
SKILL_SHARE=$(ros2 pkg prefix --share robot_skill_bridge)
export GZ_SIM_RESOURCE_PATH="$SKILL_SHARE/examples:${GZ_SIM_RESOURCE_PATH:-}"
ros2 launch pepper_gz simulation.launch.py paused:=false
```

En otra terminal del mismo workspace:

```bash
source install/setup.bash
SKILL_SHARE=$(ros2 pkg prefix --share robot_skill_bridge)
ros2 run ros_gz_sim create -world minimal_world \
  -file "$SKILL_SHARE/examples/visual_targets/model.sdf"
ros2 launch robot_skill_bridge skill_bridge.launch.py
```

Inicia después la demo conversacional en una tercera terminal. Las posiciones de
los objetos solo pertenecen a la escena: el agente y el bridge no las consultan
ni usan los nombres de las entidades de Gazebo para localizar objetivos.

En las pruebas iniciales de aproximación por segmentos del 6 de noviembre de 2026, el avance de 25 cm y el desplazamiento
lateral de 20 cm terminaron con errores de odometría de 1,47 y 1,48 cm.
La aproximación visual terminó a 0,819 m del cono y 0,836 m de la esfera para una
consigna de 0,8 m. El primer recorrido incluyó búsqueda y el segundo corrección
de orientación. Las mediciones y resultados de cada paso están en
[`examples/validation_results.json`](examples/validation_results.json), como registro histórico del modo por segmentos, con sus
límites de interpretación. No validan el modo de destino único. No equivalen a una evaluación estadística del modelo
visual ni a una medición independiente de precisión física.

## Saludar al ver una persona

Las conversaciones y las órdenes robóticas directas no adquieren imágenes.
Qwen2.5 interpreta primero el texto y solicita `perception_required: true` cuando
necesita comprobar la escena o una condición visual. Solo entonces se ejecuta
`look_front` y se envía la imagen nueva a Qwen3-VL. Este devuelve una descripción
y observaciones; Qwen2.5 usa esos datos de texto para responder y decidir las
habilidades. El modelo visual no recibe el catálogo ni solicita acciones.
Si falta una imagen válida,
no se ejecuta una acción pendiente de esa comprobación.

Las reglas automáticas se evalúan después de una observación solicitada. Para
observar continuamente sin esperar preguntas:

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

`host_demo/agent_config.json` configura ambos modelos:

```json
{
  "model": "qwen2.5:7b",
  "vision_model": "qwen3-vl:2b-instruct"
}
```

`model` procesa exclusivamente texto: conversación, interpretación inicial de
órdenes y respuestas sobre las imágenes analizadas. `vision_model`
solo recibe peticiones que incluyen una imagen actual y devuelve hechos visuales.
Un turno visual natural requiere interpretación de texto, análisis de imagen y
respuesta de texto; un turno normal no consulta el modelo visual. `--watch`
solicita observación explícitamente en cada ciclo y usa después Qwen2.5 para
evaluar las reglas y responder.
La personalidad, la memoria y las reglas siguen siendo comunes al agente.

Las habilidades no hacen una inferencia para confirmar su ejecución. La
respuesta sale de `action_responses` en `agent_config.json`, usando únicamente
`result.get('ok') is True` para confirmar éxito. Para `wave_right`, el éxito
responde «¡Listo, ya saludé!» y un fallo «No pude completar el saludo.». Las
habilidades nuevas usan `default` salvo que tengan sus propias plantillas.

La memoria conserva los últimos 12 turnos y el estado de los disparadores en:

```text
~/.local/state/pepper_agent/memory.json
```

Se respeta `XDG_STATE_HOME`. `--memory-file /ruta/sesion.json` selecciona otra
sesión y `--no-memory` conserva memoria únicamente durante el proceso. La memoria
no guarda imágenes, trayectorias ni articulaciones. No es una base de memoria
semántica ilimitada; al superar el límite se descartan los turnos antiguos.

El contexto enviado al modelo se limita por separado mediante
`context_history_turns: 4` y `context_history_chars: 2000`. Solo incluye el texto
reciente del diálogo; omite observaciones, acciones y resultados del historial.
El archivo de memoria conserva hasta `max_history_turns: 12`. Un valor 0 en
cualquiera de los límites de contexto omite el historial del prompt.

La interpretación inicial utiliza instrucciones cortas y un resumen de nombres
y descripciones de habilidades. No envía ejemplos de conversación ni la
configuración de reglas automáticas. El origen de la acción lo determina el
programa según el modo de ejecución, y las reglas se aplican fuera del modelo.
Conversaciones y comandos directos necesitan una inferencia en condiciones
normales. Ninguna habilidad añade una inferencia después de recibir su resultado.

`SKILL_BRIDGE_URL` cambia la URL del puente y `OLLAMA_URL` el endpoint `/api/chat`.
`--agent-config /ruta/agent.json` permite otra personalidad o configuración.
El cliente no importa ROS 2 ni el paquete Python del bridge.

El esquema JSON se envía mediante `format` de Ollama, sin copiarlo ni añadir un
ejemplo al prompt. Solo si falla la sintaxis se permite una corrección con el
mismo modelo y el esquema como ayuda. La decisión se valida antes de actuar;
no se repite la ejecución de una habilidad para corregir su respuesta.

## Separación entre módulos

- `catalog.py` y `http_api.py`: metadatos y contratos genéricos; no conocen ROS,
  Gazebo, Pepper ni Ollama. Los adaptadores se inyectan desde `main.py`.
- `perception.py`: observación, proveedor y backend de adquisición. La detección
  semántica corresponde al agente, no a este contrato.
- `ros2_camera_provider.py` y `ros2_target_provider.py`: adaptadores ROS 2 de
  imagen y localización métrica. `TargetProvider.locate()` permite sustituir
  la implementación sin introducir ROS en el agente.
- `trajectory_backend.py`: ejecuta `FollowJointTrajectory`, exige el controlador
  activo y verifica movimiento y posición final con `/joint_states`.
- `config/pepper_skills.yaml`: tópicos, acciones, articulaciones, límites y
  trayectorias específicos del robot.
- `host_demo/`: agente, memoria, política reactiva y adaptadores HTTP/Ollama.

`GET /v1/skills` publica `name`, `description`, `kind` y el esquema `parameters`.
`POST /v1/skills/execute` conserva `{"skill":"nombre"}` para habilidades sin
argumentos y admite adicionalmente `{"skill":"nombre","parameters":{...}}`.
El backend devuelve `SkillResult`; exclusivamente el adaptador HTTP elige el código
de respuesta. `POST /v1/skills/cancel` solicita cancelar el movimiento actual.
No recibe trayectorias ni velocidades articulares del LLM. Las acciones se serializan; consultar
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
