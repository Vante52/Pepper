# Descripción física de Pepper

Este paquete contiene geometrías, colisiones, propiedades inerciales, el árbol
cinemático y su visualización en RViz. El modelo editable es
`urdf/pepper_robot.xacro`; `urdf/pepper.urdf` es la exportación con los valores
por defecto.

## Árbol cinemático

```text
base_link
└── base                       fija; antiguo frame Tibia
    ├── WheelFL_link           WheelFL
    ├── WheelFR_link           WheelFR
    ├── WheelB_link            WheelB
    ├── sensores de base
    └── Pelvis                 KneePitch
        └── Hip                HipPitch
            └── torso          HipRoll
                ├── Neck → Head
                ├── LShoulder → brazo izquierdo
                ├── RShoulder → brazo derecho
                └── sensores del torso
```

`base_link` es la raíz del URDF y está fijo al chasis. `base` conserva el frame,
geometría, masa, sensores y ruedas del antiguo link `Tibia`.

## Referencia a la altura de la cadera

Por defecto, `base_link` está centrado en X/Y a **0.602 m del suelo nominal**:
`0.334 m` desde el suelo hasta `base`, más `0.268 m` hasta el pivote `HipPitch`
con las articulaciones en cero. En esa postura coincide con el origen de `Hip`.

La unión fija queda:

```text
T_base_link_base = Translation(0, 0, 0.334 - base_link_height)
                = Translation(0, 0, -0.268)  # valores por defecto, en metros
```

Esta posición es una referencia anatómica elegida a la altura de la cadera.
No cambia el centro de masa físico del modelo. Con las masas actuales y todas
las coordenadas articulares en cero, el COM calculado permanece aproximadamente
a `0.4145 m` del suelo. Las masas e inercias siguen en sus piezas; no se añade
masa al link de referencia vacío.

`base_link` no sigue el movimiento de la cadera: permanece fijo al chasis,
conservando `base_link → base` como unión fija. Al flexionar la rodilla,
el origen de `Hip` puede separarse de `base_link`.

## Conservación de la cinemática original

La cadena original descendía desde el torso: `torso → Hip → Pelvis → Tibia`.
Al invertirla, se trasladaron los frames hijos hasta los nuevos pivotes,
compensando los orígenes visuales, de colisión y de centro de masa, además de
las conexiones de cabeza, brazos y sensores del torso.

| Unión | Padre → hijo | Origen XYZ (m) | Eje |
| --- | --- | --- | --- |
| KneePitch | base → Pelvis | 0, 0, 0 | 0, -1, 0 |
| HipPitch | Pelvis → Hip | 0, 0, 0.268 | 0, -1, 0 |
| HipRoll | Hip → torso | 0, 0, 0.079 | -1, 0, 0 |

Los nombres, límites, esfuerzos, velocidades y coordenadas articulares se
conservan: un valor articular anterior produce la misma postura física relativa
al chasis. Subir ahora `base_link` solo cambia el origen de referencia, sin
alterar las posiciones relativas de las piezas.

## Visualización independiente

Desde la raíz del workspace:

```bash
colcon build --packages-select pepper_description
source install/setup.bash
ros2 launch pepper_description display.launch.py
```

El lanzamiento inicia `robot_state_publisher`, la GUI de estados articulares y
RViz. Tanto el Fixed Frame como el Target Frame de RViz son `base_link`.
La cuadrícula XY de RViz pasa por este frame, a la altura nominal de la cadera;
no representa el suelo.

Los argumentos disponibles son `base_link_height`, `gui`, `rviz` y
`use_sim_time`. Para ejecutar solo `robot_state_publisher`:

```bash
ros2 launch pepper_description display.launch.py gui:=false rviz:=false
```

La localización, `base_footprint`, `odom` y las poses externas se gestionan fuera
de este paquete. La simulación de Gazebo también se mantiene aparte.
