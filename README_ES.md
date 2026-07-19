# reBot-Isaacsim

[简体中文](./README.md) | [English](./README_EN.md) | Español

reBot-Isaacsim es un proyecto de simulación en NVIDIA Isaac Sim diseñado específicamente para el reBotArm. Aprovecha el motor de física de alta fidelidad de Isaac Sim para reproducir con precisión en un entorno virtual las características cinemáticas del brazo robótico y la lógica de coordinación de la pinza (gripper), y ofrece una plataforma independiente de simulación pura para el desarrollo de algoritmos de control, la verificación de la planificación de trayectorias y las pruebas de protocolos de comunicación.

## Descripción general de los componentes

Este proyecto proporciona varios componentes emisores para cubrir distintos casos de uso:

| Componente | Descripción |
|------|------|
| `gravity_joint_sender` | **Modo de compensación de gravedad + asa**: para brazos robóticos modificados (pinza retirada, asa acoplada); el modo de compensación de gravedad permite mover el brazo a mano y sincroniza en tiempo real los ángulos articulares con Isaac Sim |
| `isaacsim_ik_sender` | **Modo de cinemática inversa (IK)**: se introduce la pose del efector final; el solver de IK calcula los ángulos articulares y los envía a Isaac Sim |
| `isaacsim_traj_sender` | **Modo de planificación de trayectorias (Traj)**: sobre la base de la IK, añade planificación de trayectorias en el espacio articular (perfil MIN_JERK) para un control de movimiento suave |
| `isaacsim_joint_test_sender` | **Modo de prueba de articulaciones**: no requiere brazo físico; envía trayectorias de ángulos articulares predefinidas para verificar el receptor de Isaac Sim y la comunicación |
| `joint_reader_sender` | **Modo de mapeo Real-to-Sim**: lee los ángulos articulares en modo de solo lectura y los mapea a Isaac Sim; adecuado para usarlo junto con otros proyectos de control (por ejemplo, cuando el brazo físico está ejecutando otras tareas, esta función permite reflejar simultáneamente su movimiento en Isaac Sim para visualizarlo) |

## Arquitectura del sistema

```
┌──────────────────────────────────────────────────────────────────┐
│                         reBot-Isaacsim                           │
│                                                                  │
│   ┌──────────────────────┐        ┌──────────────────────────┐   │
│   │ Emisor (Terminal 2)  │  UDP   │   Receptor (Terminal 1)  │   │
│   │                      │  JSON  │                          │   │
│   │ gravity_joint_sender │──────▶ │ isaacsim_joint_receiver  │   │
│   │                      │ 5005   │                          │   │
│   │  • reBotArm_control  │        │  • Isaac Sim             │   │
│   │    _py entorno uv    │        │  • USD suelo + brazo     │   │
│   │  • MIT + FF gravedad │        │  • Sinc. articulaciones  │   │
│   │  • Guiado a mano OK  │        │  • Pinza biarticulada    │   │
│   └──────────────────────┘        └──────────────────────────┘   │
└──────────────────────────────────────────────────────────────────┘
```

## Estructura de directorios

```
reBot-Isaacsim/
├── pyproject.toml                           # Configuración del workspace de uv
├── README.md
├── README_EN.md                             # Versión en inglés de este README
├── README_ES.md                             # Versión en español de este README
├── reBotArm_Isaacsim/                       # Directorio principal de ejemplos
│   ├── gravity_joint_sender.py              # Modo compensación de gravedad + asa (brazo modificado, guiado a mano)
│   ├── isaacsim_ik_sender.py                # Modo de cinemática inversa (control IK)
│   ├── isaacsim_traj_sender.py              # Modo de planificación de trayectorias (IK + trayectoria en espacio articular)
│   ├── isaacsim_joint_test_sender.py        # Modo de prueba de articulaciones (trayectoria predefinida, sin hardware)
│   ├── joint_reader_sender.py                # Modo de mapeo Real-to-Sim (solo lectura, visualización sincronizada)
│   ├── isaacsim_joint_receiver.py           # Receptor de Isaac Sim (sincronización de ángulos articulares)
│   ├── live_sync.py                         # Script auxiliar con instrucciones de arranque
│   ├── run_sender.sh                        # Lanza el emisor
│   └── run_isaacsim_receiver.sh             # Lanza el receptor de Isaac Sim
├── third_party/
│   └── reBotArm_control_py/                 # Biblioteca de control principal (entorno uv independiente)
│       ├── pyproject.toml
│       └── ...
└── usd/
    └── RS-rebot-dev-arm/
        └── 00-arm-rs_asm-v3.usda            # Asset del robot para Isaac Sim
```

## Dependencias y requisitos previos

| Componente | Requisito |
|------|------|
| Isaac Sim | Instalado y con la variable de entorno `ISAACSIM_ROOT` configurada |
| Firmware de reBotArm | Firmware del brazo flasheado y bus CAN conectado (`can0`) |
| Interfaz CAN | `can0` activa (UP) con un bitrate de 1 Mbps (`can_restart can0`) |
| Python | 3.10+ |
| uv | Recomendado para gestionar los entornos de Python |
| reBotArm_control_py | Se ha ejecutado `uv sync` dentro de `third_party/reBotArm_control_py` |

### Comprobar la interfaz CAN

```bash
# Ver el estado de la interfaz CAN
ip link show can0
# Asegúrate de que el estado es UP y el bitrate es 1000000

# Si necesitas configurar o reiniciar el CAN:
sudo ip link set can0 down
sudo ip link set can0 up type can bitrate 1000000 restart-ms 100
```

## Preparación del entorno

### 1. Variable de entorno de Isaac Sim

Asegúrate de que lo siguiente está definido en `.bashrc` o en la configuración de tu shell:

```bash
export ISAACSIM_ROOT=/home/seeed/IsaacSim/_build/linux-x86_64/release
```

### 2. Entorno de reBotArm_control_py

```bash
cd third_party/reBotArm_control_py
uv sync
```

## Arranque (modo de dos terminales)

Se necesitan dos terminales independientes. **El terminal 1 es siempre el receptor de Isaac Sim** y **en el terminal 2 se elige el emisor correspondiente a la función deseada**.

### Terminal 1 — Arrancar el receptor de Isaac Sim (común a todos los modos)

```bash
cd reBotArm_Isaacsim
./run_isaacsim_receiver.sh
```

**Salida esperada:**
- Se abre la interfaz gráfica de Isaac Sim
- Se cargan los assets USD del suelo y del brazo
- Escucha en UDP `127.0.0.1:5005`
- Queda a la espera de que el emisor se conecte

### Terminal 2 — Elegir el emisor según la función

**Orden de arranque: primero el receptor y después el emisor.**

#### 1. Modo de compensación de gravedad + asa (`gravity_joint_sender`)

Para brazos robóticos modificados (pinza retirada, asa acoplada); permite guiar el brazo a mano para controlar la simulación de Isaac Sim:

```bash
cd reBotArm_Isaacsim
./run_sender.sh
```

**Comportamiento esperado:**
- Se conecta el brazo físico y se activa el modo MIT con prealimentación (feed-forward) de gravedad
- El brazo puede moverse libremente con la mano
- Los ángulos articulares se envían por UDP a 60 Hz

#### 2. Modo de cinemática inversa (`isaacsim_ik_sender`)

Introduce la pose del efector final (posición/orientación); el sistema la resuelve mediante IK y mueve el brazo en Isaac Sim. Ejecuta directamente con `uv run` desde `reBotArm_Isaacsim/`:

```bash
cd reBotArm_Isaacsim
uv run python isaacsim_ik_sender.py
```

**Formato de entrada (una orden por línea):**
```
x y z                       # posición (m), se mantiene la orientación
x y z r p y                 # posición + orientación (m/grados)
q j1 j2 j3 j4 j5 j6         # ángulos articulares directos (grados)
gripper <0–1>                # actualiza solo la pinza
```

#### 3. Modo de planificación de trayectorias (`isaacsim_traj_sender`)

IK más planificación de trayectorias en el espacio articular (MIN_JERK) para un movimiento suave. Ejecuta directamente con `uv run` desde `reBotArm_Isaacsim/`:

```bash
cd reBotArm_Isaacsim
uv run python isaacsim_traj_sender.py
```

**Formato de entrada (una orden por línea):**
```
x y z                       # posición (m)
x y z r p y                 # posición + orientación (m/grados)
q j1 j2 j3 j4 j5 j6         # objetivo directo en espacio articular (grados)
gripper <0–1>                # actualiza solo la pinza
speed <escala>               # ajusta la escala de duración de la trayectoria
resync                       # vuelve a leer del simulador los ángulos articulares actuales
```

#### 4. Modo de prueba de articulaciones (`isaacsim_joint_test_sender`)

No requiere hardware; envía en bucle una trayectoria predefinida para verificar la comunicación y el receptor de Isaac Sim:

```bash
cd reBotArm_Isaacsim
uv run python isaacsim_joint_test_sender.py
```

El emisor de prueba recorre en bucle varias poses articulares predefinidas con interpolación lenta; no se necesita conexión CAN.

#### 5. Modo de mapeo Real-to-Sim (`joint_reader_sender`)

Ángulos articulares de solo lectura mapeados a Isaac Sim; adecuado para usarlo mientras el brazo físico ejecuta otras tareas (visualización simultánea). Ejecuta directamente con `uv run` desde `reBotArm_Isaacsim/`:

```bash
cd reBotArm_Isaacsim
uv run python joint_reader_sender.py
```

**Comportamiento esperado:**
- Los ángulos articulares se leen únicamente en modo de realimentación pasiva (no se envía ningún comando de control)
- Los ángulos articulares se envían por UDP a 60 Hz
- Cuando el brazo físico está controlado por otro proyecto, su movimiento se sigue reflejando en Isaac Sim para visualizarlo

## Protocolo de comunicación

JSON sobre UDP en `127.0.0.1:5005`.

**Contenido (payload) que el emisor envía en cada datagrama:**

```json
{
  "sequence": 123,
  "timestamp": 1718000000.123,
  "joint_positions": [0.0, 0.1, 0.2, -0.1, 0.0, -0.02],
  "gripper_position": 0.05
}
```

| Campo | Tipo | Descripción |
|------|------|------|
| `sequence` | int | Número de secuencia monótonamente creciente |
| `timestamp` | float | Marca de tiempo Unix (segundos) |
| `joint_positions` | float[6] | Ángulos de las 6 primeras articulaciones (rad) |
| `gripper_position` | float | Posición de la pinza (m); el emisor la convierte mediante `GRIPPER_POSITION_SCALE=0.03` |

**Cadena de control de la pinza:**
emisor `gripper_q` → `gripper_position = -gripper_q × 0.03` → receptor `× 0.01` → objetivo de posición de las dos articulaciones

## Parámetros de configuración

### Emisor (`gravity_joint_sender.py`)

| Parámetro | Valor por defecto | Descripción |
|------|--------|------|
| `ARM_JOINT_COUNT` | 6 | Número de articulaciones |
| `DEFAULT_PORT` | 5005 | Puerto UDP |
| `DEFAULT_SEND_HZ` | 60.0 | Frecuencia de envío (Hz) |
| `GRIPPER_POSITION_SCALE` | 0.03 | Factor de conversión del ángulo de la pinza a posición |
| `position_alpha` | 0.2 | Coeficiente del filtro paso bajo |

### Receptor (`isaacsim_joint_receiver.py`)

| Parámetro | Valor por defecto | Descripción |
|------|--------|------|
| `ARM_JOINT_COUNT` | 6 | Número de articulaciones |
| `DEFAULT_PORT` | 5005 | Puerto UDP |
| `DEFAULT_RENDER_HZ` | 120.0 | Frecuencia de renderizado de la simulación (Hz) |
| `GRIPPER_POSITION_SCALE` | 0.01 | Factor de escala adicional de la posición de la pinza |
| `ROBOT_PRIM_PATH` | `/World/reBotArm` | Ruta del Prim del robot dentro de Isaac Sim |
| `ASSET_RELATIVE_PATH` | `usd/RS-rebot-dev-arm/00-arm-rs_asm-v3.usda` | Ruta del asset USD relativa a la raíz del repositorio |

## Resolución de problemas

### `OSError: [Errno 98] Address already in use`

El puerto 5005 ya está en uso. Identifica primero el proceso que lo ocupa y termínalo:

```bash
# Consultar el proceso que ocupa el puerto
sudo lsof -i :5005

# Terminar el proceso (sustituye <PID> por el valor real)
kill <PID>
```

### No se encuentra el asset de Isaac Sim

Confirma que la ruta del asset USD existe o comprueba que `REPO_ROOT` es correcto:

```bash
ls usd/RS-rebot-dev-arm/00-arm-rs_asm-v3.usda
```

### El bus CAN no está listo

Asegúrate de que la interfaz CAN está activa con el bitrate correcto:

```bash
can_restart can0
# Verificar:
ip -details link show can0 | grep bitrate
```

### Los ángulos articulares no se sincronizan

- Confirma que los puertos del emisor y del receptor coinciden (ambos 5005)
- Comprueba que el registro del emisor sigue imprimiendo `[send]`
- Comprueba que el registro del receptor sigue imprimiendo `[recv]`
- Prueba con `isaacsim_joint_test_sender.py` para descartar problemas de hardware

## Componentes y entornos de Python

| Componente | Entorno de Python | Script de arranque |
|------|------------|---------|
| Emisor (brazo físico) | Entorno uv de `reBotArm_control_py` | `run_sender.sh` |
| Emisor (modo de prueba) | Entorno uv de `reBotArm_control_py` | `isaacsim_joint_test_sender.py` |
| Receptor | Python oficial de Isaac Sim (`python.sh`) | `run_isaacsim_receiver.sh` |
