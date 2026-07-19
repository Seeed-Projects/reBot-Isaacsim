# reBot-Isaacsim

[简体中文](./README.md) | English | [Español](./README_ES.md)

reBot-Isaacsim is an NVIDIA Isaac Sim simulation project designed specifically for the reBotArm. It leverages Isaac Sim's high-fidelity physics engine to accurately replicate the kinematic characteristics and gripper coordination logic of the robot arm in a virtual environment, providing an independent simulation-only environment for control algorithm development, trajectory planning verification, and communication protocol testing.

## Component Overview

This project provides multiple sender components to cover different use cases:

| Component | Description |
|------|------|
| `gravity_joint_sender` | **Gravity Compensation + Handle Mode**: For modified robot arms (gripper removed, handle attached), using gravity compensation mode to allow manual manipulation, real-time joint angle sync to Isaac Sim |
| `isaacsim_ik_sender` | **Inverse Kinematics (IK) Mode**: Input end-effector pose, compute joint angles via IK solver, send to Isaac Sim |
| `isaacsim_traj_sender` | **Trajectory Planning (Traj) Mode**: Based on IK, adds joint-space trajectory planning (MIN_JERK profile) for smooth motion control |
| `isaacsim_joint_test_sender` | **Joint Test Mode**: No physical arm required, sends preset joint angle trajectories to verify Isaac Sim receiver and communication |
| `joint_reader_sender` | **Real-to-Sim Mapping Mode**: Read-only joint angles mapped to Isaac Sim, suitable for use with other control projects (e.g., when the physical arm is running other tasks, this feature can simultaneously map to Isaac Sim for visualization) |

## System Architecture

```
┌──────────────────────────────────────────────────────────────────┐
│                         reBot-Isaacsim                           │
│                                                                  │
│   ┌──────────────────────┐        ┌──────────────────────────┐   │
│   │ Sender (Terminal 1)  │  UDP   │   Receiver (Terminal 2)  │   │
│   │                      │  JSON  │                          │   │
│   │ gravity_joint_sender │──────▶ │ isaacsim_joint_receiver  │   │
│   │                      │ 5005   │                          │   │
│   │  • reBotArm_control  │        │  • Isaac Sim             │   │
│   │    _py uv env        │        │  • Ground + arm USD      │   │
│   │  • MIT + gravity FF  │        │  • Joint-angle sync      │   │
│   │  • Hand-guided OK    │        │  • Gripper dual-joint    │   │
│   └──────────────────────┘        └──────────────────────────┘   │
└──────────────────────────────────────────────────────────────────┘
```

## Directory Layout

```
reBot-Isaacsim/
├── pyproject.toml                           # uv workspace configuration
├── README.md
├── README_EN.md                             # English version of this README
├── README_ES.md                             # Spanish version of this README
├── reBotArm_Isaacsim/                       # Main example directory
│   ├── gravity_joint_sender.py              # Gravity comp + handle mode (modified arm, hand-guided)
│   ├── isaacsim_ik_sender.py                # Inverse kinematics mode (IK control)
│   ├── isaacsim_traj_sender.py              # Trajectory planning mode (IK + joint-space trajectory)
│   ├── isaacsim_joint_test_sender.py        # Joint test mode (preset trajectory, no hardware)
│   ├── joint_reader_sender.py                # Real-to-Sim mapping mode (read-only, sync visualization)
│   ├── isaacsim_joint_receiver.py           # Isaac Sim receiver (joint-angle sync)
│   ├── live_sync.py                         # Launch-instructions helper script
│   ├── run_sender.sh                        # Launch the sender
│   └── run_isaacsim_receiver.sh             # Launch the Isaac Sim receiver
├── third_party/
│   └── reBotArm_control_py/                 # Core control library (independent uv env)
│       ├── pyproject.toml
│       └── ...
└── usd/
    └── RS-rebot-dev-arm/
        └── 00-arm-rs_asm-v3.usda            # Isaac Sim robot asset
```

## Dependencies and Prerequisites

| Component | Requirement |
|------|------|
| Isaac Sim | Installed and `ISAACSIM_ROOT` environment variable configured |
| reBotArm firmware | Arm firmware flashed, CAN bus connected (`can0`) |
| CAN interface | `can0` is up with a bitrate of 1 Mbps (`can_restart can0`) |
| Python | 3.10+ |
| uv | Recommended for managing Python environments |
| reBotArm_control_py | `uv sync` has been run inside `third_party/reBotArm_control_py` |

### Check the CAN interface

```bash
# View CAN interface status
ip link show can0
# Make sure the state is UP and bitrate is 1000000

# If you need to configure or restart CAN:
sudo ip link set can0 down
sudo ip link set can0 up type can bitrate 1000000 restart-ms 100
```

## Environment Setup

### 1. Isaac Sim environment variable

Make sure the following is set in `.bashrc` or your shell config:

```bash
export ISAACSIM_ROOT=/home/seeed/IsaacSim/_build/linux-x86_64/release
```

### 2. reBotArm_control_py environment

```bash
cd third_party/reBotArm_control_py
uv sync
```

## Launch (Two-Terminal Mode)

Two independent terminals are required. **Terminal 1 is always the Isaac Sim receiver**, **Terminal 2 selects the corresponding sender based on the desired feature**.

### Terminal 1 — Launch the Isaac Sim receiver (common to all modes)

```bash
cd reBotArm_Isaacsim
./run_isaacsim_receiver.sh
```

**Expected output:**
- The Isaac Sim GUI launches
- Ground and arm USD assets are loaded
- It listens on UDP `127.0.0.1:5005`
- It waits for the sender to connect

### Terminal 2 — Choose the sender based on the feature

**Launch order: receiver first, then the sender.**

#### ① Gravity Compensation + Handle Mode (`gravity_joint_sender`)

For modified robot arms (gripper removed, handle attached), allows manual hand-guided control to drive the Isaac Sim simulation:

```bash
cd reBotArm_Isaacsim
./run_sender.sh
```

**Expected behavior:**
- The physical arm connects and MIT + gravity feed-forward compensation is enabled
- The arm can be moved freely by hand
- Joint angles are streamed over UDP at 60 Hz

#### ② Inverse Kinematics Mode (`isaacsim_ik_sender`)

Input end-effector pose (position/orientation), solve via IK and drive the Isaac Sim arm. Run directly with `uv run` from `reBotArm_Isaacsim/`:

```bash
cd reBotArm_Isaacsim
uv run python isaacsim_ik_sender.py
```

**Input format (one per line):**
```
x y z                       # position (m), orientation held
x y z r p y                 # position + orientation (m/deg)
q j1 j2 j3 j4 j5 j6         # direct joint angles (deg)
gripper <0~1>                # update gripper only
```

#### ③ Trajectory Planning Mode (`isaacsim_traj_sender`)

IK plus joint-space trajectory planning (MIN_JERK) for smooth motion. Run directly with `uv run` from `reBotArm_Isaacsim/`:

```bash
cd reBotArm_Isaacsim
uv run python isaacsim_traj_sender.py
```

**Input format (one per line):**
```
x y z                       # position (m)
x y z r p y                 # position + orientation (m/deg)
q j1 j2 j3 j4 j5 j6         # direct joint-space target (deg)
gripper <0~1>                # update gripper only
speed <scale>                # adjust trajectory duration scale
resync                       # re-read current joint angles from simulator
```

#### ④ Joint Test Mode (`isaacsim_joint_test_sender`)

No hardware required; preset trajectory loop to verify communication and Isaac Sim receiver:

```bash
cd reBotArm_Isaacsim
uv run python isaacsim_joint_test_sender.py
```

The test sender loops through a few preset joint poses with slow interpolation; no CAN connection is required.

#### ⑤ Real-to-Sim Mapping Mode (`joint_reader_sender`)

Read-only joint angles mapped to Isaac Sim; suitable for use while the physical arm is running other tasks (simultaneous visualization). Run directly with `uv run` from `reBotArm_Isaacsim/`:

```bash
cd reBotArm_Isaacsim
uv run python joint_reader_sender.py
```

**Expected behavior:**
- Joint angles are read in passive feedback mode only (no control commands are sent)
- Joint angles are streamed over UDP at 60 Hz
- When the physical arm is being controlled by another project, this still mirrors its motion into Isaac Sim for visualization

## Communication Protocol

UDP JSON on `127.0.0.1:5005`.

**Per-frame payload sent by the sender:**

```json
{
  "sequence": 123,
  "timestamp": 1718000000.123,
  "joint_positions": [0.0, 0.1, 0.2, -0.1, 0.0, -0.02],
  "gripper_position": 0.05
}
```

| Field | Type | Description |
|------|------|------|
| `sequence` | int | Monotonically increasing sequence number |
| `timestamp` | float | Unix timestamp (seconds) |
| `joint_positions` | float[6] | First 6 joint angles (rad) |
| `gripper_position` | float | Gripper position (m); the sender converts it via `GRIPPER_POSITION_SCALE=0.03` |

**Gripper control chain:**
sender `gripper_q` → `gripper_position = -gripper_q × 0.03` → receiver `× 0.01` → dual-joint position target

## Configuration Parameters

### Sender (`gravity_joint_sender.py`)

| Parameter | Default | Description |
|------|--------|------|
| `ARM_JOINT_COUNT` | 6 | Number of joints |
| `DEFAULT_PORT` | 5005 | UDP port |
| `DEFAULT_SEND_HZ` | 60.0 | Send frequency (Hz) |
| `GRIPPER_POSITION_SCALE` | 0.03 | Scale factor from gripper angle to position |
| `position_alpha` | 0.2 | Low-pass filter coefficient |

### Receiver (`isaacsim_joint_receiver.py`)

| Parameter | Default | Description |
|------|--------|------|
| `ARM_JOINT_COUNT` | 6 | Number of joints |
| `DEFAULT_PORT` | 5005 | UDP port |
| `DEFAULT_RENDER_HZ` | 120.0 | Simulation render frequency (Hz) |
| `GRIPPER_POSITION_SCALE` | 0.01 | Additional gripper position scale factor |
| `ROBOT_PRIM_PATH` | `/World/reBotArm` | Robot Prim path inside Isaac Sim |
| `ASSET_RELATIVE_PATH` | `usd/RS-rebot-dev-arm/00-arm-rs_asm-v3.usda` | USD asset path relative to the repo root |

## Troubleshooting

### `OSError: [Errno 98] Address already in use`

Port 5005 is already in use. First identify and stop the occupying process:

```bash
# Inspect the process holding the port
sudo lsof -i :5005

# Kill the process (replace <PID> with the actual value)
kill <PID>
```

### Isaac Sim asset not found

Confirm the USD asset path exists, or check that `REPO_ROOT` is correct:

```bash
ls usd/RS-rebot-dev-arm/00-arm-rs_asm-v3.usda
```

### CAN bus not ready

Make sure the CAN interface is up at the correct bitrate:

```bash
can_restart can0
# Verify:
ip -details link show can0 | grep bitrate
```

### Joint angles out of sync

- Confirm the sender and receiver ports match (both 5005)
- Check that the sender log keeps printing `[send]`
- Check that the receiver log keeps printing `[recv]`
- Try `isaacsim_joint_test_sender.py` to rule out hardware issues

## Components and Python Environments

| Component | Python environment | Launcher |
|------|------------|---------|
| Sender (physical arm) | `reBotArm_control_py` uv environment | `run_sender.sh` |
| Sender (test mode) | `reBotArm_control_py` uv environment | `isaacsim_joint_test_sender.py` |
| Receiver | Isaac Sim official Python (`python.sh`) | `run_isaacsim_receiver.sh` |
