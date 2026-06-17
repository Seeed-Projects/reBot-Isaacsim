# reBot-Isaacsim

A real-time mirror system that synchronizes a reBotArm physical robot arm with an NVIDIA Isaac Sim simulation. The system streams joint angles (including gravity-compensated, hand-guided motion) and gripper state from the real arm to Isaac Sim over a UDP JSON channel.

## System Architecture

```
┌──────────────────────────────────────────────────────────────────┐
│                         reBot-Isaacsim                           │
│                                                                  │
│   ┌─────────────────────┐         ┌─────────────────────────┐   │
│   │ Sender (Terminal 1)  │  UDP   │   Receiver (Terminal 2)  │   │
│   │                     │  JSON  │                         │   │
│   │ gravity_joint_sender │──────▶│ isaacsim_joint_receiver  │   │
│   │                     │ 5005   │                         │   │
│   │  • reBotArm_control  │        │  • Isaac Sim            │   │
│   │    _py uv env         │        │  • Ground + arm USD     │   │
│   │  • MIT + gravity FF   │        │  • Joint-angle sync     │   │
│   │  • Hand-guided OK     │        │  • Gripper dual-joint   │   │
│   └─────────────────────┘        └─────────────────────────┘   │
└──────────────────────────────────────────────────────────────────┘
```

## Directory Layout

```
reBot-Isaacsim/
├── pyproject.toml                           # uv workspace configuration
├── README.md
├── README_EN.md                             # English version of this README
├── reBotArm_Isaacsim/                       # Main example directory
│   ├── gravity_joint_sender.py              # Physical-arm sender (gravity comp + UDP)
│   ├── isaacsim_joint_receiver.py           # Isaac Sim receiver (joint-angle sync)
│   ├── isaacsim_joint_test_sender.py        # Test sender (no hardware, preset trajectory)
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
ip link show can0
# Make sure the state is UP and bitrate is 1000000

# If you need to restart CAN:
can_restart can0
# or
sudo ip link set can0 down && sudo ip link set can0 up type can bitrate 1000000 restart-ms 100
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

Two independent terminals are required:

### Terminal 1 — Launch the Isaac Sim receiver

```bash
cd reBotArm_Isaacsim
./run_isaacsim_receiver.sh
```

**Expected output:**
- The Isaac Sim GUI launches
- Ground and arm USD assets are loaded
- It listens on UDP `127.0.0.1:5005`
- It waits for the sender to connect

### Terminal 2 — Launch the sender (physical arm)

```bash
cd reBotArm_Isaacsim
./run_sender.sh
```

**Expected behavior:**
- The physical arm connects and gravity-compensation mode is enabled
- The arm can be moved freely by hand
- Joint angles are streamed over UDP at 60 Hz

**Launch order: receiver first, then the sender.**

## Hardware-Free Test Mode

If no physical arm is available, use the test sender to verify the Isaac Sim receiver:

```bash
# In Terminal 2, use the test sender instead of the real one
cd reBotArm_Isaacsim
./run_sender.sh --test
# or, directly:
python third_party/reBotArm_control_py/.venv/bin/python isaacsim_joint_test_sender.py
```

The test sender loops through a few preset joint poses with slow interpolation; no CAN connection is required.

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
