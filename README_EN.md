# reBot-Isaacsim

[简体中文](./README.md) | English | [Español](./README_ES.md)

reBot-Isaacsim is an NVIDIA Isaac Sim simulation project designed specifically for the reBotArm. It leverages Isaac Sim's high-fidelity physics engine to accurately replicate the kinematic characteristics and gripper coordination logic of the robot arm in a virtual environment, providing an independent simulation-only environment for control algorithm development, trajectory planning verification, and communication protocol testing.

## Component Overview

This project provides multiple sender components to cover different use cases:

| Component | Description |
|------|------|
| `gravity_joint_sender` | **Gravity Compensation + Handle Mode**: For modified robot arms (gripper removed, handle attached), hand-guided; compensation comes from upstream `GravityCompensation`, this repo only mirrors joint angles to Isaac Sim |
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
│   │ Sender (Terminal 2)  │  UDP   │   Receiver (Terminal 1)  │   │
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
│   ├── gravity_joint_sender.py              # Gravity-comp handle mode (upstream GravityCompensation + UDP)
│   ├── isaacsim_ik_sender.py                # Inverse kinematics mode (IK control)
│   ├── isaacsim_traj_sender.py              # Trajectory planning mode (IK + joint-space trajectory)
│   ├── isaacsim_joint_test_sender.py        # Joint test mode (preset trajectory, no hardware)
│   ├── joint_reader_sender.py                # Real-to-Sim mapping mode (read-only, sync visualization)
│   ├── isaacsim_joint_receiver.py           # Isaac Sim receiver (joint-angle sync)
│   ├── live_sync.py                         # Launch-instructions helper script
│   ├── set_hw_rs.py                         # Point submodule hardware YAML at RS (local; do not commit)
│   ├── run_sender.sh                        # Launch the sender
│   └── run_isaacsim_receiver.sh             # Launch the Isaac Sim receiver
├── .gitmodules
├── third_party/
│   └── reBotArm_control_py/                 # git submodule: upstream control library
└── usd/
    └── ReBot_Arm_RS/
        └── ReBot_Arm_RS.usda            # Isaac Sim robot asset
```

## Dependencies and Prerequisites

| Component | Requirement |
|------|------|
| Isaac Sim | Installed and `ISAACSIM_ROOT` environment variable configured |
| reBotArm firmware | Arm firmware flashed, CAN bus connected (`can0`) |
| CAN interface | `can0` is up with a bitrate of 1 Mbps (`can_restart can0`) |
| Python | 3.10+ |
| uv | Recommended for managing Python environments |
| reBotArm_control_py | git submodule has been `update --init`, and `uv sync` has been run in that directory |

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

This repo references the upstream control library as a git submodule. Fetch it first, then install dependencies:

```bash
git clone --recurse-submodules <this-repo>
# already cloned:
git submodule update --init --recursive

cd third_party/reBotArm_control_py
uv sync
```

### 3. Switch the hardware config to RS

This repo's Isaac Sim asset is RS (`usd/ReBot_Arm_RS`). The current SDK defaults `rebotarm.yaml` to RS; verify the actual configuration first. Both `RebotArm()` and `load_robot_model()` follow this file, so gravity compensation, joint-reader, IK, and Traj all need RS first; leave it on DM and the motor protocol will not match, and Pinocchio will load the DM URDF. This only dirties the submodule working tree — do not commit it:

```bash
cd reBotArm_Isaacsim
python set_hw_rs.py
```

On success it prints `.../config/rebotarm.yaml -> rebotarm_rs.yaml`.

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
python set_hw_rs.py
./run_sender.sh
```

**Expected behavior:**
- `set_hw_rs.py` points the submodule `rebotarm.yaml` at `rebotarm_rs.yaml` so motors and the gravity model share one YAML (local change; do not commit)
- The physical arm connects and starts upstream `GravityCompensation` (same MIT + `g(q)` feed-forward as `example/9`)
- The arm can be moved freely by hand
- This script only forwards joint angles to Isaac Sim over UDP at 60 Hz
- Do not also run upstream `example/9` at the same time; the two processes would contend for CAN

#### ② Inverse Kinematics Mode (`isaacsim_ik_sender`)

Input end-effector pose (position/orientation), solve via IK and drive the Isaac Sim arm. `load_robot_model()` reads the submodule `rebotarm.yaml`, so switch to RS first:

```bash
cd reBotArm_Isaacsim
python set_hw_rs.py
uv run python isaacsim_ik_sender.py
```

**Input format (one per line):**
```
x y z                       # position (m), keep last joint target orientation
x y z r p y                 # position + orientation (m/deg)
q j1 j2 j3 j4 j5 j6         # direct joint angles (deg)
gripper <0~1>                # set ratio; sent with the next joint command
```

The sender starts with a zero joint target; it does not automatically read the simulator pose. A `q` or successful IK command updates that target. Failed IK results are not sent.

#### ③ Trajectory Planning Mode (`isaacsim_traj_sender`)

IK plus joint-space trajectory planning (MIN_JERK) for smooth motion. It also uses `load_robot_model()` against that YAML, so switch to RS first:

```bash
cd reBotArm_Isaacsim
python set_hw_rs.py
uv run python isaacsim_traj_sender.py
```

**Input format (one per line):**
```
x y z                       # position (m)
x y z r p y                 # position + orientation (m/deg)
q j1 j2 j3 j4 j5 j6         # direct joint-space target (deg)
gripper <0~1>                # update gripper only
speed <scale>                # speed scale 0.1–5.0; higher is faster
resync                       # read measured joints (older receiver returns targets)
```

At startup the sender requests joint feedback. If unavailable, motion commands are blocked until `resync` succeeds. The updated receiver provides measured simulator joints; older versions provide the last target. Failed IK or out-of-limit joints are not sent.

#### ④ Joint Test Mode (`isaacsim_joint_test_sender`)

No hardware required; preset trajectory loop to verify communication and Isaac Sim receiver:

```bash
cd reBotArm_Isaacsim
uv run python isaacsim_joint_test_sender.py
```

The test sender loops through a few preset joint poses with slow interpolation; it does not read the hardware YAML, so neither `set_hw_rs.py` nor CAN is required.

#### ⑤ Real-to-Sim Mapping Mode (`joint_reader_sender`)

Read RS hardware positions and mirror them in Isaac Sim. `RebotArm()` reads the submodule's `rebotarm.yaml`; select RS first:

```bash
cd reBotArm_Isaacsim
python set_hw_rs.py
uv run python joint_reader_sender.py
```

- `set_hw_rs.py` changes the local motor configuration; do not commit that local change.
- Sends RS position queries, but no enable, disable, mode-change or motion commands. Exiting does not disable motors.
- The UDP target rate is 60 Hz, subject to CAN bus load. Sending pauses on missing or invalid motor feedback and resumes when it recovers.
- The default CAN host ID is `0xfc`; use e.g. `--can-host-id 0xfb` if another process uses it. Check bus load and compatibility before using it alongside a hardware controller.
- The default gripper mapping is 0 rad closed to 5 rad open, corresponding to 0–45 mm per simulated finger. `--gripper-closed` and `--gripper-open` optionally override the display mapping only; they do not change motor zero or motion limits.

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
| `gripper_position` | float | Gripper finger position target (m); each sender computes it with its own mapping (see below) |

**Gripper control chain:**
The receiver applies the received `gripper_position` directly as the position target of both prismatic finger joints, clipped per finger to `[0, upper limit]` (USD upper limit: 0.05 m on both fingers; a single motor drives both through one pinion, so their travel is rigidly 1:1). There is no extra scaling on the receiver side. The senders map their input to `gripper_position` as follows:

| Sender | Mapping to `gripper_position` (m) |
|------|------|
| `gravity_joint_sender` | `gripper_q × 0.03` (`GRIPPER_POSITION_SCALE = 0.03`) |
| `joint_reader_sender` | `clip((gripper_q - closed) / (open - closed), 0, 1) × 0.045 m` (default `closed=0`, `open=5` rad) |
| `isaacsim_traj_sender` | `ratio × 0.045` (`gripper <0~1>` input, clipped to 0.045 m) |
| `isaacsim_ik_sender` | `ratio × 0.045` m simulated travel per finger, sent with the next joint command |

## Configuration Parameters

### Sender (`gravity_joint_sender.py`)

| Parameter | Default | Description |
|------|--------|------|
| Hardware YAML | `set_hw_rs.py` → `rebotarm_rs.yaml` | `RebotArm()` reads submodule `config/rebotarm.yaml`; motors and Pinocchio share it |
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
| `ROBOT_PRIM_PATH` | `/World/reBotArm` | Robot Prim path inside Isaac Sim |
| `ASSET_RELATIVE_PATH` | `usd/ReBot_Arm_RS/ReBot_Arm_RS.usda` | USD asset path relative to the repo root |

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
ls usd/ReBot_Arm_RS/ReBot_Arm_RS.usda
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
