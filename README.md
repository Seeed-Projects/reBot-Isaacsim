# reBot-Isaacsim

reBotArm 真实机械臂与 NVIDIA Isaac Sim 仿真环境的实时镜像系统。通过 UDP JSON 将真实机械臂的关节角（支持重力补偿手动掰动）与夹爪状态实时同步到 Isaac Sim 仿真场景。

## 系统架构

```
┌──────────────────────────────────────────────────────────────────┐
│                         reBot-Isaacsim                           │
│                                                                  │
│   ┌─────────────────────┐         ┌─────────────────────────┐   │
│   │ 发送端 (Terminal 1)  │  UDP   │   接收端 (Terminal 2)    │   │
│   │                     │  JSON  │                         │   │
│   │ gravity_joint_sender │──────▶│ isaacsim_joint_receiver  │   │
│   │                     │ 5005   │                         │   │
│   │  • reBotArm_control  │        │  • Isaac Sim 仿真        │   │
│   │    _py uv 环境        │        │  • 地面 + 机械臂 USD    │   │
│   │  • MIT + 重力前馈     │        │  • 关节角同步           │   │
│   │  • 允许手动掰动       │        │  • 夹爪双关节联动       │   │
│   └─────────────────────┘        └─────────────────────────┘   │
└──────────────────────────────────────────────────────────────────┘
```

## 目录结构

```
reBot-Isaacsim/
├── pyproject.toml                           # uv 工作空间配置
├── README.md
├── reBotArm_Isaacsim/                       # 主示例目录
│   ├── gravity_joint_sender.py              # 真实机械臂发送端（重力补偿 + UDP）
│   ├── isaacsim_joint_receiver.py           # Isaac Sim 接收端（关节角同步）
│   ├── isaacsim_joint_test_sender.py        # 测试发送端（无需硬件，预设轨迹）
│   ├── live_sync.py                         # 启动说明脚本
│   ├── run_sender.sh                        # 启动发送端
│   └── run_isaacsim_receiver.sh             # 启动 Isaac Sim 接收端
├── third_party/
│   └── reBotArm_control_py/                 # 核心控制库（独立 uv 环境）
│       ├── pyproject.toml
│       └── ...
└── usd/
    └── RS-rebot-dev-arm/
        └── 00-arm-rs_asm-v3.usda            # Isaac Sim 机械臂资产
```

## 依赖与前提条件

| 组件 | 要求 |
|------|------|
| Isaac Sim | 已安装并配置 `ISAACSIM_ROOT` 环境变量 |
| reBotArm 固件 | 机械臂固件已烧录，CAN 总线已连接（`can0`） |
| CAN 接口 | `can0` 已 up 且 bitrate 为 1 Mbps（`can_restart can0`） |
| Python | 3.10+ |
| uv | 推荐使用 uv 管理 Python 环境 |
| reBotArm_control_py | 已在 `third_party/reBotArm_control_py` 中运行 `uv sync` |

### 检查 CAN 接口

```bash
ip link show can0
# 确保状态为 UP，bitrate 1000000

# 如需重启 CAN：
can_restart can0
# 或
sudo ip link set can0 down && sudo ip link set can0 up type can bitrate 1000000 restart-ms 100
```

## 环境准备

### 1. Isaac Sim 环境变量

确保 `.bashrc` 或 shell 配置中已设置：

```bash
export ISAACSIM_ROOT=/home/seeed/IsaacSim/_build/linux-x86_64/release
```

### 2. reBotArm_control_py 环境

```bash
cd third_party/reBotArm_control_py
uv sync
```

## 启动（双终端模式）

需要两个独立终端：

### 终端 1 — 启动 Isaac Sim 接收端

```bash
cd reBotArm_Isaacsim
./run_isaacsim_receiver.sh
```

**预期输出：**
- 启动 Isaac Sim 图形界面
- 加载地面和机械臂 USD 资产
- 监听 UDP `127.0.0.1:5005`
- 等待发送端连接

### 终端 2 — 启动发送端（真实机械臂）

```bash
cd reBotArm_Isaacsim
./run_sender.sh
```

**预期行为：**
- 连接真实机械臂，启用重力补偿模式
- 机械臂可自由掰动
- 关节角以 60 Hz 持续通过 UDP 发送

**启动顺序：先接收端，再发送端。**

## 无硬件测试模式

在没有真实机械臂的情况下，使用测试发送端验证 Isaac Sim 接收端：

```bash
# 终端 2 使用测试发送端代替真实发送端
cd reBotArm_Isaacsim
./run_sender.sh --test
# 或直接：
python third_party/reBotArm_control_py/.venv/bin/python isaacsim_joint_test_sender.py
```

测试发送端在几个预设关节姿态之间缓慢插值循环发送，无需 CAN 连接。

## 通信协议

UDP JSON，端口 `127.0.0.1:5005`。

**发送端每帧 Payload：**

```json
{
  "sequence": 123,
  "timestamp": 1718000000.123,
  "joint_positions": [0.0, 0.1, 0.2, -0.1, 0.0, -0.02],
  "gripper_position": 0.05
}
```

| 字段 | 类型 | 说明 |
|------|------|------|
| `sequence` | int | 递增序号 |
| `timestamp` | float | Unix 时间戳（秒） |
| `joint_positions` | float[6] | 前 6 个关节角（rad） |
| `gripper_position` | float | 夹爪位置（m），由发送端通过 `GRIPPER_POSITION_SCALE=0.03` 转换 |

**夹爪控制链：**
发送端 `gripper_q` → `gripper_position = -gripper_q × 0.03` → 接收端 `× 0.01` → 双关节位置目标

## 配置参数

### 发送端 (`gravity_joint_sender.py`)

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `ARM_JOINT_COUNT` | 6 | 关节数 |
| `DEFAULT_PORT` | 5005 | UDP 端口 |
| `DEFAULT_SEND_HZ` | 60.0 | 发送频率（Hz） |
| `GRIPPER_POSITION_SCALE` | 0.03 | 夹爪角到位置的缩放系数 |
| `position_alpha` | 0.2 | 低通滤波系数 |

### 接收端 (`isaacsim_joint_receiver.py`)

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `ARM_JOINT_COUNT` | 6 | 关节数 |
| `DEFAULT_PORT` | 5005 | UDP 端口 |
| `DEFAULT_RENDER_HZ` | 120.0 | 仿真渲染频率（Hz） |
| `GRIPPER_POSITION_SCALE` | 0.01 | 夹爪位置再缩放系数 |
| `ROBOT_PRIM_PATH` | `/World/reBotArm` | Isaac Sim 中的机械臂 Prim 路径 |
| `ASSET_RELATIVE_PATH` | `usd/RS-rebot-dev-arm/00-arm-rs_asm-v3.usda` | USD 资产相对路径 |

## 常见问题

### `OSError: [Errno 98] Address already in use`

端口 5005 已被占用。先确认并终止占用进程：

```bash
# 查看占用端口的进程
sudo lsof -i :5005

# 终止进程（将 PID 替换为实际值）
kill <PID>
```

### Isaac Sim 资产未找到

确认 USD 资产路径存在，或检查 `REPO_ROOT` 是否正确：

```bash
ls usd/RS-rebot-dev-arm/00-arm-rs_asm-v3.usda
```

### CAN 总线未就绪

确保 CAN 接口 up 且 bitrate 正确：

```bash
can_restart can0
# 验证：
ip -details link show can0 | grep bitrate
```

### 关节角不同步

- 确认发送端和接收端端口一致（均为 5005）
- 检查发送端日志中 `[send]` 是否有持续输出
- 检查接收端日志中 `[recv]` 是否有持续输出
- 尝试使用 `isaacsim_joint_test_sender.py` 排除硬件问题

## 组件与 Python 环境

| 组件 | Python 环境 | 启动脚本 |
|------|------------|---------|
| 发送端（真实机械臂） | `reBotArm_control_py` uv 环境 | `run_sender.sh` |
| 发送端（测试模式） | `reBotArm_control_py` uv 环境 | `isaacsim_joint_test_sender.py` |
| 接收端 | Isaac Sim 官方 Python (`python.sh`) | `run_isaacsim_receiver.sh` |
