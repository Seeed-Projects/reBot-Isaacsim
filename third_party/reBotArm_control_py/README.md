# reBotArm Control Python

## 快速启动（双进程）

两个脚本需要**分别在两个终端**中启动，先启动接收端，再启动发送端：

**终端 1 — Isaac Sim 接收端：**

```bash
/home/seeed/IsaacSim/_build/linux-x86_64/release/python.sh \
    example/11b_isaacsim_joint_receiver.py
```

**终端 2 — 机械臂发送端：**

```bash
uv run python example/11a_gravity_joint_sender.py
```

运行后预期行为：真实机械臂进入重力补偿可手动掰动，Isaac Sim 中的仿真机械臂实时跟随。

---

## Isaac Sim 实时镜像示例

当前仓库已将 Isaac Sim 实时镜像改为双进程方案：

- `example/11a_gravity_joint_sender.py`
  - 在当前工程 `uv` 环境中运行
  - 负责连接真实机械臂、启动重力补偿、读取前 6 个关节角
  - 通过 UDP JSON 持续发送关节角数据
- `example/11b_isaacsim_joint_receiver.py`
  - 使用 Isaac 官方 `python.sh` 运行
  - 负责启动 Isaac Sim、加入地面、加载 `config/RS-rebot-dev-arm/00-arm-rs_asm-v3.usda`
  - 通过 UDP 接收关节角并驱动仿真机械臂同步
- `example/11_isaacsim_live_sync.py`
  - 保留为入口说明脚本
  - 用于提示新的 sender / receiver 启动方式

## 为什么改成双进程

当前工程依赖 `uv` 环境中的 `motorbridge`，而新的 Isaac Sim release 使用自己的一套 Python 3.12 运行时。

仓库里保留了一个探索性桥接脚本：`scripts/run_with_isaacsim_uv.sh`。它会：

- 使用当前工程 `.venv` 里的 Python 和 `motorbridge`
- 注入 Isaac Sim release 目录中的运行时环境变量
- 尝试直接启动脚本，而不依赖 Isaac Sim 自带 `python.sh`

但该桥接方式目前只能完成 Python 包层面的桥接，`SimulationApp` 的底层原生运行时仍以 Isaac 官方 3.12 解释器最稳定。因此更稳妥的方案是将硬件控制与仿真拆到两个独立进程中。

## 前提条件

请先完成 Isaac Sim 源码构建，并确保以下目录存在：

- `/home/seeed/IsaacSim/_build/linux-x86_64/release`

该目录下至少应包含：

- `setup_conda_env.sh`
- `setup_python_env.sh`
- `isaac-sim.sh`
- `python.sh`

## 推荐运行方式

先启动 Isaac Sim 接收端：

```bash
/home/seeed/IsaacSim/_build/linux-x86_64/release/python.sh \
example/11b_isaacsim_joint_receiver.py
```

再启动真实机械臂发送端：

```bash
uv run python example/11a_gravity_joint_sender.py
```

运行后预期行为：

- 真实机械臂进入重力补偿，可由用户手动掰动
- 发送端持续发送前 6 个关节角
- Isaac Sim 接收端中的机械臂实时跟随

## 探索性桥接命令

如果你只是想继续验证旧的桥接脚本，可在项目根目录执行：

```bash
./scripts/run_with_isaacsim_uv.sh example/11_isaacsim_live_sync.py
```

如果 Isaac Sim release 不在默认路径，可显式指定真实路径，例如：

```bash
ISAACSIM_ROOT=/home/seeed/IsaacSim/_build/linux-x86_64/release \
./scripts/run_with_isaacsim_uv.sh example/11_isaacsim_live_sync.py
```

注意：当前 `11_isaacsim_live_sync.py` 已不再负责真正启动一体化流程，它只会提示你使用新的双进程脚本。

## 注意事项

- 当前 `uv` 环境是 Python 3.10，而新 Isaac Sim 已迁移到 Python 3.12；这正是两边不能直接共用一个解释器的根因。
- 默认使用本机回环地址 `127.0.0.1:5005` 传输关节角数据。
- 发送端会使能全部关节并启动重力补偿，请务必在安全环境下运行。
