# RS USD 资产

源模型：[Yang-Ci/reBot-DevArm](https://github.com/Yang-Ci/reBot-DevArm/tree/f33fa5feb56ef16ee292ff6db28d9aff9e1d8d0c/Rebot_Arm_description/RS)，提交 `f33fa5feb56ef16ee292ff6db28d9aff9e1d8d0c`；与 SDK 的 RS URDF 动力学参数一致。两侧夹爪上限均为 0.05 m。模型及网格许可证见 `urdf/RS/UPSTREAM_LICENSE`。

从 `urdf/RS/urdf/ReBot_Arm_RS.urdf` 使用 urdf-usd-converter 0.1.0 生成，再由 `scripts/prepare_asset.py` 恢复完整惯量、应用驱动参数并将夹爪碰撞体改为凸分解。新模型的六个关节轴与旧 USD 相反，接收器在 UDP 协议与新资产之间转换符号。设置 `REBOT_ISAAC_USE_LEGACY_USD=1` 可回退到旧资产。

生成后通过了 URDF/USD 质量、质心、惯量、关节轴和限位静态检查，以及 Isaac Sim 无界面 UDP 接收与夹爪限位测试。验证不涉及真机，也不能证明真机重力补偿已校准。旧 URDF 保留给旧 USD、MJCF 和验证工具使用。
