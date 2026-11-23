# RS USD 资产

当前资产的质量、质心和惯量与 [SDK 的 RS URDF](../../urdf/RS/urdf/ReBot_Arm_RS.urdf) 一致。模型总质量为 6.5503 kg，每侧手指为 55.9 g，滑动行程各为 0–50 mm。模型来源及许可证见 [RS URDF 说明](../../urdf/RS/README.md)。

从 `urdf/RS/urdf/ReBot_Arm_RS.urdf` 使用 urdf-usd-converter 0.1.0 生成，再由 `scripts/prepare_asset.py` 恢复完整惯量、应用驱动参数并将夹爪碰撞体改为凸分解。新模型的六个关节轴与旧 USD 相反，接收器在 UDP 协议与新资产之间转换符号。设置 `REBOT_ISAAC_USE_LEGACY_USD=1` 可回退到旧资产。

生成后通过了 URDF/USD 质量、质心、惯量、关节轴和限位静态检查，以及 Isaac Sim 无界面 UDP 接收与夹爪限位测试。验证不涉及真机，也不能证明真机重力补偿已校准。旧 URDF 保留给旧 USD、MJCF 和验证工具使用。
