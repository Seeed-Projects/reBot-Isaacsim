# 新版 RS USD（本地测试）

源模型：[Yang-Ci/reBot-DevArm](https://github.com/Yang-Ci/reBot-DevArm/tree/ae9a098abeaa2e363f6b1b7fe2843317bb09f9a5/Rebot_Arm_description/RS)，提交 `ae9a098abeaa2e363f6b1b7fe2843317bb09f9a5`。本地将两侧夹爪上限统一为 0.05 m，并同步更新 USD。模型及网格许可证见 `urdf/RS/UPSTREAM_LICENSE`。

从 `urdf/RS/urdf/ReBot_Arm_RS.urdf` 使用本机 urdf_usd_converter 0.1.0 生成。`scripts/prepare_asset.py` 根据 URDF 恢复完整惯量、使用现有资产的驱动参数并将夹爪碰撞体改为凸分解；参数仅经仿真验证。新模型的六个关节轴与旧 USD 相反，接收器在 UDP 协议与新资产之间转换符号。设置 `REBOT_ISAAC_USE_LEGACY_USD=1` 可回退到旧资产。

2026-11-06：URDF 和 USD 静态质量、质心、惯量、关节轴和框架检查通过；独立 PhysX 保持测试和模拟 UDP 接收测试通过。仅测试了有限姿态和载荷，不代表真机重力补偿已校准。旧 URDF 仍供旧 USD、MJCF 和验证工具使用。
