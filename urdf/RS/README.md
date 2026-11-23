# RS 源模型

当前 RS URDF 同步自 [Seeed-Projects/reBot-DevArm](https://github.com/Seeed-Projects/reBot-DevArm/tree/103357f3a3d1e45e65c81b45ebf30607e5fa4c6b/Rebot_Arm_description/RS)。网格来自 Yang-Ci/reBot-DevArm 提交 `f33fa5feb56ef16ee292ff6db28d9aff9e1d8d0c`。完整许可证见 `UPSTREAM_LICENSE`。

当前 RS URDF 使用实测质量和质心。模型总质量为 6.5503 kg，每侧夹爪手指为 55.9 g，滑动行程各为 0–50 mm。 每侧 55.9 g 由 43.9 g 金属件及 12.0 g 3D 打印件组成。Pinocchio 中包含六个旋转关节与两个夹爪滑动关节，`nq = nv = 8`。 [URDF 源文件](urdf/ReBot_Arm_RS.urdf)

该 URDF 已用于生成 `usd/ReBot_Arm_RS`；旧目录仍由旧资产与 MJCF 的校验工具引用。
