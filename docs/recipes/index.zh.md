# 任务示例

这些完整示例按任务组织，可直接使用已安装的 Python 包运行。每个脚本都内嵌
同一个小型 URDF，并用解析结果或数值一致性验证输出。

- [从 URDF 计算正运动学](forward-kinematics.md)
- [雅可比与 NumPy 布局](jacobian.md)
- [重力补偿](gravity-compensation.md)
- [逆动力学与正动力学（RNEA / ABA）](inverse-forward-dynamics.md)
- [浮动基座动力学](floating-base.md)
- [复用输出缓冲区](reuse-buffers.md)

Rust 用户可查看 [crate 入门示例](../languages/rust.md) 和仓库中的
[`recipes` 示例](https://github.com/xiaojie-xue/dynibo/blob/main/examples/rust/recipes.rs)，
用 `cargo run --example recipes` 运行相同的六类计算。
