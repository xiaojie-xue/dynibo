# 使用指南

本指南介绍所有 dynibo 语言接口共享的模型和数值约定。它独立于具体语言的 API
reference：Rust、Python、C++ 和 C 使用相同的数学含义、排列顺序、单位和生命周期规则。

## 推荐阅读顺序

1. [机器人模型与 URDF](robot-model-and-urdf.md)：dynibo 会从文件中加载什么。
2. [关节与广义坐标](joint-and-generalized-coordinates.md)：输入和输出尺寸。
3. [参考系与空间向量](frames-and-spatial-vectors.md)：位姿、twist、wrench 和矩阵布局。
4. [固定基座与浮动基座](fixed-and-floating-bases.md)：基座状态如何参与计算。
5. [Workspace 与内存分配](workspaces-and-allocation.md)：复用和并发规则。
6. [运动学](kinematics.md)和[动力学](dynamics.md)：具体计算能力。

施加载荷前请阅读[外部载荷](external-loads.md)；将 dynibo 集成到长时间运行或并发应用前，
请阅读[错误处理与线程安全](errors-and-thread-safety.md)。

不同语言的写法和所有权差异汇总在 [API 对照](../languages/api-mapping.md)中。
