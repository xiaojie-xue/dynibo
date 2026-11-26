# 参考系与空间向量

参考系约定是 API 契约的重要组成部分。即使数值本身计算无误，如果放错了参考系或作用点，
最终结果仍然是错的。

## 位姿

位姿包含以米为单位的平移和单位四元数。C、C++ 和 Python 的四元数系数顺序为
`(x, y, z, w)`；Rust 通过 `Frame` 使用 nalgebra 的 `Isometry3<f64>` 表示。

正运动学返回目标 link 在世界坐标系下的位姿：

$$
{}^W T_{target}(q) = {}^W T_{base}
\prod_{i \in path} {}^{i-1}T_i(q_i).
$$

## Twist 与加速度

空间运动向量使用角分量在前的顺序：

```text
[angular_x, angular_y, angular_z, linear_x, linear_y, linear_z]
```

运动学速度和加速度结果均为**经典速度**与**经典加速度**（而非 Featherstone 空间加速度，
后者在线加速度项上相差 $\omega \times v$），在世界坐标系下表达，作用点是函数文档指定的
目标原点或工具点。Tool pose 相对于目标 link frame 定义，用于选择另一个刚性连接点。

## Wrench 与载荷

Wrench 使用 torque 在前的顺序：

```text
[torque_x, torque_y, torque_z, force_x, force_y, force_z]
```

外部载荷作用于目标 link 原点，并在该 link 的局部坐标系下表达。符号和所有权规则见
[外部载荷](external-loads.md)。

## 矩阵布局

雅可比矩阵尺寸为 `6 x G`，质量矩阵尺寸为 `G x G`。所有语言接口的一维矩阵缓冲区
均采用列主序（column-major）存储。对于行数为 `rows` 的矩阵：

```text
values[column * rows + row]
```

雅可比矩阵中，每个连续的六元素列表示角分量在前的空间运动响应。在 Python/NumPy
中将一维矩阵转为二维视图时应指定 `order="F"`（例如
`J.reshape((6, robot.generalized_count), order="F")`）。将结果传给行主序（row-major）
线性代数库时应显式转换。
