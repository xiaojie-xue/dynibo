# 参考系与空间向量

参考系约定是 API contract 的一部分。即使一个向量的数值正确，如果使用了错误的
参考系或作用点，结果仍然是错误的。

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

运动学速度和加速度结果在世界坐标系下表达，作用点是函数文档指定的目标原点或工具点。
Tool pose 相对于目标 link frame 定义，用于选择另一个刚性连接点。

## Wrench 与载荷

Wrench 使用 torque 在前的顺序：

```text
[torque_x, torque_y, torque_z, force_x, force_y, force_z]
```

外部载荷作用于目标 link 原点，并在该 link 的局部坐标系下表达。符号和所有权规则见
[外部载荷](external-loads.md)。

## 矩阵布局

雅可比矩阵尺寸为 `6 x G`，质量矩阵尺寸为 `G x G`。所有语言接口的一维矩阵 buffer
均使用 column-major 存储。对于行数为 `rows` 的矩阵：

```text
values[column * rows + row]
```

雅可比矩阵中，每个连续的六元素列表示一个 angular-first 空间运动响应。将结果传给
row-major 线性代数库时应显式转换。
