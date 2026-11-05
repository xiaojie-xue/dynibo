# 重力补偿

计算固定基座机械臂抵消重力所需的保持力矩。机械臂质量为 1 kg，质心位于局部 x 轴
0.5 m 处，关节绕 y 轴转动，保持力矩为 `-0.5 * 9.80665 * cos(q)` N·m。
代码同时验证解析力矩，以及静止时施加该力矩会得到零加速度。

安装已发布的包，然后保存并运行下面的完整 Python 脚本。代码会在临时目录创建 URDF，
无需克隆仓库、下载 mesh 或安装 ROS。

```bash
python -m pip install dynibo
python gravity_compensation.py
```

```python
--8<-- "examples/python/recipes/gravity_compensation.py"
```

## 结果与约定

这里计算的是数值保持力矩。执行器接口、增益、限幅、摩擦与接触约束由应用处理。
可选的 `loads` 是作用于 link 原点、以 link 局部坐标表达的阻力 wrench。

[Python API 参考](../reference/python.md) · [参考系与空间向量](../user-guide/frames-and-spatial-vectors.md)
