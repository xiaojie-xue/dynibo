# 雅可比与 NumPy 布局

计算世界坐标中的几何雅可比，并正确转换为 NumPy 矩阵。第 0–2 行是角速度，
第 3–5 行是目标原点的线速度。示例用正运动学的中心有限差分验证线速度块。

安装已发布的包，然后保存并运行下面的完整 Python 脚本。代码会在临时目录创建 URDF，
无需克隆仓库、下载 mesh 或安装 ROS。

```bash
python -m pip install dynibo
python jacobian.py
```

```python
--8<-- "examples/python/recipes/jacobian.py"
```

## 结果与约定

固定与浮动基座都使用 `reshape((6, robot.generalized_count), order="F")`。
`forward_kinematics_all` 返回连续的七元素位姿记录，应使用默认顺序 `reshape((-1, 7))`。

[Python API 参考](../reference/python.md) · [参考系与空间向量](../user-guide/frames-and-spatial-vectors.md)
