# 从 URDF 计算正运动学

从 URDF 加载机器人，根据关节角计算工具位姿。示例中的单关节机械臂绕 y 轴旋转，
工具距离关节一米，因此世界坐标中的平移为 `(cos(q), 0, -sin(q))`。代码会验证解析结果。

安装已发布的包，然后保存并运行下面的完整 Python 脚本。代码会在临时目录创建 URDF，
无需克隆仓库、下载 mesh 或安装 ROS。

```bash
python -m pip install dynibo
python forward_kinematics.py
```

```python
--8<-- "examples/python/recipes/forward_kinematics.py"
```

## 结果与约定

`Pose.rotation_xyzw` 的四元数顺序为 `(x, y, z, w)`。位姿把 link 局部坐标转换到世界坐标。
关节顺序是 URDF 中非固定关节的广度优先顺序，不是名称排序。

[Python API 参考](../reference/python.md) · [参考系与空间向量](../user-guide/frames-and-spatial-vectors.md)
