# 浮动基座动力学

加载浮动基座模型，通过 `BaseState` 显式传入位姿、速度和加速度。关节数组长度仍为 J；
广义力与加速度长度为 G = J + 6，依次为世界坐标中的基座角分量、线分量和关节分量。
示例先验证 RNEA/ABA 的往返一致性，再将基座 wrench 置零计算无驱动的自由飞行。

安装已发布的包，然后保存并运行下面的完整 Python 脚本。代码会在临时目录创建 URDF，
无需克隆仓库、下载 mesh 或安装 ROS。

```bash
python -m pip install dynibo
python floating_base.py
```

```python
--8<-- "examples/python/recipes/floating_base.py"
```

## 结果与约定

根 link 必须有严格正质量，关节体惯量还必须可求解。前六个广义力是基座 wrench，
不是电机力矩。正动力学忽略 `base.acceleration`，因为加速度是待求结果。
该示例不包含地面接触或平衡控制器。

[Python API 参考](../reference/python.md) · [参考系与空间向量](../user-guide/frames-and-spatial-vectors.md)
