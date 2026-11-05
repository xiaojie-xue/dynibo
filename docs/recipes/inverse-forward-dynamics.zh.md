# 逆动力学与正动力学（RNEA / ABA）

先用递归牛顿–欧拉算法（RNEA）计算关节力矩，再用关节体算法（ABA）恢复期望加速度。
示例还验证 `tau = M @ qdd + Cqd + gravity`；该单关节模型的惯量为 0.35 kg·m²。

安装已发布的包，然后保存并运行下面的完整 Python 脚本。代码会在临时目录创建 URDF，
无需克隆仓库、下载 mesh 或安装 ROS。

```bash
python -m pip install dynibo
python inverse_forward_dynamics.py
```

```python
--8<-- "examples/python/recipes/inverse_forward_dynamics.py"
```

## 结果与约定

q、qd 和 qdd 都按非固定关节顺序排列。`forces` 是广义力数组，不是基座与关节位置数组。
验证往返一致性时，逆动力学与正动力学必须传入相同的阻力载荷。

[Python API 参考](../reference/python.md) · [参考系与空间向量](../user-guide/frames-and-spatial-vectors.md)
