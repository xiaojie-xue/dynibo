# Python 指南

Python binding 在 dynibo 原生库之上提供面向对象接口。使用
`python -m pip install dynibo` 安装，并从 `dynibo` 导入公开类型。

## 生命周期与错误

建议将 `Robot` 用作 context manager，以便确定性地释放原生资源：

```python
from dynibo import DyniboError, Robot

try:
    with Robot.from_urdf("robot.urdf") as robot:
        print(robot.name)
except DyniboError as error:
    print(f"dynibo: {error}")
```

无效模型输入会抛出 `ModelError`，数值求解器失败会抛出 `SolverError`，原生边界
捕获到 panic 时会抛出 `PanicError`。这些异常都继承自 `DyniboError`。

## 数组与结果

关节输入接受由数字组成的 Python sequence。Pose 和 twist 是不可变的值对象。
矩阵方法返回使用 column-major 顺序的一维 tuple，详见
[参考系与空间向量](../user-guide/frames-and-spatial-vectors.md)。

每个 `Robot` 持有一个原生 workspace，同一实例上的调用会被串行化。需要并行计算时，
请使用相互独立的 robot 实例。

[打开 Python API 参考](../reference/python.md){ .md-button }
