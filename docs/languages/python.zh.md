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

关节输入接受 NumPy 数组或由数字组成的 Python sequence；连续 `float64` 数组走
零拷贝路径。Pose 和 twist 是不可变的值对象。向量和矩阵方法返回 `float64`
NumPy 数组，矩阵仍使用 column-major 顺序的一维布局，详见
[参考系与空间向量](../user-guide/frames-and-spatial-vectors.md)。

控制循环可以通过 `out=` 复用调用方分配的存储：

```python
import numpy as np

q = np.zeros(robot.joint_count)
gravity = np.empty(robot.generalized_count)
robot.gravity(q, out=gravity)
```

每个 `Robot` 持有一个原生 workspace，同一实例上的调用会被串行化。需要并行计算时，
请使用相互独立的 robot 实例。

## 浮动基

`FloatingRobot` 拥有独立 workspace，且不保存可变的基座状态。每次计算都将
`BaseState` 作为第一个参数传入：

```python
from dynibo import BaseState, FloatingRobot, Pose

with FloatingRobot.from_urdf("robot.urdf") as robot:
    target = robot.link_id("tool")
    q = [0.0] * robot.joint_count
    base = BaseState(frame=Pose(translation=(0.1, 0.0, 0.0)))
    pose = robot.forward_kinematics(base, q, target)
    mass = robot.mass_matrix(base, q)
```

浮动基满足 `generalized_count == joint_count + 6`；广义输出先是世界坐标系下的
角分量，再是线分量。只有固定 `Robot` 提供 `set_base_frame()`。

[打开 Python API 参考](../reference/python.md){ .md-button }
