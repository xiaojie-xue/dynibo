# Python 指南

Python 绑定在 dynibo 原生库之上提供面向对象接口。使用 `python -m pip install dynibo`
安装，并从 `dynibo` 导入公开类型。

## 生命周期与错误

建议将 `Robot` 用作上下文管理器，以便确定性地释放原生资源：

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

关节输入接受 NumPy 数组或数值序列；连续 `float64` 数组在拷贝到原生 Rust 缓冲区执行
释放 GIL 的计算前，无需额外的 dtype 转换或步长重排。`Pose` 和 `Twist` 是不可变的
值对象。向量和矩阵方法返回 `float64` NumPy 数组，矩阵采用列主序的一维布局，转为二维
视图时请使用 `arr.reshape((rows, cols), order="F")`，详见
[参考系与空间向量](../user-guide/frames-and-spatial-vectors.md)。

控制循环可通过 `out=` 复用调用方预先分配的输出缓冲区：

```python
import numpy as np

q = np.zeros(robot.joint_count)
gravity = np.empty(robot.generalized_count)
robot.gravity(q, out=gravity)
```

每个 `Robot` 持有一个原生 workspace，同一实例上的调用会按顺序串行执行。需要并行计算时，
请使用 `fork()` 或独立的机器人实例。

## 浮动基座

`FloatingRobot` 拥有独立 workspace，且不保存可变的基座状态。每次计算都需将
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

浮动基座满足 `generalized_count == joint_count + 6`；广义输出先是世界坐标系下的角分量，
再是线分量。只有固定基座 `Robot` 提供 `set_base_frame()`；`Robot` 和 `FloatingRobot`
均提供 `gravity_vector` 和 `set_gravity()`。

`link_id()` 与 `link_id_at()` 返回绑定到所属模型的 `LinkId`（`int` 子类），既可直接用于
NumPy 数组索引，又能拒绝跨模型误用；传入普通 `int` 仍兼容，但会触发 `DeprecationWarning`。
软件包内置 PEP 561 类型存根（`py.typed` 与 `__init__.pyi`）。

[打开 Python API 参考](../reference/python.md){ .md-button }
