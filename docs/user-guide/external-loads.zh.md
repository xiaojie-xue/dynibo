# 外部载荷

重力、逆动力学和正动力学均支持在 link 原点处施加阻力 wrench。每个载荷由绑定到所属模型的
link ID、力矩（torque）和力（force）分量组成。逆动力学会将该 wrench 加到所需广义力中，
正动力学会从可用广义力中减去它；如果你从“作用在机器人上的物理外力”出发，需取相反符号。

## 参考系与作用点

载荷作用于所选 link 的原点，并在该 link 的局部坐标系下表达。如果力实际作用在偏移
位置，应先将其换算为 link 原点处的等效 wrench。分量使用 torque 在前的顺序。

## 创建载荷

=== "Rust"

    ```rust
    use dynibo::nalgebra::Vector3;
    use dynibo::{IndexedLoad, Wrench};

    let load = IndexedLoad {
        link: tool,
        wrench: Wrench::new(
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(0.0, 0.0, -10.0),
        ),
    };
    ```

=== "Python"

    ```python
    from dynibo import Load

    load = Load(
        link_id=tool,
        torque=(0.0, 0.0, 0.0),
        force=(0.0, 0.0, -10.0),
    )
    gravity = robot.gravity(q, [load])
    ```

=== "C++"

    ```cpp
    const DyniboLoad load =
        robot.make_load(tool, {0.0, 0.0, 0.0}, {0.0, 0.0, -10.0});
    const auto gravity = robot.gravity(q, {load});
    ```

=== "C"

    ```c
    const DyniboLoad load = {
        .link_id = tool,
        .torque = {0.0, 0.0, 0.0},
        .force = {0.0, 0.0, -10.0},
    };
    check(dynibo_gravity(
        robot, workspace, q, J, &load, 1, output, G));
    ```

所有 link ID 必须由参与计算的 robot 生成。多个载荷可以作用于相同或不同 link，
dynibo 会累加它们的影响。

## 复用载荷存储

Rust 和 Python robot 提供 `load_buffer()`，创建时为每个 link 预留一个载荷的容量。
`set` 替换载荷，`add` 累加载荷，`remove` 和 `clear` 保留容量。更新失败时原有
载荷保持不变。Rust 向动力学方法传入 `buffer.as_slice()`；Python 则直接传入该缓冲区对象：

```python
loads = robot.load_buffer()
loads.set(tool, force=(0.0, 0.0, -10.0))
robot.gravity(q, loads=loads, out=output)
loads.clear()
```

Python `LoadBuffer` 归属于创建它的模型，可供该模型的 `fork()` 实例共享；独立加载的
机器人实例会拒绝它（即使缓冲区为空）。原生计算期间会借用缓冲区，同时拒绝并发修改。
你也可以直接传入 `Load` 序列，但每次调用都会重新转换。

## 无载荷调用

Rust、Python 和 C++ 可以直接传入空集合。C 只有在 `load_count` 为零时才可将载荷指针
设为 `NULL`。载荷数组由调用方持有，dynibo 不会在调用结束后保留引用。
