# 外部载荷

重力、逆动力学和正动力学可以包含 link 原点处的阻力 wrench。每个载荷由模型局部
link ID、torque 和 force 分量组成。逆动力学会把该 wrench 加到所需广义力中，正动力学
会从可用广义力中减去它；如果输入的是实际作用于机器人的物理外力，应使用相反符号。

## 参考系与作用点

载荷作用于所选 link 的原点，并在该 link 的局部坐标系下表达。如果力实际作用在偏移
位置，应先将其换算为 link 原点处的等效 wrench。分量使用 torque 在前的顺序。

## 创建载荷

=== "Rust"

    ```rust
    use dynibo::{IndexedLoad, Wrench};
    use nalgebra::Vector3;

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
    DyniboLoad load{
        tool,
        {0.0, 0.0, 0.0},
        {0.0, 0.0, -10.0},
    };
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
载荷保持不变。Rust 向动力学方法传入 `buffer.as_slice()`；Python 直接传入 buffer：

```python
loads = robot.load_buffer()
loads.set(tool, force=(0.0, 0.0, -10.0))
robot.gravity(q, loads=loads, out=output)
loads.clear()
```

Python `LoadBuffer` 属于创建它的模型。独立加载的 robot
会拒绝它，即使 buffer 为空。原生计算期间会借用 buffer，并拒绝并发修改。
原有 `Load` 列表接口仍然支持，列表会在每次调用时转换。

## 无载荷调用

Rust、Python 和 C++ 可以传入空 collection。C 只有在 `load_count` 为零时才可以将
载荷指针设为 `NULL`。载荷数组由调用方持有，dynibo 不会在调用结束后保存它。
