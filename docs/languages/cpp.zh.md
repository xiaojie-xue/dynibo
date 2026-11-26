# C++ 指南

C++17 接口是建立在 dynibo 稳定 C ABI 之上的 header-only RAII 封装。包含
`<dynibo/dynibo.hpp>`，并通过安装后的 CMake target 链接原生库：

```cmake
find_package(dynibo CONFIG REQUIRED)
target_compile_features(my_robot PRIVATE cxx_std_17)
target_link_libraries(my_robot PRIVATE dynibo::dynibo)
```

## 所有权与错误

`dynibo::Robot` 同时持有原生机器人句柄和可复用 workspace。对象不可复制，但可以移动；
析构函数会自动释放两个句柄。失败通过 `dynibo::Error` 异常报告：

```cpp
try {
    dynibo::FloatingRobot robot("robot.urdf");
    // 使用 robot……
} catch (const dynibo::Error& error) {
    std::cerr << error.what() << '\n';
    std::cerr << "status: " << error.status() << '\n';
}
```

每个 `Robot` 只包含一个可变 workspace，因此不要在同一对象上并发调用计算方法。
每个并行工作线程应使用独立的 `Robot` 实例。

## 值类型

当前封装直接复用与 ABI 兼容的 C 值类型：

| 含义 | 类型 |
|---|---|
| Link 标识 | `dynibo::LinkId`（绑定到所属模型，提供 `.index()`；可传给 `robot.make_load(...)`） |
| 位姿 | `DyniboPose` |
| 空间运动 | `DyniboTwist` |
| 外部载荷 | `DyniboLoad` |
| IK 配置 | `DyniboIkOptions` |
| 浮动基座状态 | 每次计算传入 `dynibo::BaseState` |

`dynibo::Robot` 和 `dynibo::FloatingRobot` 均提供 `set_gravity()` 与
`gravity_vector()`。矩阵操作返回列主序的一维 `std::vector<double>`；如需避免循环内
分配内存，可使用 `*_into` 重载（如 `jacobian_into`、`mass_matrix_into`、
`inverse_dynamics_into`、`forward_dynamics_into`、`forward_kinematics_all_into`）
写入尺寸精确匹配的预分配缓冲区，详见[参考系与空间向量](../user-guide/frames-and-spatial-vectors.md)。

## 与 C API 互操作

`native_handle()` 和 `workspace_handle()` 提供直接调用底层 C API 的入口。返回的指针
仅为借用：不要手动销毁它们，也不要在 `dynibo::Robot` 被移动或销毁后继续使用。

[打开 C++ API 参考](../cpp-api/dynibo_8hpp.md){ .md-button }
