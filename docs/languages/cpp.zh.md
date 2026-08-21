# C++ 指南

C++17 接口是建立在 dynibo 稳定 C ABI 之上的 header-only RAII 封装。包含
`<dynibo/dynibo.hpp>`，并通过安装后的 CMake target 链接原生库：

```cmake
find_package(dynibo CONFIG REQUIRED)
target_compile_features(my_robot PRIVATE cxx_std_17)
target_link_libraries(my_robot PRIVATE dynibo::dynibo)
```

## 所有权与错误

`dynibo::Robot` 同时持有原生 robot handle 和可复用 workspace。对象不可复制，
但可以移动；析构函数会释放两个 handle。失败通过 `dynibo::Error` 报告：

```cpp
try {
    dynibo::Robot robot("robot.urdf", DYNIBO_BASE_FLOATING);
    // 使用 robot……
} catch (const dynibo::Error& error) {
    std::cerr << error.what() << '\n';
    std::cerr << "status: " << error.status() << '\n';
}
```

每个 `Robot` 只有一个可变 workspace，因此不要在同一对象上并发调用计算方法。
每个并行 worker 应使用独立的 `Robot`。

## 值类型

当前封装直接复用与 ABI 兼容的 C 值类型：

| 含义 | 类型 |
|---|---|
| 位姿 | `DyniboPose` |
| 空间运动 | `DyniboTwist` |
| 外部载荷 | `DyniboLoad` |
| IK 配置 | `DyniboIkOptions` |
| 基座模式 | `DyniboBaseMode` |

矩阵操作返回 column-major 顺序的一维 `std::vector<double>`，详见
[参考系与空间向量](../user-guide/frames-and-spatial-vectors.md)。

## 与 C API 互操作

`native_handle()` 和 `workspace_handle()` 是调用 C API 的高级入口。返回的指针只是
借用：不要销毁它们，也不要在 `dynibo::Robot` 被移动或销毁后继续持有。

[打开 C++ API 参考](../cpp-api/dynibo_8hpp.md){ .md-button }
