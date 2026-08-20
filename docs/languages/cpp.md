# C++ Guide

The C++17 interface is a header-only RAII wrapper over dynibo's stable C ABI.
Include `<dynibo/dynibo.hpp>` and link the native library through the installed
CMake target:

```cmake
find_package(dynibo CONFIG REQUIRED)
target_compile_features(my_robot PRIVATE cxx_std_17)
target_link_libraries(my_robot PRIVATE dynibo::dynibo)
```

## Ownership and errors

`dynibo::Robot` owns both a native robot handle and its reusable workspace. It
cannot be copied, but it can be moved. Its destructor releases both handles.
Failures are reported as `dynibo::Error`:

```cpp
try {
    dynibo::Robot robot("robot.urdf", DYNIBO_BASE_FLOATING);
    // Use robot...
} catch (const dynibo::Error& error) {
    std::cerr << error.what() << '\n';
    std::cerr << "status: " << error.status() << '\n';
}
```

One `Robot` contains one mutable workspace. Do not invoke calculation methods
concurrently on the same object. Use a separate `Robot` per parallel worker.

## Value types

The wrapper intentionally reuses ABI-compatible C value types:

| Meaning | Type |
|---|---|
| Pose | `DyniboPose` |
| Spatial motion | `DyniboTwist` |
| External load | `DyniboLoad` |
| IK settings | `DyniboIkOptions` |
| Base mode | `DyniboBaseMode` |

Matrix operations return flat `std::vector<double>` values in column-major
order. See [Frames and Spatial Vectors](../user-guide/frames-and-spatial-vectors.md).

## Native interoperation

`native_handle()` and `workspace_handle()` are escape hatches for calling the C
API. The returned pointers are borrowed: do not destroy them and do not retain
them after the `dynibo::Robot` is moved or destroyed.

[Open the C++ API reference](../cpp-api/dynibo_8hpp.md){ .md-button }
