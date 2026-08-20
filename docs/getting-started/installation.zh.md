# 安装

请根据应用使用的语言选择对应的软件包。

## Rust

从 crates.io 添加 dynibo：

```bash
cargo add dynibo
```

支持的 Rust 版本和 crate features 可以在
[Rust API 参考](https://docs.rs/dynibo)中查看。

## Python

从 PyPI 安装 wheel：

```bash
python -m pip install dynibo
```

wheel 已包含 dynibo 原生动态库，没有运行时 Python 包依赖。

## C 和 C++

构建 C/C++ 包需要安装 Rust、Cargo，以及 CMake 3.16 或更高版本：

```bash
cmake -S . -B build/c -DCMAKE_BUILD_TYPE=Release
cmake --build build/c --parallel
cmake --install build/c --prefix /opt/dynibo
```

在其他 CMake 项目中使用已安装的软件包：

```cmake
find_package(dynibo CONFIG REQUIRED)
target_link_libraries(my_robot PRIVATE dynibo::dynibo)
```

如果使用自定义安装位置，请在配置使用方项目时传入：

```bash
cmake -S . -B build -DCMAKE_PREFIX_PATH=/opt/dynibo
```

C 程序包含 `<dynibo/dynibo.h>`；C++ 程序包含
`<dynibo/dynibo.hpp>`，并要求 C++17。
