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

wheel 包含直接调用 Rust core 的 PyO3 扩展，并以 NumPy 作为运行时数组依赖。

## C 和 C++

### 预编译包

正式版本会在
[GitHub Release Assets](https://github.com/xiaojie-xue/dynibo/releases)
中提供适用于指定操作系统和 CPU 架构的 C/C++ 预编译包。每个压缩包包含动态库、
C/C++ 头文件、pkg-config 元数据、CMake package 配置和项目许可证。请选择匹配的
压缩包，并将其 SHA-256 摘要与同一 Release 中的 `SHA256SUMS` 对比。

例如，解压 Linux x86-64 包并让 CMake 使用解压目录：

```bash
version=X.Y.Z
archive="dynibo-${version}-Linux-X64"
tar -xzf "${archive}.tar.gz"
cmake -S . -B build \
  -DCMAKE_PREFIX_PATH="$PWD/${archive}"
cmake --build build
```

运行应用时必须让系统能够找到动态库：

| 平台 | 运行时动态库位置 |
| --- | --- |
| Linux | 将 `<package>/lib` 加入 `LD_LIBRARY_PATH` |
| macOS | 将 `<package>/lib` 加入 `DYLD_LIBRARY_PATH` |
| Windows | 将 `<package>/bin` 加入 `PATH` |

预编译包只覆盖压缩包名称中标明的平台和架构；其他目标需要从源码构建。

### 从源码构建

构建 C/C++ 包需要安装 Rust、Cargo，以及 CMake 3.16 或更高版本：

```bash
cmake -S . -B build/c -DCMAKE_BUILD_TYPE=Release
cmake --build build/c --parallel
cmake --install build/c --prefix /opt/dynibo
```

在其他 CMake 项目中使用已安装的软件包或解压后的预编译包：

```cmake
find_package(dynibo CONFIG REQUIRED)
target_link_libraries(my_robot PRIVATE dynibo::dynibo)
```

配置使用方项目时传入安装目录或预编译包解压目录：

```bash
cmake -S . -B build -DCMAKE_PREFIX_PATH=/opt/dynibo
```

C 程序包含 `<dynibo/dynibo.h>`；C++ 程序包含
`<dynibo/dynibo.hpp>`，并要求 C++17。
