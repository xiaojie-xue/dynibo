# 安装

请根据应用使用的语言选择对应的软件包。

## Rust

从 crates.io 添加 dynibo：

```bash
cargo add dynibo
```

最低支持 **Rust 1.88**，CI 会用该版本验证整个 workspace。
crate features 可以在 [Rust API 参考](https://docs.rs/dynibo)中查看。

## Python

从 PyPI 安装 wheel：

```bash
python -m pip install dynibo
```

wheel 包含直接调用 Rust core 的 PyO3 扩展，并以 NumPy 作为运行时数组依赖。

### Wheel 构建目标

CI 和 tag 发布流程会为以下平台构建并安装测试 CPython 3.9–3.14 的 wheel：

| 平台 | 架构 | Linux 兼容要求 |
|---|---|---|
| Linux | x86-64 | glibc 2.28 或更新版本 |
| Linux | aarch64 / ARM64 | glibc 2.28 或更新版本 |
| macOS | Apple Silicon arm64 | — |
| Windows | x86-64 | — |

Linux aarch64 使用原生 `ubuntu-24.04-arm` runner，生成
`manylinux_2_28_aarch64` wheel。目前不构建 Alpine/musl wheel。
已发布版本的实际覆盖范围请查看对应的 [PyPI 文件](https://pypi.org/project/dynibo/#files)，
旧版本不一定具备所有目标。若希望只安装 wheel、避免意外从源码构建，可使用：

```bash
python -m pip install --only-binary=:all: dynibo
```

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
