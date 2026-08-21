# Installation

Choose the package that matches the language used by your application.

## Rust

Add dynibo from crates.io:

```bash
cargo add dynibo
```

The supported Rust version and crate features are listed in the
[Rust API reference](https://docs.rs/dynibo).

## Python

Install the wheel from PyPI:

```bash
python -m pip install dynibo
```

The wheel bundles the native dynibo library and has no runtime Python package
dependencies.

## C and C++

Building the C/C++ package requires Rust with Cargo and CMake 3.16 or newer:

```bash
cmake -S . -B build/c -DCMAKE_BUILD_TYPE=Release
cmake --build build/c --parallel
cmake --install build/c --prefix /opt/dynibo
```

Consume the installed package from CMake:

```cmake
find_package(dynibo CONFIG REQUIRED)
target_link_libraries(my_robot PRIVATE dynibo::dynibo)
```

When using a custom install prefix, pass it while configuring the consumer:

```bash
cmake -S . -B build -DCMAKE_PREFIX_PATH=/opt/dynibo
```

C programs include `<dynibo/dynibo.h>`. C++ programs include
`<dynibo/dynibo.hpp>` and require C++17.
