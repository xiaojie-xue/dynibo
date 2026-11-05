# Installation

Choose the package that matches the language used by your application.

## Rust

Add dynibo from crates.io:

```bash
cargo add dynibo
```

The minimum supported Rust version is **1.88**, tested in CI for the complete
workspace. Crate features are listed in the [Rust API reference](https://docs.rs/dynibo).

## Python

Install the wheel from PyPI:

```bash
python -m pip install dynibo
```

The wheel contains a PyO3 extension that calls the Rust core directly. NumPy is
installed as its runtime array dependency.

### Wheel build targets

CI and tagged releases build and install-test CPython 3.9–3.14 wheels on:

| Platform | Architecture | Linux compatibility |
|---|---|---|
| Linux | x86-64 | glibc 2.28 or newer |
| Linux | aarch64 / ARM64 | glibc 2.28 or newer |
| macOS | Apple Silicon arm64 | — |
| Windows | x86-64 | — |

Linux aarch64 runs on a native `ubuntu-24.04-arm` runner and produces
`manylinux_2_28_aarch64` wheels. Alpine/musl is not a wheel target.
For an already published version, check its [PyPI files](https://pypi.org/project/dynibo/#files):
older versions may not have every target. To require a wheel and avoid an
unexpected source build, use:

```bash
python -m pip install --only-binary=:all: dynibo
```

## C and C++

### Prebuilt packages

Tagged releases provide prebuilt C/C++ packages for the operating systems and
CPU architectures listed in the
[GitHub Release assets](https://github.com/xiaojie-xue/dynibo/releases). Each
archive contains the shared library, C and C++ headers, pkg-config metadata,
CMake package configuration, and the project license. Download the matching
archive and compare its SHA-256 digest with the release's `SHA256SUMS`.

For example, extract the Linux x86-64 package and point CMake at the extracted
directory:

```bash
version=X.Y.Z
archive="dynibo-${version}-Linux-X64"
tar -xzf "${archive}.tar.gz"
cmake -S . -B build \
  -DCMAKE_PREFIX_PATH="$PWD/${archive}"
cmake --build build
```

The shared library directory must be available when running the application:

| Platform | Runtime library location |
| --- | --- |
| Linux | Add `<package>/lib` to `LD_LIBRARY_PATH` |
| macOS | Add `<package>/lib` to `DYLD_LIBRARY_PATH` |
| Windows | Add `<package>/bin` to `PATH` |

Prebuilt packages cover only the platforms and architectures named by their
archives. Build from source for other targets.

### Build from source

Building the C/C++ package requires Rust with Cargo and CMake 3.16 or newer:

```bash
cmake -S . -B build/c -DCMAKE_BUILD_TYPE=Release
cmake --build build/c --parallel
cmake --install build/c --prefix /opt/dynibo
```

Consume either the installed package or an extracted prebuilt package from
CMake:

```cmake
find_package(dynibo CONFIG REQUIRED)
target_link_libraries(my_robot PRIVATE dynibo::dynibo)
```

Pass the installation prefix or extracted package directory while configuring
the consumer:

```bash
cmake -S . -B build -DCMAKE_PREFIX_PATH=/opt/dynibo
```

C programs include `<dynibo/dynibo.h>`. C++ programs include
`<dynibo/dynibo.hpp>` and require C++17.
