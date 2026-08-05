# Rust、Python 与 C/C++ 安装和发布

[English](RELEASING.md) | 简体中文

三种语言入口共用 `dynibo` Rust 核心，不会重复实现运动学或动力学算法：

| 分发包 | 安装方式 | 接口层 |
|---|---|---|
| Rust crate `dynibo` | crates.io / Cargo | 原生 Rust API |
| Python distribution `dynibo` | PyPI / pip | `ctypes` + wheel 内置 `dynibo-c` |
| C/C++ package `dynibo` | CMake 安装包 / GitHub Release | 稳定 C ABI + C++17 RAII header |

Python 的 distribution 与 import 名称都是 `dynibo`。C ABI 单独放在 workspace 成员
`dynibo-c` 中，因此核心 crate 仍不含项目自身的 `unsafe` 代码。

## Rust crate

从本仓库使用：

```toml
[dependencies]
dynibo = { path = "/path/to/dynibo" }
```

发布后使用：

```toml
[dependencies]
dynibo = "0.1"
```

```rust
use dynibo::{Frame, Robot};

let robot = Robot::from_urdf("robot.urdf")?;
let tool = robot.link_id("tool")?;
let mut workspace = robot.workspace();
let q = vec![0.0; robot.joint_count()];

let pose = robot.forward_kinematics(&q, tool, &mut workspace)?;
let mut jacobian = vec![0.0; 6 * robot.joint_count()];
robot.jacobian(&q, tool, &mut workspace, &mut jacobian)?;
let mut gravity = vec![0.0; robot.joint_count()];
robot.gravity(&q, &Frame::identity(), &[], &mut workspace, &mut gravity)?;
# Ok::<(), dynibo::Error>(())
```

发布者在更新 `Cargo.toml` 版本并提交后执行：

```bash
cargo package -p dynibo --locked
cargo publish -p dynibo --locked
```

`cargo publish` 需要 crates.io token；本仓库不会保存该 token。

## Python package

从当前 checkout 安装时需要 Rust toolchain：

```bash
python -m pip install .
```

发布到 PyPI 后，用户只需：

```bash
python -m pip install dynibo
```

wheel 已包含本平台的动态库，运行时不需要 Rust、NumPy 或其他 Python 依赖：

```python
import numpy as np
from dynibo import Load, Pose, Robot

with Robot("robot.urdf") as robot:
    tool = robot.link_id("tool")
    q = [0.0] * robot.joint_count
    pose = robot.forward_kinematics(q, tool)
    flat = robot.jacobian(q, tool)
    jacobian = np.asarray(flat).reshape((6, robot.joint_count), order="F")
    gravity = robot.gravity(q)
```

`Pose.rotation_xyzw` 固定使用 `(x, y, z, w)` 顺序。Jacobian 是列主序 `6 x N`，每列
依次为角速度三分量和线速度三分量。`Robot` 内含一个可复用 Workspace，不应由多个线程
同时调用；并行计算请为每个工作线程创建一个 `Robot`。

native 参数错误会抛出 `ValueError`；模型加载错误抛出 `dynibo.ModelError`，迭代求解错误
抛出 `dynibo.SolverError`，后两者都继承 `dynibo.DyniboError` 和 `RuntimeError`。

本机构建 wheel：

```bash
python -m pip install build
python -m build --wheel
```

正式发布应分别在 Linux、macOS、Windows 构建。推荐用 `cibuildwheel`，它还会把 Linux
产物修复成 PyPI 接受的 manylinux wheel：

```bash
python -m pip install cibuildwheel twine
python -m cibuildwheel --output-dir wheelhouse
python -m twine check wheelhouse/*
python -m twine upload wheelhouse/*
```

上传前同步 `pyproject.toml`、`setup.py`、`bindings/python/dynibo/__init__.py` 与 Cargo 包版本。
PyPI token 应通过 CI secret 或 trusted publishing 提供。

## 自动发布 registry 包

发布一个非 prerelease 的 GitHub Release 会触发
[release workflow](../.github/workflows/release.yml)。release tag 必须使用
`vMAJOR.MINOR.PATCH` 格式，例如 `v0.1.0`。Rust crate、Python distribution、C ABI crate、
CMake project 和运行时版本常量只要有一处与 tag 不一致，workflow 就会拒绝发布。

workflow 会先执行格式检查、lint 和 workspace 测试，再构建并测试 Rust crate、Python
source distribution，以及 Linux、macOS、Windows wheel。只有所有打包 job 都成功，才会
开始上传；验证过的 `.crate`、wheel 和 source distribution 也会保留为 workflow artifact。

首次发布前需要在仓库中完成一次性配置：

1. 创建名为 `crates-io` 的 GitHub environment，将 crates.io publishing token 保存为
   environment secret `CRATES_IO_TOKEN`；
2. 创建名为 `pypi` 的 GitHub environment；在 PyPI 的 `dynibo` 项目中，将本仓库的
   `.github/workflows/release.yml` 和 `pypi` environment 配置为 Trusted Publisher。PyPI
   会通过 OIDC 签发短期凭据，GitHub 中不保存 PyPI API token；
3. 如果正式发布需要人工确认，为两个 environment 配置 required reviewer。

发布新版本时，先更新全部版本字段并执行：

```bash
python ci/check-release-version.py v0.1.0
bash ci/test-all.sh
```

提交并为该 revision 创建对应 tag，然后发布该 tag 的非 prerelease GitHub Release。
workflow 只把核心 crate `dynibo` 发布到 crates.io；内部 workspace 成员 `dynibo-c` 保持
不发布。Python sdist 和全部平台 wheel 会一起上传到 PyPI。

## C/C++ package

从源码构建并安装：

```bash
cmake -S . -B build/c -DCMAKE_BUILD_TYPE=Release
cmake --build build/c --parallel
cmake --install build/c --prefix /opt/dynibo
```

生成可发布的二进制压缩包：

```bash
cmake --build build/c --target package
```

压缩包包含动态库、`dynibo.h`、`dynibo.hpp`、pkg-config 文件和 CMake package config。
应在每个目标操作系统和 CPU 架构分别生成包。

### C

```c
#include <dynibo/dynibo.h>

DyniboRobot *robot = NULL;
DyniboWorkspace *workspace = NULL;
if (dynibo_robot_load_urdf("robot.urdf", &robot) != DYNIBO_STATUS_OK) {
    fprintf(stderr, "%s\n", dynibo_last_error_message());
    return 1;
}
dynibo_workspace_create(robot, &workspace);

size_t tool;
dynibo_robot_link_id(robot, "tool", &tool);
size_t n = dynibo_robot_joint_count(robot);
double *q = calloc(n, sizeof(double));
DyniboPose pose;
dynibo_forward_kinematics(robot, workspace, q, n, tool, &pose);

free(q);
dynibo_workspace_destroy(workspace);
dynibo_robot_destroy(robot);
```

所有 fallible C 函数返回 `DyniboStatus`。非零时用 `dynibo_last_error_message()` 读取当前线程
的错误文本。opaque handle 必须由对应的 `destroy` 函数释放。

状态码按调用方处理方式分为 `DYNIBO_STATUS_INVALID_ARGUMENT`、
`DYNIBO_STATUS_MODEL_ERROR`、`DYNIBO_STATUS_SOLVER_ERROR` 和 `DYNIBO_STATUS_PANIC`。
`DYNIBO_STATUS_ERROR` 继续作为 `DYNIBO_STATUS_MODEL_ERROR` 的兼容别名。

### C++17

```cpp
#include <dynibo/dynibo.hpp>

dynibo::Robot robot("robot.urdf");
auto tool = robot.link_id("tool");
std::vector<double> q(robot.joint_count(), 0.0);
auto pose = robot.forward_kinematics(q, tool);
auto jacobian = robot.jacobian(q, tool);
auto gravity = robot.gravity(q);
```

C++ header 自动管理 Robot/Workspace，并把 C 错误转成 `dynibo::Error`。一个 wrapper 对象同样
不应被多个线程同时计算；可以通过 `dynibo::Error::status()` 获取原始 `DyniboStatus` 并进行
程序化处理。

CMake consumer：

```cmake
find_package(dynibo CONFIG REQUIRED)
target_link_libraries(my_robot_app PRIVATE dynibo::dynibo)
```

也可使用：

```bash
cc app.c $(pkg-config --cflags --libs dynibo)
```

运行时必须能找到 `libdynibo_c`；非系统 prefix 可设置平台对应的动态库搜索路径，或在应用
安装阶段配置 RPATH。

## 发布前检查

仓库的 [Package CI](../.github/workflows/package-ci.yml) 在每次 push、pull request 和手动触发
时构建真实发布产物，并上传为 Actions artifact：

- Rust：生成 `.crate`，解压到源码 workspace 外，再运行包中携带的全部测试；
- Python：在 Linux、macOS、Windows 用 cibuildwheel 构建 wheel，并从临时目录测试已
  安装的 wheel；另行构建和验证 sdist；
- C/C++：在三个平台生成 CPack 压缩包，解压后由外部 CMake consumer 通过
  `find_package(dynibo)` 分别运行 C 和 C++ 测试。

Package CI 不会上传 crates.io、PyPI 或创建 GitHub Release。只有独立的 release workflow
会在上述 repository environment 和凭据完成配置后发布 registry 包。

本地单元测试只针对 Rust source workspace，不构建或运行 Python、C、C++ 测试：

```bash
cargo fmt --all -- --check
cargo clippy --workspace --all-targets -- -D warnings
cargo test --workspace --all-targets
cargo +nightly llvm-cov --branch --workspace --all-targets
```

本地完整验证可统一运行：

```bash
bash ci/test-all.sh
```

覆盖率 CI 会对 LLVM JSON 汇总执行 `ci/check-coverage.py`，要求行覆盖率至少为 85%、
分支覆盖率至少为 75%。

`ci/test-rust-package.sh`、`ci/test-native-package.py` 和 cibuildwheel 的
`tests/python/test_package.py` 由 GitHub Actions 调用，分别验证发布包，不属于本地
默认单元测试流程。
