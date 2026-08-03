# Rust、Python 与 C/C++ 打包和使用

三种语言入口共用 `dyno` Rust 核心，不会重复实现运动学或动力学算法：

| 分发包 | 安装方式 | 接口层 |
|---|---|---|
| Rust crate `dyno` | crates.io / Cargo | 原生 Rust API |
| Python distribution `dyno-robotics` | PyPI / pip | `ctypes` + wheel 内置 `dyno-c` |
| C/C++ package `dyno` | CMake 安装包 / GitHub Release | 稳定 C ABI + C++17 RAII header |

Python 的 distribution 名称是 `dyno-robotics`，import 名称是 `dyno`。C ABI 单独放在
workspace 成员 `dyno-c` 中，因此核心 crate 仍不含项目自身的 `unsafe` 代码。

## Rust crate

从本仓库使用：

```toml
[dependencies]
dyno = { path = "/path/to/dyno" }
```

发布后使用：

```toml
[dependencies]
dyno = "0.1"
```

```rust
use dyno::{Frame, Robot};

let robot = Robot::from_urdf("robot.urdf")?;
let tool = robot.link_id("tool")?;
let mut workspace = robot.workspace();
let q = vec![0.0; robot.joint_count()];

let pose = robot.forward_kinematics(&q, tool, &mut workspace)?;
let mut jacobian = vec![0.0; 6 * robot.joint_count()];
robot.jacobian(&q, tool, &mut workspace, &mut jacobian)?;
let mut gravity = vec![0.0; robot.joint_count()];
robot.gravity(&q, &Frame::identity(), &[], &mut workspace, &mut gravity)?;
# Ok::<(), dyno::Error>(())
```

发布者在更新 `Cargo.toml` 版本并提交后执行：

```bash
cargo package -p dyno
cargo publish -p dyno
```

`cargo publish` 需要 crates.io token；本仓库不会保存该 token。

## Python package

从当前 checkout 安装时需要 Rust toolchain：

```bash
python -m pip install .
```

发布到 PyPI 后，用户只需：

```bash
python -m pip install dyno-robotics
```

wheel 已包含本平台的动态库，运行时不需要 Rust、NumPy 或其他 Python 依赖：

```python
import numpy as np
from dyno import Load, Pose, Robot

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

上传前同步 `pyproject.toml`、`setup.py`、`python/dyno/__init__.py` 与 Cargo 包版本。
PyPI token 应通过 CI secret 或 trusted publishing 提供。

## C/C++ package

从源码构建并安装：

```bash
cmake -S . -B build/c -DCMAKE_BUILD_TYPE=Release
cmake --build build/c --parallel
cmake --install build/c --prefix /opt/dyno
```

生成可发布的二进制压缩包：

```bash
cmake --build build/c --target package
```

压缩包包含动态库、`dyno.h`、`dyno.hpp`、pkg-config 文件和 CMake package config。
应在每个目标操作系统和 CPU 架构分别生成包。

### C

```c
#include <dyno/dyno.h>

DynoRobot *robot = NULL;
DynoWorkspace *workspace = NULL;
if (dyno_robot_load_urdf("robot.urdf", &robot) != DYNO_STATUS_OK) {
    fprintf(stderr, "%s\n", dyno_last_error_message());
    return 1;
}
dyno_workspace_create(robot, &workspace);

size_t tool;
dyno_robot_link_id(robot, "tool", &tool);
size_t n = dyno_robot_joint_count(robot);
double *q = calloc(n, sizeof(double));
DynoPose pose;
dyno_forward_kinematics(robot, workspace, q, n, tool, &pose);

free(q);
dyno_workspace_destroy(workspace);
dyno_robot_destroy(robot);
```

所有 fallible C 函数返回 `DynoStatus`。非零时用 `dyno_last_error_message()` 读取当前线程
的错误文本。opaque handle 必须由对应的 `destroy` 函数释放。

### C++17

```cpp
#include <dyno/dyno.hpp>

dyno::Robot robot("robot.urdf");
auto tool = robot.link_id("tool");
std::vector<double> q(robot.joint_count(), 0.0);
auto pose = robot.forward_kinematics(q, tool);
auto jacobian = robot.jacobian(q, tool);
auto gravity = robot.gravity(q);
```

C++ header 自动管理 Robot/Workspace，并把 C 错误转成 `dyno::Error`。一个 wrapper 对象同样
不应被多个线程同时计算。

CMake consumer：

```cmake
find_package(dyno CONFIG REQUIRED)
target_link_libraries(my_robot_app PRIVATE dyno::dyno)
```

也可使用：

```bash
cc app.c $(pkg-config --cflags --libs dyno)
```

运行时必须能找到 `libdyno_c`；非系统 prefix 可设置平台对应的动态库搜索路径，或在应用
安装阶段配置 RPATH。

## 发布前检查

仓库的 [Package CI](.github/workflows/package-ci.yml) 在每次 push、pull request 和手动触发
时构建真实发布产物，并上传为 Actions artifact：

- Rust：生成 `.crate`，解压到源码 workspace 外，再运行包中携带的全部测试；
- Python：在 Linux、macOS、Windows 用 cibuildwheel 构建 wheel，并从临时目录测试已
  安装的 wheel；另行构建和验证 sdist；
- C/C++：在三个平台生成 CPack 压缩包，解压后由外部 CMake consumer 通过
  `find_package(dyno)` 分别运行 C 和 C++ 测试。

CI 不会自动上传 crates.io、PyPI 或创建 GitHub Release；只有验证通过的包会成为 workflow
artifact。发布仍需显式配置 registry credentials 和 release policy。

本地单元测试只针对 Rust source workspace，不构建或运行 Python、C、C++ 测试：

```bash
cargo fmt --all -- --check
cargo clippy --workspace --all-targets -- -D warnings
cargo test --workspace --all-targets
cargo +nightly llvm-cov --branch --workspace --all-targets
```

覆盖率 CI 会对 LLVM JSON 汇总执行 `ci/check-coverage.py`，要求行覆盖率至少为 85%、
分支覆盖率至少为 75%。

`ci/test-rust-package.sh`、`ci/test-native-package.py` 和 cibuildwheel 的
`package-tests/python/test_package.py` 由 GitHub Actions 调用，分别验证发布包，不属于本地
默认单元测试流程。
