# Examples

The examples use the bundled 7-DOF Franka model and exercise every main
kinematics and dynamics calculation:

- `forward_kinematics`
- `jacobian`
- `jacobian_derivative`
- `forward_velocity_kinematics`
- `forward_acceleration_kinematics`
- `inverse_kinematics`
- `mass_matrix`
- `velocity_product_forces`
- `gravity`, including an external link load
- `inverse_dynamics`, including an external link load

## Rust

[`rust/franka.rs`](rust/franka.rs) uses the native Rust API and the reusable
calculation storage owned by `Robot`:

```bash
cargo run --example franka
```

## Python

[`python/franka.py`](python/franka.py) uses the Python package, which owns its
native workspace internally. From an environment where `dynibo` is installed:

```bash
python examples/python/franka.py
```

An alternative URDF and target link can be supplied with `URDF --target LINK`.
The sample joint states expect seven non-fixed joints.

## C++

[`cpp/franka.cpp`](cpp/franka.cpp) uses the RAII C++ wrapper, whose `Robot`
owns its reusable native workspace. Build it together with the library and run
it from the repository root:

```bash
cmake -S . -B build/cpp -DDYNIBO_BUILD_EXAMPLES=ON
cmake --build build/cpp --parallel
./build/cpp/dynibo_cpp_example examples/data/franka/franka_fer.urdf fer_link8
```

## C

[`c/franka.c`](c/franka.c) directly uses the stable C ABI. Build it together
with the library and run it from the repository root:

```bash
cmake -S . -B build/c -DDYNIBO_BUILD_EXAMPLES=ON
cmake --build build/c --parallel
./build/c/dynibo_c_example examples/data/franka/franka_fer.urdf fer_link8
```

Depending on the platform and build layout, the shared-library directory may
need to be added to `PATH`, `LD_LIBRARY_PATH`, or `DYLD_LIBRARY_PATH` before
running the C executable.

## Unitree G1 (Rust, Python, C++)

The G1 examples load `data/unitree-g1/g1_29dof_mode_11.urdf` with a floating
base, without loading meshes. The existing Franka examples remain fixed-base.

| Model | Joint state size | Generalized velocity/force size | Jacobian |
|---|---:|---:|---:|
| Franka FER, fixed base | 7 | 7 | 6 x 7 |
| Unitree G1, floating base | 29 | 35 | 6 x 35 |

Each G1 example computes the left hand's pose and world-aligned Jacobian, runs
inverse dynamics (RNEA), and verifies forward dynamics (ABA) recovers the
prescribed acceleration. It then sets the first six generalized forces to zero
to demonstrate an unactuated floating base. This is unconstrained free-flight
dynamics; it does not simulate foot contacts or balance control.

Run from the repository root:

```bash
cargo run --release --example g1
python examples/python/g1.py
cmake -S . -B build/g1 -DCMAKE_BUILD_TYPE=Release -DDYNIBO_BUILD_EXAMPLES=ON
cmake --build build/g1 --parallel
./build/g1/dynibo_cpp_g1_example
```

- [Rust](rust/g1.rs) uses reusable output buffers and `BaseState`.
- [Python](python/g1.py) uses the current NumPy/PyO3 package, including `out=`.
- [C++](cpp/g1.cpp) uses the RAII `FloatingRobot` wrapper; an optional first
  command-line argument overrides the G1 URDF path.

Floating-base vectors place world-expressed angular components before linear
components, then joint entries in Dynibo's breadth-first URDF order. RNEA's
first six outputs are a base wrench, not motor torques. ABA returns the base's
classical acceleration followed by joint accelerations. See
[model licensing](data/README.md).
