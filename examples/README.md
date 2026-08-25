# Complete examples

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

## C

[`c/franka.c`](c/franka.c) directly uses the stable C ABI. Build it together
with the library and run it from the repository root:

```bash
cmake -S . -B build/c -DDYNIBO_BUILD_EXAMPLES=ON
cmake --build build/c --parallel
./build/c/dynibo_c_example examples/data/franka_fer.urdf fer_link8
```

Depending on the platform and build layout, the shared-library directory may
need to be added to `PATH`, `LD_LIBRARY_PATH`, or `DYLD_LIBRARY_PATH` before
running the C executable.
