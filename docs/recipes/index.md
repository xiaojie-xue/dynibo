# Task Recipes

Complete examples organized by task. Each script works with the installed
Python package, embeds a small URDF, and verifies an analytical result or
numerical consistency.

- [URDF Forward Kinematics](forward-kinematics.md)
- [Jacobian and NumPy Layout](jacobian.md)
- [Gravity Compensation](gravity-compensation.md)
- [Inverse and Forward Dynamics (RNEA / ABA)](inverse-forward-dynamics.md)
- [Floating-Base Dynamics](floating-base.md)
- [Reusing Output Buffers](reuse-buffers.md)

For Rust, see the [crate quick start](../languages/rust.md) and the
[`recipes` example](https://github.com/xiaojie-xue/dynibo/blob/main/examples/rust/recipes.rs).
Run `cargo run --example recipes` to exercise the same six tasks.
