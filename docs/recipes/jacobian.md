# Jacobian and NumPy Layout

Compute a world geometric Jacobian and reshape it correctly for NumPy.
Rows 0–2 are angular and rows 3–5 are linear at the target origin.
The example checks the linear block against a central finite difference
of forward kinematics, rather than only checking the array size.

Install the published package, then save and run the complete Python script
below. It writes its own URDF in a temporary directory; no repository clone,
mesh downloads, or ROS installation are needed.

```bash
python -m pip install dynibo
python jacobian.py
```

```python
--8<-- "examples/python/recipes/jacobian.py"
```

## Results and conventions

Use `reshape((6, robot.generalized_count), order="F")`, including for a
floating base. `forward_kinematics_all` instead returns consecutive
seven-value pose records; reshape those with `(-1, 7)` in the default order.

[Python API reference](../reference/python.md) · [Frames and spatial vectors](../user-guide/frames-and-spatial-vectors.md)
