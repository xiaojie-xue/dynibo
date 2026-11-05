# Floating-Base Dynamics

Load a floating-base model and pass its pose, velocity, and acceleration
explicitly with `BaseState`. Joint arrays still have length J. Generalized
forces and accelerations have length G = J + 6, ordered as world angular base,
world linear base, then joints. The example verifies an RNEA/ABA round trip
and then sets the root wrench to zero to calculate unactuated free flight.

Install the published package, then save and run the complete Python script
below. It writes its own URDF in a temporary directory; no repository clone,
mesh downloads, or ROS installation are needed.

```bash
python -m pip install dynibo
python floating_base.py
```

```python
--8<-- "examples/python/recipes/floating_base.py"
```

## Results and conventions

The root must have strictly positive mass, and the articulated inertia must
be solvable. The six root effort entries are a wrench, not motor torques.
Forward dynamics ignores `base.acceleration`, because acceleration is the
result. This example has no ground contacts or balance controller.

[Python API reference](../reference/python.md) · [Frames and spatial vectors](../user-guide/frames-and-spatial-vectors.md)
