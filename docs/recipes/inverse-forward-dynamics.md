# Inverse and Forward Dynamics (RNEA / ABA)

Compute joint torque with recursive Newton–Euler inverse dynamics (RNEA),
then recover the requested acceleration with articulated-body forward dynamics
(ABA). The example also verifies `tau = M @ qdd + Cqd + gravity`; the
one-joint inertia is 0.35 kg·m².

Install the published package, then save and run the complete Python script
below. It writes its own URDF in a temporary directory; no repository clone,
mesh downloads, or ROS installation are needed.

```bash
python -m pip install dynibo
python inverse_forward_dynamics.py
```

```python
--8<-- "examples/python/recipes/inverse_forward_dynamics.py"
```

## Results and conventions

Keep q, qd, and qdd in non-fixed joint order. `forces` is a generalized vector,
not a base-plus-joint position vector. Pass identical resisting loads to the
inverse and forward calls when checking a round trip.

[Python API reference](../reference/python.md) · [Frames and spatial vectors](../user-guide/frames-and-spatial-vectors.md)
