# URDF Forward Kinematics

Load a URDF and compute a tool pose from joint angles. The one-joint model
has a one-metre arm rotating about world y, so the tool translation is
`(cos(q), 0, -sin(q))`. The example verifies this analytical result.

Install the published package, then save and run the complete Python script
below. It writes its own URDF in a temporary directory; no repository clone,
mesh downloads, or ROS installation are needed.

```bash
python -m pip install dynibo
python forward_kinematics.py
```

```python
--8<-- "examples/python/recipes/forward_kinematics.py"
```

## Results and conventions

`Pose.rotation_xyzw` uses quaternion order `(x, y, z, w)`. Poses map local
link coordinates into world coordinates. Joint order is breadth-first among
non-fixed URDF joints, not an alphabetical sort.

[Python API reference](../reference/python.md) · [Frames and spatial vectors](../user-guide/frames-and-spatial-vectors.md)
