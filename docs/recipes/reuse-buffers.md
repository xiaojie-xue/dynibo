# Reusing Output Buffers

Create contiguous float64 inputs, flat output arrays, and a native load buffer
once, then reuse them across calculations. The returned array is the exact
`out` object; a reshaped Jacobian can share its storage. Assertions verify
buffer identity and the numerical result at each angle.

Install the published package, then save and run the complete Python script
below. It writes its own URDF in a temporary directory; no repository clone,
mesh downloads, or ROS installation are needed.

```bash
python -m pip install dynibo
python reuse_buffers.py
```

```python
--8<-- "examples/python/recipes/reuse_buffers.py"
```

## Results and conventions

Native calculation storage and output arrays are reused here. This does not
claim that an entire Python loop is allocation-free: scalar/value objects,
Python calls, and this example's assertions may allocate. Avoid a new list of
`Load` objects on each call by reusing `robot.load_buffer()`.

[Python API reference](../reference/python.md) · [Frames and spatial vectors](../user-guide/frames-and-spatial-vectors.md)
