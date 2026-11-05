# Gravity Compensation

Compute the torque required to hold a fixed-base arm against gravity.
The arm has mass 1 kg and its centre of mass is 0.5 m along local x.
At joint angle q, the required y-axis torque is
`-0.5 * 9.80665 * cos(q)` N·m. The example checks this value and verifies
that applying the holding torque gives zero acceleration at rest.

Install the published package, then save and run the complete Python script
below. It writes its own URDF in a temporary directory; no repository clone,
mesh downloads, or ROS installation are needed.

```bash
python -m pip install dynibo
python gravity_compensation.py
```

```python
--8<-- "examples/python/recipes/gravity_compensation.py"
```

## Results and conventions

This is a numerical holding-torque calculation. Actuator interfaces, gains,
saturation, friction, and contact constraints belong to the application.
Optional `loads` are resisting wrenches in link-local axes at link origins.

[Python API reference](../reference/python.md) · [Frames and spatial vectors](../user-guide/frames-and-spatial-vectors.md)
