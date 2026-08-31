#!/usr/bin/env python3
"""Franka/G1 public Python API benchmarks against Pinocchio; see README.md."""

from __future__ import annotations

import argparse
from collections import defaultdict, deque
from datetime import datetime, timezone
import gc
import hashlib
import json
import os
from pathlib import Path
import platform
import random
import statistics
import timeit
import xml.etree.ElementTree as ET

import numpy as np
import pinocchio as pin
import dynibo
from dynibo import BaseState, FloatingRobot, Pose, Robot, Twist

ROOT = Path(__file__).resolve().parents[1]
MODELS = (
    ("franka_fixed", "examples/data/franka/franka_fer.urdf", "fer_link8", False, 7),
    ("g1_floating", "examples/data/unitree-g1/g1_29dof_mode_11.urdf", "left_rubber_hand", True, 29),
)


def active_joints(path):
    """Read Dynibo's breadth-first joint order, retaining URDF sibling order.

    Python does not expose joint_name yet. The complete Jacobian/RNEA/ABA
    comparisons below validate this ordering against independently named
    Pinocchio joints; never copy a branched model's vectors without mapping.
    """
    root = ET.parse(path).getroot()
    children = defaultdict(list)
    child_links = set()
    for joint in root.findall("joint"):
        children[joint.find("parent").get("link")].append(joint)
        child_links.add(joint.find("child").get("link"))
    roots = [link.get("name") for link in root.findall("link") if link.get("name") not in child_links]
    assert len(roots) == 1
    queue = deque(roots)
    result = []
    while queue:
        for joint in children[queue.popleft()]:
            queue.append(joint.find("child").get("link"))
            if joint.get("type") != "fixed":
                assert joint.get("type") == "revolute"
                result.append(joint)
    return result


def make_case(spec):
    name, relative, target_name, floating, n = spec
    path = ROOT / relative
    robot = (FloatingRobot if floating else Robot).from_urdf(path)
    joints = active_joints(path)
    assert robot.joint_count == len(joints) == n
    g = robot.generalized_count
    assert g == n + (6 if floating else 0)
    indices = np.arange(1, n + 1, dtype=float)
    lower = np.array([float(j.find("limit").get("lower")) for j in joints])
    upper = np.array([float(j.find("limit").get("upper")) for j in joints])
    q = (lower + upper) * 0.5 + (upper - lower) * 0.1 * np.sin(0.37 * indices)
    v = 0.4 * np.cos(0.23 * indices)
    a = 0.3 * np.sin(0.41 * indices)
    omega, linear = np.array([0.1, -0.05, 0.08]), np.array([0.2, 0.0, -0.1])
    alpha, acc = np.array([0.02, 0.03, -0.01]), np.array([0.1, -0.2, 0.05])
    base = BaseState(Pose(translation=(0.2, -0.1, 0.8)), Twist(omega, linear), Twist(alpha, acc))
    base_args = (base,) if floating else ()
    target = robot.link_id(target_name)
    model = pin.buildModelFromUrdf(str(path), pin.JointModelFreeFlyer()) if floating else pin.buildModelFromUrdf(str(path))
    model.gravity.linear = np.array([0.0, 0.0, -9.80665])
    data = model.createData()
    frame = model.getFrameId(target_name, pin.BODY)
    assert frame < model.nframes and model.nv == g
    pq, pv, pa = pin.neutral(model), np.zeros(g), np.zeros(g)
    columns = [3, 4, 5, 0, 1, 2] if floating else []
    if floating:
        pq[:3] = base.frame.translation
        pv[:3], pv[3:6] = linear, omega
        pa[:3], pa[3:6] = acc - np.cross(omega, linear), alpha
    for i, joint in enumerate(joints):
        jid = model.getJointId(joint.get("name"))
        assert jid < model.njoints
        pj = model.joints[jid]
        assert pj.nq == pj.nv == 1
        pq[pj.idx_q], pv[pj.idx_v], pa[pj.idx_v] = q[i], v[i], a[i]
        columns.append(pj.idx_v)
    forces = robot.inverse_dynamics(*base_args, q, v, a)
    expected = np.r_[alpha, acc, a] if floating else a
    roundtrip = robot.forward_dynamics(*base_args, q, v, forces)
    np.testing.assert_allclose(roundtrip, expected, atol=1e-8, rtol=1e-8)
    forces = forces.copy()
    if floating:
        forces[:6] = 0.0  # Unactuated base, no contact constraints.
    ptau = np.empty(g)
    ptau[columns] = forces
    if floating:
        dyn_functions = {
            "jacobian": lambda: robot.jacobian(base, q, target),
            "rnea": lambda: robot.inverse_dynamics(base, q, v, a),
            "aba": lambda: robot.forward_dynamics(base, q, v, forces),
        }
    else:
        dyn_functions = {
            "jacobian": lambda: robot.jacobian(q, target),
            "rnea": lambda: robot.inverse_dynamics(q, v, a),
            "aba": lambda: robot.forward_dynamics(q, v, forces),
        }
    pin_functions = {
        "jacobian": lambda: pin.computeFrameJacobian(model, data, pq, frame, pin.LOCAL_WORLD_ALIGNED),
        "rnea": lambda: pin.rnea(model, data, pq, pv, pa),
        "aba": lambda: pin.aba(model, data, pq, pv, ptau),
    }
    functions = {op: {"dynibo": dyn_functions[op], "pinocchio": pin_functions[op]}
                 for op in dyn_functions}
    errors = {}
    for operation, functions_pair in functions.items():
        actual = np.array(functions_pair["dynibo"](), copy=True)
        reference = np.array(functions_pair["pinocchio"](), copy=True)
        if operation == "jacobian":
            actual = actual.reshape((6, g), order="F")
            reference = reference[np.ix_([3, 4, 5, 0, 1, 2], columns)]
        else:
            reference = reference[columns]
            if operation == "aba" and floating:
                reference[3:6] += np.cross(omega, linear)
        assert np.isfinite(actual).all() and np.isfinite(reference).all()
        np.testing.assert_allclose(actual, reference, atol=1e-8, rtol=1e-8,
                                   err_msg=f"{name}/{operation}")
        errors[operation] = float(np.max(np.abs(actual - reference)))
    metadata = {"name": name, "path": relative, "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
                "target": target_name, "joints": n, "generalized_count": g,
                "joint_order": [j.get("name") for j in joints], "q": q.tolist(), "v": v.tolist(), "a": a.tolist(),
                "validation_max_abs_error": errors}
    return functions, metadata


def measure_pair(functions, samples, batch_seconds):
    timers = {name: timeit.Timer(fn) for name, fn in functions.items()}
    iterations = {}
    for name, timer in timers.items():
        number = 1
        while timer.timeit(number) < batch_seconds:
            number *= 2
        iterations[name] = number
    values = {name: [] for name in timers}
    rng = random.Random(20260831)
    for _ in range(samples):
        order = list(timers)
        rng.shuffle(order)
        for name in order:
            ns = timers[name].timeit(iterations[name]) * 1e9 / iterations[name]
            values[name].append(ns)
    return {name: {"median_ns": statistics.median(timings),
                   "p10_ns": float(np.percentile(timings, 10)),
                   "p90_ns": float(np.percentile(timings, 90)),
                   "iterations_per_sample": iterations[name], "samples_ns": timings}
            for name, timings in values.items()}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--samples", type=int, default=30)
    parser.add_argument("--batch-seconds", type=float, default=0.02)
    parser.add_argument("--cpu", type=int)
    parser.add_argument("--check", action="store_true", help="validate both models without timing")
    parser.add_argument("--output", type=Path, default=ROOT / "target/benchmarks/python.json")
    args = parser.parse_args()
    if args.samples < 3 or args.batch_seconds <= 0:
        parser.error("need at least 3 samples and a positive batch time")
    if args.cpu is not None:
        os.sched_setaffinity(0, {args.cpu})
    native = __import__("dynibo._dynibo", fromlist=["__file__"])
    report = {"timestamp_utc": datetime.now(timezone.utc).isoformat(), "language": "python",
              "python": platform.python_version(), "numpy": np.__version__, "dynibo": dynibo.__version__,
              "dynibo_extension": native.__file__,
              "dynibo_extension_sha256": hashlib.sha256(Path(native.__file__).read_bytes()).hexdigest(),
              "pinocchio": pin.__version__, "platform": platform.platform(),
              "cpu_affinity": sorted(os.sched_getaffinity(0)) if hasattr(os, "sched_getaffinity") else None,
              "output_api": "default", "samples": args.samples,
              "gc_disabled_during_timing": True, "models": [], "results": []}
    for spec in MODELS:
        functions, metadata = make_case(spec)
        report["models"].append(metadata)
        print("validated", spec[0], metadata["validation_max_abs_error"], flush=True)
        if args.check:
            continue
        for operation, pair in functions.items():
            gc.collect()
            stats = measure_pair(pair, args.samples, args.batch_seconds)
            speedup = stats["pinocchio"]["median_ns"] / stats["dynibo"]["median_ns"]
            report["results"].append({"model": spec[0], "operation": operation, **stats, "speedup": speedup})
            print(f"{spec[0]:14} {operation:8} dynibo={stats['dynibo']['median_ns']/1000:.3f} us "
                  f"pinocchio={stats['pinocchio']['median_ns']/1000:.3f} us speedup={speedup:.3f}x", flush=True)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n")
    print("saved", args.output)


if __name__ == "__main__":
    main()
