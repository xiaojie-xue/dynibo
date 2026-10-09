"""Benchmark installed Python batch FK and reusable loads with preallocated outputs.

Run: python benches/reuse.py robot.urdf [--floating] [--number 10000]
"""

import argparse
import json
import platform
import statistics
import timeit

import dynibo
import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("urdf")
    parser.add_argument("--floating", action="store_true")
    parser.add_argument("--number", type=int, default=10000)
    args = parser.parse_args()
    cls = dynibo.FloatingRobot if args.floating else dynibo.Robot
    with cls(args.urdf) as robot:
        prefix = (dynibo.BaseState(),) if args.floating else ()
        q = np.zeros(robot.joint_count)
        target = robot.link_count - 1
        loads = [dynibo.Load(target, force=(0.5, -0.2, 0.1))]
        buffer = robot.load_buffer()
        buffer.set(target, force=(0.5, -0.2, 0.1))
        force_out = np.empty(robot.generalized_count)
        pose_out = np.empty(7 * robot.link_count)
        np.testing.assert_allclose(robot.gravity(*prefix, q, loads=loads),
                                   robot.gravity(*prefix, q, loads=buffer))
        robot.forward_kinematics_all(*prefix, q, out=pose_out)
        for i, row in enumerate(pose_out.reshape(-1, 7)):
            pose = robot.forward_kinematics(*prefix, q, i)
            np.testing.assert_allclose(row[:3], pose.translation, atol=1e-12)
            np.testing.assert_allclose(row[3:], pose.rotation_xyzw, atol=1e-12)
        cases = {
            "gravity_list_ns": lambda: robot.gravity(*prefix, q, loads=loads, out=force_out),
            "gravity_buffer_ns": lambda: robot.gravity(*prefix, q, loads=buffer, out=force_out),
            "fk_individual_ns": lambda: [robot.forward_kinematics(*prefix, q, i) for i in range(robot.link_count)],
            "fk_batch_ns": lambda: robot.forward_kinematics_all(*prefix, q, out=pose_out),
        }
        results = {name: statistics.median(timeit.repeat(call, repeat=7, number=args.number))
                   * 1e9 / args.number for name, call in cases.items()}
        print(json.dumps({"python": platform.python_version(), "platform": platform.platform(),
                          "dynibo": dynibo.__version__, "model": robot.name,
                          "number": args.number, "results": results}, indent=2))


if __name__ == "__main__":
    main()
