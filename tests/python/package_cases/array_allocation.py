"""Allocation failures in NumPy argument adaptation, isolated from the test runner."""

from __future__ import annotations

from pathlib import Path
import subprocess
import sys
import textwrap
import unittest

from .support import URDF


class ArrayAllocationTests(unittest.TestCase):
    @unittest.skipUnless(
        sys.platform == "linux" and Path("/proc/self/statm").exists(),
        "requires Linux address-space limits and /proc",
    )
    def test_copy_allocation_failure_preserves_outputs_and_allows_recovery(self) -> None:
        source = textwrap.dedent("""
            import os
            from pathlib import Path
            import resource
            import sys

            import dynibo
            import numpy as np

            # The broadcast view is small; only its contiguous copy needs 512 MiB.
            large = np.broadcast_to(np.zeros(1), (1 << 26,))
            for robot_type in (dynibo.Robot, dynibo.FloatingRobot):
                with robot_type(sys.argv[1]) as robot:
                    prefix = (dynibo.BaseState(),) if robot_type is dynibo.FloatingRobot else ()
                    q = np.zeros(robot.joint_count)
                    expected = robot.mass_matrix(*prefix, q)
                    out = np.full_like(expected, 123.0)
                    pages = int(Path("/proc/self/statm").read_text().split()[0])
                    current = pages * os.sysconf("SC_PAGE_SIZE")
                    previous = resource.getrlimit(resource.RLIMIT_AS)
                    limit = current + (64 << 20)
                    if previous[0] != resource.RLIM_INFINITY:
                        limit = min(limit, previous[0])
                    try:
                        resource.setrlimit(resource.RLIMIT_AS, (limit, previous[1]))
                        try:
                            robot.mass_matrix(*prefix, large, out=out)
                        except MemoryError:
                            pass
                        else:
                            raise AssertionError("expected NumPy copy allocation to fail")
                    finally:
                        resource.setrlimit(resource.RLIMIT_AS, previous)
                    np.testing.assert_array_equal(out, 123.0)
                    assert robot.mass_matrix(*prefix, q, out=out) is out
                    np.testing.assert_allclose(out, expected, atol=1.0e-11, rtol=1.0e-10)
        """)
        result = subprocess.run(
            [sys.executable, "-c", source, str(URDF)],
            capture_output=True, text=True, timeout=30,
        )
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
