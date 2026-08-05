#!/usr/bin/env python3
"""Extract a CPack archive and test it as an external C/C++ consumer."""

from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys
import tarfile
from pathlib import Path


def run(command: list[str], **kwargs: object) -> None:
    print("+", " ".join(command), flush=True)
    subprocess.run(command, check=True, **kwargs)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--package-dir", type=Path, required=True)
    parser.add_argument("--configuration", default="Release")
    args = parser.parse_args()

    project = Path(__file__).resolve().parents[1]
    archives = sorted(args.package_dir.resolve().glob("dynibo-*.tar.gz"))
    if len(archives) != 1:
        raise SystemExit(f"expected exactly one CPack archive, found: {archives}")

    work = project / "target" / "package-test" / "native"
    extract = work / "installed"
    build = work / "build"
    if work.exists():
        shutil.rmtree(work)
    extract.mkdir(parents=True)
    with tarfile.open(archives[0], "r:gz") as archive:
        archive.extractall(extract, filter="data")

    configs = list(extract.rglob("dynibo-config.cmake"))
    if len(configs) != 1:
        raise SystemExit(f"expected exactly one dynibo-config.cmake, found: {configs}")
    prefix = configs[0].parents[3]
    urdf = (project / "tests" / "data" / "test_arm.urdf").resolve()

    run([
        "cmake", "-S", str(project / "tests" / "native"),
        "-B", str(build),
        f"-DCMAKE_PREFIX_PATH={prefix}",
        f"-DDYNIBO_TEST_URDF={urdf}",
        f"-DCMAKE_BUILD_TYPE={args.configuration}",
    ])
    run(["cmake", "--build", str(build), "--config", args.configuration, "--parallel"])

    environment = os.environ.copy()
    path_entries = [prefix / "bin", prefix / "lib"]
    environment["PATH"] = os.pathsep.join(map(str, path_entries)) + os.pathsep + environment["PATH"]
    if sys.platform == "darwin":
        environment["DYLD_LIBRARY_PATH"] = str(prefix / "lib")
    elif os.name != "nt":
        environment["LD_LIBRARY_PATH"] = str(prefix / "lib")
    run([
        "ctest", "--test-dir", str(build), "--build-config", args.configuration,
        "--output-on-failure",
    ], env=environment)


if __name__ == "__main__":
    main()
