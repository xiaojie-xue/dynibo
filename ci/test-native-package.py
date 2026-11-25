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
    subprocess.run(command, check=True, timeout=600, **kwargs)


def check_relocatable_linkage(prefix: Path, build: Path) -> None:
    if sys.platform == "darwin":
        lib = prefix / "lib" / "libdynibo_c.dylib"
        install_id = subprocess.check_output(["otool", "-D", str(lib)], text=True, timeout=30)
        if "@rpath/libdynibo_c.dylib" not in install_id:
            raise SystemExit(f"expected @rpath install_name in {lib}, got:\n{install_id}")
        for binary in (build / "dynibo_c_package_test", build / "dynibo_cpp_package_test"):
            if not binary.exists():
                raise SystemExit(f"expected built consumer binary {binary}")
            deps = subprocess.check_output(["otool", "-L", str(binary)], text=True, timeout=30)
            if "@rpath/libdynibo_c.dylib" not in deps:
                raise SystemExit(f"expected @rpath linkage in {binary}, got:\n{deps}")
    elif sys.platform.startswith("linux") and shutil.which("readelf"):
        libs = list(prefix.rglob("libdynibo_c.so"))
        if len(libs) != 1:
            raise SystemExit(f"expected one libdynibo_c.so under {prefix}, found: {libs}")
        dynamic = subprocess.check_output(["readelf", "-d", str(libs[0])], text=True, timeout=30)
        if "(SONAME)" not in dynamic or "[libdynibo_c.so]" not in dynamic:
            raise SystemExit(f"expected SONAME [libdynibo_c.so] in {libs[0]}, got:\n{dynamic}")
        for binary in (build / "dynibo_c_package_test", build / "dynibo_cpp_package_test"):
            if not binary.exists():
                raise SystemExit(f"expected built consumer binary {binary}")
            deps = subprocess.check_output(["readelf", "-d", str(binary)], text=True, timeout=30)
            if "(NEEDED)" not in deps or "[libdynibo_c.so]" not in deps:
                raise SystemExit(
                    f"expected non-path NEEDED [libdynibo_c.so] in {binary}, got:\n{deps}"
                )


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
    reference = (project / "tests" / "data" / "pinocchio_reference_v1.tsv").resolve()

    run([
        "cmake", "-S", str(project / "tests" / "native"),
        "-B", str(build),
        f"-DCMAKE_PREFIX_PATH={prefix}",
        f"-DDYNIBO_TEST_URDF={urdf}",
        f"-DDYNIBO_TEST_REFERENCE={reference}",
        f"-DCMAKE_BUILD_TYPE={args.configuration}",
    ])
    run(["cmake", "--build", str(build), "--config", args.configuration, "--parallel"])
    check_relocatable_linkage(prefix, build)

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
    run([
        sys.executable, str(project / "ci" / "test-native-build.py"),
        "--prefix", str(prefix),
    ])


if __name__ == "__main__":
    main()
