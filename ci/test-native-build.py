#!/usr/bin/env python3
"""Exercise native profile/output selection and relocated pkg-config consumers."""

from __future__ import annotations

import argparse
import os
import shlex
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path


PROJECT = Path(__file__).resolve().parents[1]


def run(command: list[str], **kwargs: object) -> None:
    print("+", shlex.join(command), flush=True)
    subprocess.run(command, check=True, timeout=600, **kwargs)


def test_pkg_config(prefix: Path, work: Path) -> None:
    tools = list(dict.fromkeys(
        str(Path(executable).resolve())
        for name in ("pkg-config", "pkgconf")
        if (executable := shutil.which(name)) is not None
    ))
    if os.name == "nt" or not tools:
        print("Skipping pkg-config consumer: requires POSIX and pkg-config", flush=True)
        return
    metadata = list(prefix.rglob("dynibo.pc"))
    if len(metadata) != 1:
        raise RuntimeError(f"expected one installed dynibo.pc, found: {metadata}")
    work.mkdir(parents=True, exist_ok=True)
    environment = os.environ.copy()
    environment["PKG_CONFIG_LIBDIR"] = str(metadata[0].parent)
    environment.pop("PKG_CONFIG_PATH", None)
    library_dir = metadata[0].parent.parent
    loader_path = "DYLD_LIBRARY_PATH" if sys.platform == "darwin" else "LD_LIBRARY_PATH"
    environment[loader_path] = str(library_dir)
    for index, tool in enumerate(tools):
        print(f"Testing pkg-config consumer with {tool}", flush=True)
        flags = subprocess.check_output(
            [tool, "--cflags", "--libs", "dynibo"],
            env=environment, text=True, timeout=30,
        )
        executable = work / f"pkg-config-consumer-{index}"
        run([
            *shlex.split(environment.get("CC", "cc")),
            str(PROJECT / "tests" / "native" / "test_c.c"),
            "-o", str(executable), *shlex.split(flags), "-lm",
        ], env=environment)
        run([
            str(executable), str(PROJECT / "tests" / "data" / "test_arm.urdf"),
        ], env=environment)


def test_build(work: Path) -> None:
    source = work / "source snapshot"
    source.mkdir()
    for name in ("Cargo.toml", "Cargo.lock", "CMakeLists.txt", "build.rs", "LICENSE"):
        shutil.copy2(PROJECT / name, source / name)
    for name in ("src", "bindings", "benches", "examples"):
        shutil.copytree(
            PROJECT / name, source / name,
            ignore=shutil.ignore_patterns("__pycache__", "*.pyc", "results"),
        )

    build = work / "native build"
    environment = os.environ.copy()
    environment["CARGO_TARGET_DIR"] = str(work / "redirected cargo output")
    run([
        "cmake", "-S", str(source), "-B", str(build),
        "-DDYNIBO_CARGO_PROFILE=dev", "-DCMAKE_INSTALL_LIBDIR=lib64",
    ], env=environment)
    run(["cmake", "--build", str(build), "--config", "Debug", "--parallel"], env=environment)
    installed = work / "custom install"
    run(["cmake", "--install", str(build), "--prefix", str(installed), "--config", "Debug"])
    relocated = work / "relocated install"
    installed.rename(relocated)
    test_pkg_config(relocated, work / "consumer build")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--prefix", type=Path, help="only test an existing installation")
    args = parser.parse_args()
    with tempfile.TemporaryDirectory(prefix="dynibo-native-build-") as temporary:
        work = Path(temporary)
        if args.prefix is None:
            test_build(work)
        else:
            test_pkg_config(args.prefix.resolve(), work)


if __name__ == "__main__":
    main()
