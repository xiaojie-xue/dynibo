"""Build a platform wheel containing the Rust C ABI shared library."""

from __future__ import annotations

import os
import shutil
import subprocess
import sys
from pathlib import Path

from setuptools import Distribution, find_packages, setup
from setuptools.command.build_py import build_py
from wheel.bdist_wheel import bdist_wheel


ROOT = Path(__file__).resolve().parent


def native_library_name() -> str:
    if sys.platform == "win32":
        return "dynibo_c.dll"
    if sys.platform == "darwin":
        return "libdynibo_c.dylib"
    return "libdynibo_c.so"


class BinaryDistribution(Distribution):
    def has_ext_modules(self) -> bool:
        return True


class BuildPy(build_py):
    def run(self) -> None:
        super().run()
        profile = os.environ.get("DYNIBO_CARGO_PROFILE", "release")
        command = ["cargo", "build", "-p", "dynibo-c", "--locked"]
        if profile == "release":
            command.append("--release")
        else:
            command.extend(("--profile", profile))
        subprocess.run(command, cwd=ROOT, check=True)
        source = ROOT / "target" / profile / native_library_name()
        destination = Path(self.build_lib) / "dynibo" / native_library_name()
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, destination)


class PlatformWheel(bdist_wheel):
    """The bundled C ABI is Python-version independent but platform specific."""

    def get_tag(self) -> tuple[str, str, str]:
        _, _, platform = super().get_tag()
        return "py3", "none", platform


setup(
    name="dynibo",
    version="0.4.0",
    description="Python bindings for tree-structured robot kinematics and dynamics",
    author="Xiaojie Xue",
    long_description=(ROOT / "bindings/python/README.md").read_text(encoding="utf-8"),
    long_description_content_type="text/markdown",
    license="MIT",
    url="https://github.com/xiaojie-xue/dynibo",
    project_urls={
        "Documentation": "https://dynibo.readthedocs.io/",
        "Rust API": "https://docs.rs/dynibo",
        "Source Code": "https://github.com/xiaojie-xue/dynibo",
    },
    python_requires=">=3.9",
    package_dir={"": "bindings/python"},
    packages=find_packages("bindings/python"),
    package_data={"dynibo": ["*.so", "*.dylib", "*.dll"]},
    include_package_data=True,
    cmdclass={"build_py": BuildPy, "bdist_wheel": PlatformWheel},
    distclass=BinaryDistribution,
)
