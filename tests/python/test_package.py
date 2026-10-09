"""Run all installed-package tests through the existing command-line entry."""

import unittest

from package_cases.robot import PackageTests
from package_cases.values import ValueTypeTests
from package_cases.floating import FloatingMotionTests
from package_cases.contracts import BindingContractTests


if __name__ == "__main__":
    unittest.main()
