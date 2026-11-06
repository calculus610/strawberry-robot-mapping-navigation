#!/usr/bin/env python3
"""Run both archived unittest suites against a chosen native180 projector build."""

import argparse
import importlib
import os
from pathlib import Path
import runpy
import sys
import types
import unittest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runtime", type=Path, required=True, help="native180 root containing ws/src")
    parser.add_argument("--projection-fixture", type=Path, required=True, help="archived test_projection.py")
    parser.add_argument("--free-fixture", type=Path, required=True, help="archived test_free_evidence.py")
    parser.add_argument("--library", type=Path, required=True, help="built libfree_evidence for the runtime under test")
    args = parser.parse_args()
    scripts = (args.runtime / "ws/src/native_height_projection/scripts").resolve()
    sys.path.insert(0, str(scripts))
    os.environ["FREE_EVIDENCE_LIBRARY"] = str(args.library.resolve(strict=True))

    # Import the runtime under test before the archived fixtures can prepend
    # their historical source paths.
    for name in ("free_evidence", "projection_core"):
        module = importlib.import_module(name)
        if Path(module.__file__).resolve().parent != scripts:
            parser.error(f"{name} came from a different runtime: {module.__file__}")
    suite = unittest.TestSuite()
    loader = unittest.TestLoader()
    for fixture in (args.projection_fixture, args.free_fixture):
        module = types.ModuleType(fixture.stem)
        module.__dict__.update(runpy.run_path(str(fixture.resolve(strict=True))))
        suite.addTests(loader.loadTestsFromModule(module))
    result = unittest.TextTestRunner(verbosity=2).run(suite)
    if not result.wasSuccessful() or result.testsRun < 18:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
