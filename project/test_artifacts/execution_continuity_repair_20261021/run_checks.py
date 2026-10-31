"""Isolated production callback counterexamples, not DDS or physical validation."""
from pathlib import Path
import importlib
import json
import sys
import unittest

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT / 'nav2_adapter/test'), str(ROOT / 'nav2_adapter'),
                str(ROOT / 'official_adapter'),
                str(ROOT / 'wildos_port/repro_check/src/nebula2-wildos/visual_navigation')]
from offline_ros_fakes import install
install()
names = sys.argv[1:] or ['test_committed_frontier_path_ros']
suite = unittest.TestSuite(unittest.defaultTestLoader.loadTestsFromModule(importlib.import_module(name))
                           for name in names)
result = unittest.TextTestRunner(verbosity=2).run(suite)
print(json.dumps(dict(passed=result.wasSuccessful(), tests=result.testsRun,
                      failures=len(result.failures), errors=len(result.errors),
                      real_ros=False, vehicle=False)))
raise SystemExit(0 if result.wasSuccessful() else 1)
