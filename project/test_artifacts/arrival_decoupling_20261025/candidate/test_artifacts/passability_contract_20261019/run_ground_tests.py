from pathlib import Path
import importlib.util, sys, unittest
ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT/'nav2_adapter'),str(ROOT/'official_adapter')]
suite=unittest.TestSuite()
for path in [ROOT/'nav2_adapter/test/test_pointcloud.py',ROOT/'official_adapter/test/test_terrain.py']:
 spec=importlib.util.spec_from_file_location(path.stem,path);module=importlib.util.module_from_spec(spec)
 spec.loader.exec_module(module);suite.addTests(unittest.defaultTestLoader.loadTestsFromModule(module))
result=unittest.TextTestRunner(verbosity=2).run(suite)
sys.exit(0 if result.wasSuccessful() else 1)
