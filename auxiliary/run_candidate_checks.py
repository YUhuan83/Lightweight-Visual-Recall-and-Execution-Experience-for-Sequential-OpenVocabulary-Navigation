import sys,unittest
from pathlib import Path
root=Path(sys.argv[1]);names=sys.argv[2:]
sys.path[:0]=[str(root/'nav2_adapter/test'),str(root/'nav2_adapter'),str(root/'official_adapter'),
 str(root/'wildos_port/repro_check/src/nebula2-wildos/visual_navigation'),
 str(root/'wildos_port/repro_check/src/nebula2-wildos/visual_navigation/test')]
from offline_ros_fakes import install
install()
suite=unittest.TestSuite(unittest.defaultTestLoader.loadTestsFromName(n) for n in names)
result=unittest.TextTestRunner(verbosity=2).run(suite)
raise SystemExit(not result.wasSuccessful())
