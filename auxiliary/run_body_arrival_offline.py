import os,sys,unittest
from pathlib import Path
root=Path(r'C:\Users\Yu\Desktop\拟真\test_artifacts\arrival_decoupling_20261025\candidate')
sys.path[:0]=[str(root/'nav2_adapter/test'),str(root/'nav2_adapter'),str(root/'official_adapter'),str(root/'wildos_port/repro_check/src/nebula2-wildos/visual_navigation')]
from offline_ros_fakes import install
install()
names=sys.argv[1:] or ['test_body_arrival_ros']
suite=unittest.TestSuite(unittest.defaultTestLoader.loadTestsFromName(name) for name in names)
result=unittest.TextTestRunner(verbosity=2).run(suite)
raise SystemExit(not result.wasSuccessful())
