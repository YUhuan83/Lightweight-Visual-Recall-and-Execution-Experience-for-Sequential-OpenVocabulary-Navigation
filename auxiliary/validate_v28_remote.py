import hashlib,inspect,json,sys,unittest
from pathlib import Path
root=Path('/home/ly/execution_continuity_review_20261022/v28_intent_handoff')
visual=root/'candidate/wildos_port/repro_check/src/nebula2-wildos/visual_navigation'
sys.path.insert(0,str(visual/'test'))
from visual_navigation.explorfm_triangulation.obj_mask_triangulation import ObjectMaskTriangulator
import test_fresh_bearing
path=Path(inspect.getfile(ObjectMaskTriangulator)).resolve()
expected=visual/'visual_navigation/explorfm_triangulation/obj_mask_triangulation.py'
assert path==expected,(path,expected)
assert hashlib.sha256(path.read_bytes()).hexdigest()==json.loads((root/'full/frozen_source_manifest.json').read_text())['members'][str(path.relative_to(root/'candidate'))]

class ImportedCallbackTests(test_fresh_bearing.FreshBearingTests):
    def setUp(self):
        super().setUp()
        actual=ObjectMaskTriangulator.__new__(ObjectMaskTriangulator)
        actual.__dict__.update(self.n.__dict__)
        self.n=actual

suite=unittest.TestSuite([unittest.defaultTestLoader.loadTestsFromTestCase(ImportedCallbackTests)])
for name in ('test_localization_geometry','test_triangulation_query_scope','test_semantic_mask','test_mask_lidar_delay'):
    suite.addTests(unittest.defaultTestLoader.loadTestsFromName(name))
result=unittest.TextTestRunner(verbosity=2).run(suite)
report=dict(source=str(path),tests=result.testsRun,failures=len(result.failures),errors=len(result.errors),
            passed=result.wasSuccessful(),scope='Imported production class with controlled sensor callbacks; no DDS/vehicle/semantic truth')
(root/'perception_report.json').write_text(json.dumps(report,indent=2));print(json.dumps(report))
raise SystemExit(not result.wasSuccessful())
