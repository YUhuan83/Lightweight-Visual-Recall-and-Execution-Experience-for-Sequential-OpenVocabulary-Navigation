from pathlib import Path
import importlib.util, json, sys, tempfile, unittest
ROOT=Path(__file__).resolve().parents[2]
VIS=ROOT/'wildos_port/repro_check/src/nebula2-wildos/visual_navigation'
sys.path[:0]=[str(VIS),str(ROOT/'nav2_adapter')]
temp=Path(__file__).parent/'temp';temp.mkdir(exist_ok=True);tempfile.tempdir=str(temp)
names=sys.argv[1:] or ['test_directional_coverage_contract']
suite=unittest.TestSuite()
for name in names:
 path=VIS/'test'/(name+'.py')
 spec=importlib.util.spec_from_file_location(name,path);module=importlib.util.module_from_spec(spec)
 sys.modules[name]=module;spec.loader.exec_module(module)
 suite.addTests(unittest.defaultTestLoader.loadTestsFromModule(module))
result=unittest.TextTestRunner(verbosity=2).run(suite)
print(json.dumps(dict(tests=result.testsRun,failures=len(result.failures),errors=len(result.errors))))
sys.exit(0 if result.wasSuccessful() else 1)
