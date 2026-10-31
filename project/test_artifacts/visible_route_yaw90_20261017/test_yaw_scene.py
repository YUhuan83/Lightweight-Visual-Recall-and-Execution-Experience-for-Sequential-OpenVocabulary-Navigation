"""Test-only spawn rotation scope and existing scene failure/cleanup paths."""
import ast
import importlib.util
import math
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path[:0] = [str(ROOT/'tools'),str(ROOT/'nav2_adapter')]
from test_beamng_kinematic_math import Quat, rz
from test_wildos_console import ConsoleTests

def scene_tree(path):
    tree = ast.parse(path.read_text(encoding='utf8'))
    call = next(n for n in ast.walk(tree) if isinstance(n,ast.Call)
                and isinstance(n.func,ast.Attribute) and n.func.attr=='add_vehicle')
    keyword = next(k for k in call.keywords if k.arg=='rot_quat')
    quat = ast.literal_eval(keyword.value)
    keyword.value = ast.Constant(value='SPAWN_ROTATION')
    return tree,quat

class LeftSpawnTests(unittest.TestCase):
    def test_only_spawn_quaternion_changes(self):
        old,_ = scene_tree(ROOT/'tools/wildos_console_scene.py')
        new,_ = scene_tree(HERE/'field/scene_left90.py')
        self.assertEqual(ast.dump(old),ast.dump(new))

    def test_engine_passive_rotation_is_left_90_and_keeps_up(self):
        _,old = scene_tree(ROOT/'tools/wildos_console_scene.py')
        _,new = scene_tree(HERE/'field/scene_left90.py')
        self.assertAlmostEqual(sum(x*x for x in new),1.)
        for axis in ((1,0,0),(0,1,0),(0,0,1)):
            actual=Quat(*new).rotate(axis)
            expected=rz(Quat(*old).rotate(axis),math.pi/2)
            for a,b in zip(actual,expected):
                self.assertAlmostEqual(a,b,places=12)

    def run_existing(self, name):
        spec=importlib.util.spec_from_file_location('wildos_console_scene',HERE/'field/scene_left90.py')
        scene=importlib.util.module_from_spec(spec);spec.loader.exec_module(scene)
        result=unittest.TestResult()
        tmp=HERE/'test_tmp';tmp.mkdir(exist_ok=True)
        with patch.dict(sys.modules,{'wildos_console_scene':scene}),patch.object(tempfile,'tempdir',str(tmp)):
            ConsoleTests(name).run(result)
        self.assertEqual(result.errors,[])
        self.assertEqual(result.failures,[])

    def test_stationary_single_load_and_sdk_cleanup(self):
        self.run_existing('test_fresh_scene_loads_once_and_releases_sdk_before_sensor_start')

    def test_invalid_physics_still_pauses_and_disconnects(self):
        self.run_existing('test_bad_physical_scene_is_reported_and_paused')

if __name__=='__main__':
    unittest.main(defaultTest='LeftSpawnTests',verbosity=2)
