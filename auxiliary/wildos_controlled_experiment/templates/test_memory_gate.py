"""Exercise experimental production methods without importing ROS or the model."""
import ast
import json
from pathlib import Path
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock

ROOT = Path(__file__).resolve().parent
NAV = ROOT/'wildos_port/repro_check/src/nebula2-wildos/visual_navigation/visual_navigation/wildos/nav.py'
TREE = ast.parse(NAV.read_text(encoding='utf8'))
CLASS = next(n for n in TREE.body if isinstance(n,ast.ClassDef) and n.name=='WildOS_Nav')
METHODS = {n.name:n for n in CLASS.body if isinstance(n,ast.FunctionDef)}


def method(name):
    namespace = dict(json=json, String=lambda **kw:NS(**kw), wall_time=lambda:123.)
    exec(compile(ast.fix_missing_locations(ast.Module(body=[METHODS[name]],type_ignores=[])),
                 str(NAV),'exec'),namespace)
    return namespace[name]


class MemoryGateTests(unittest.TestCase):
    def node(self, enabled):
        return NS(memory_experiment_enabled=enabled, object_goal_memory=Mock(),
            remembered_candidates_publisher=Mock(), semantic_landmarks=None,
            passive_query_candidates={}, object_query_started_ns=456,
            publish_legacy_remembered_goal=False,
            get_clock=lambda:NS(now=lambda:NS(nanoseconds=789)), get_logger=lambda:Mock())

    def test_disabled_arm_cannot_return_prior_viewpoints(self):
        n=self.node(False)
        n.object_goal_memory.candidates.side_effect=AssertionError('Historical lookup in disabled arm')
        n.passive_query_candidates={'gas station':[{'id':'passive:old'}]}
        method('_reuse_remembered_object_goal')(n,'gas station')
        data=json.loads(n.remembered_candidates_publisher.publish.call_args.args[0].data)
        self.assertEqual(data['views'],[])
        self.assertEqual(data['query'],'gas station')
        self.assertEqual(data['query_started_ns'],456)
        n.object_goal_memory.candidates.assert_not_called()

    def test_enabled_arm_retains_same_candidate_publication(self):
        n=self.node(True)
        n.object_goal_memory.candidates.return_value=[{'id':'confirmed:one'}]
        n.passive_query_candidates={'gas station':[{'id':'passive:two'}]}
        method('_reuse_remembered_object_goal')(n,'gas station')
        data=json.loads(n.remembered_candidates_publisher.publish.call_args.args[0].data)
        self.assertEqual([v['id'] for v in data['views']],['confirmed:one','passive:two'])
        n.object_goal_memory.candidates.assert_called_once_with('gas station',123.)

    def test_disabled_confirmation_callback_cannot_store_a_target(self):
        n=self.node(False)
        method('object_confirmation_callback')(n,NS(data='invalid JSON is deliberately never processed'))
        n.object_goal_memory.observe.assert_not_called()

    def test_disabled_arm_allocates_no_descriptor_memory(self):
        # Execute the actual three constructor guards with harmless factories.
        init=METHODS['__init__']
        guards=[n for n in init.body if isinstance(n,ast.If) and any(
            isinstance(x,ast.Attribute) and x.attr in ('visual_memory_enabled','visual_memory_map_enabled')
            for x in ast.walk(n.test))]
        semantic=next(n for n in init.body if isinstance(n,ast.Assign) and any(
            isinstance(t,ast.Attribute) and t.attr=='semantic_landmarks' for t in n.targets))
        defaults=ast.literal_eval(next(n.value for n in CLASS.body if isinstance(n,ast.Assign)
            and any(isinstance(t,ast.Name) and t.id=='default_config' for t in n.targets)))
        defaults.update(memory_experiment_enabled=False)
        for enabled in (False,True):
            n=self.node(enabled);n.memory_map_id='test';n.visual_memory=None;n.visual_memory_map=None;n.object_search_mode=True
            memory=Mock(return_value=Mock());map_memory=Mock(return_value=Mock());semantic_memory=Mock(return_value=Mock())
            namespace=dict(self=n,config=NS(**defaults),VisualMemory=memory,
                VisualMemoryMap=map_memory,SemanticLandmarkMemory=semantic_memory)
            exec(compile(ast.fix_missing_locations(ast.Module(body=guards+[semantic],type_ignores=[])),
                         str(NAV),'exec'),namespace)
            for factory in (memory,map_memory,semantic_memory):
                self.assertEqual(factory.call_count,int(enabled))

    def test_disabled_arm_keeps_no_scene_feature_tensor(self):
        gates=[n for fn in METHODS.values() for n in ast.walk(fn) if isinstance(n,ast.If)
               and ast.unparse(n.test)=='self.memory_experiment_enabled'
               and any(isinstance(x,ast.Attribute) and x.attr=='last_scene_features' for x in ast.walk(n))]
        self.assertEqual(len(gates),1)
        n=self.node(False);n.last_scene_features=None;n._capture_passive_landmarks=Mock()
        exec(compile(ast.fix_missing_locations(ast.Module(body=gates,type_ignores=[])),str(NAV),'exec'),dict(self=n))
        self.assertIsNone(n.last_scene_features)
        n._capture_passive_landmarks.assert_not_called()


if __name__=='__main__':
    unittest.main(verbosity=2)
