"""Canonical names preserve exactly the same implementation and state."""
import importlib
import unittest
import ast
from pathlib import Path


class RecallNamesTests(unittest.TestCase):
    def test_legacy_names_are_identical_aliases_not_new_store_instances(self):
        for old,old_class,new,new_class in (
            ('visual_memory','VisualMemory','execution_experience_store','ExecutionExperienceStore'),
            ('visual_memory_map','VisualMemoryMap','graph_observation_store','GraphObservationStore'),
            ('viewpoint_memory','ViewpointMemory','confirmed_viewpoint_cache','ConfirmedViewpointCache'),
            ('semantic_landmark_memory','SemanticLandmarkMemory','passive_landmark_index','PassiveLandmarkIndex')):
            with self.subTest(name=new_class):
                legacy=importlib.import_module('visual_navigation.wildos.'+old)
                canonical=importlib.import_module('visual_navigation.wildos.'+new)
                self.assertIs(getattr(legacy,old_class),getattr(canonical,new_class))
                self.assertEqual(getattr(canonical,new_class).__name__,new_class)

    def test_production_nav_imports_canonical_modules_and_classes(self):
        path=Path(__file__).resolve().parents[1]/'visual_navigation/wildos/nav.py'
        tree=ast.parse(path.read_text(encoding='utf8'))
        imports={node.module:{alias.name for alias in node.names} for node in ast.walk(tree)
                 if isinstance(node,ast.ImportFrom)}
        for module,name in (('execution_experience_store','ExecutionExperienceStore'),
                            ('graph_observation_store','GraphObservationStore'),
                            ('confirmed_viewpoint_cache','ConfirmedViewpointCache'),
                            ('passive_landmark_index','PassiveLandmarkIndex')):
            self.assertIn(name,imports['visual_navigation.wildos.'+module])

    def test_compatibility_cache_shares_runtime_state(self):
        from visual_navigation.wildos.viewpoint_memory import ViewpointMemory
        from visual_navigation.wildos.confirmed_viewpoint_cache import ConfirmedViewpointCache
        cache=ViewpointMemory();cache.observe('tree','odom',(0.,0.,0.),0.,1.)
        self.assertIsInstance(cache,ConfirmedViewpointCache)
        self.assertEqual(len(cache.candidates('tree',2.)),1)
