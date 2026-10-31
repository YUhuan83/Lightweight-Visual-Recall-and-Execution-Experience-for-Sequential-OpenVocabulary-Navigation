"""Offline config wiring, independent of the Qt layout verification tool."""
import ast
from pathlib import Path
import unittest
import yaml

ROOT=Path(__file__).resolve().parents[2]


class RvizConfigurationTests(unittest.TestCase):
    def setUp(self):
        self.config=yaml.safe_load((ROOT/'nav2_adapter/config/beamng_navigation.rviz').read_text())

    def test_camera_topology_object_names_and_routes_are_enabled(self):
        displays=self.config['Visualization Manager']['Displays']
        enabled={d.get('Topic',{}).get('Value') for d in displays if d.get('Enabled')}
        self.assertTrue({'/wildos/viz/rgb_target','/wildos/viz/topology',
            '/wildos/viz/object_labels','/wildos/viz/planned_path','/wildos/viz/graph_path'}<=enabled)
        self.assertEqual(sum(d['Class']=='rviz_default_plugins/Image' and d.get('Enabled') for d in displays),1)

    def test_planar_view_and_native_camera_dock_state(self):
        view=self.config['Visualization Manager']['Views']['Current']
        self.assertEqual(view['Class'],'rviz_default_plugins/TopDownOrtho')
        state=bytes.fromhex(self.config['Window Geometry']['QMainWindow State'])
        self.assertIn('RGB + Target'.encode('utf-16-be'),state)
        self.assertFalse(self.config['Window Geometry']['Hide Right Dock'])
        self.assertEqual(self.config['Window Geometry']['Width'],1920)

    def test_visualization_has_no_navigation_or_control_publisher(self):
        tree=ast.parse((ROOT/'nav2_adapter/wildos_nav2_adapter/visualization_bridge.py').read_text())
        topics=[node.args[1].value for node in ast.walk(tree) if isinstance(node,ast.Call)
                and isinstance(node.func,ast.Attribute) and node.func.attr=='create_publisher'
                and len(node.args)>1 and isinstance(node.args[1],ast.Constant)]
        self.assertTrue(topics)
        self.assertTrue(all(t.startswith('/wildos/viz/') or t in
            ('/wildos/debug_markers','/wildos/odom_trail') for t in topics))

    def test_policy_compatibility_alias_is_the_canonical_policy(self):
        tree=ast.parse((ROOT/'nav2_adapter/wildos_nav2_adapter/recall_candidate_policy.py').read_text())
        self.assertIn('RecallCandidatePolicy',[n.name for n in tree.body if isinstance(n,ast.ClassDef)])
        old=(ROOT/'nav2_adapter/wildos_nav2_adapter/memory_selection.py').read_text()
        self.assertIn('MemorySelection = RecallCandidatePolicy',old)
        bridge=(ROOT/'nav2_adapter/wildos_nav2_adapter/graph_path_bridge.py').read_text()
        self.assertIn('from .recall_candidate_policy import RecallCandidatePolicy',bridge)


if __name__=='__main__':unittest.main()
