"""A renamed execution entry must retain ownership and conflict detection."""
import ast
from pathlib import Path
import unittest
import server_test_preflight as preflight


class ContinuousProcessRolesTests(unittest.TestCase):
    def matches(self, commands):
        markers=preflight.required_process_roles('ackermann')['graph_bridge']
        return [tokens for tokens in commands if any(t in markers or Path(t).name in markers for t in tokens)]

    def test_new_entry_is_recognized_as_the_graph_execution_owner(self):
        tokens=['python','/install/lib/wildos_nav2_adapter/continuous_graph_path_bridge','--ros-args']
        self.assertEqual(len(self.matches([tokens])),1)

    def test_old_and_new_entries_together_remain_conflicting_owners(self):
        old=['python','/install/lib/wildos_nav2_adapter/graph_path_bridge']
        new=['python','/install/lib/wildos_nav2_adapter/continuous_graph_path_bridge']
        self.assertEqual(len(self.matches([old,new])),2)

    def test_unrelated_similarly_named_process_is_not_an_owner(self):
        self.assertEqual(self.matches([['python','/tmp/not_continuous_graph_path_bridge']]),[])

    def test_console_rejects_leftover_continuous_execution_owner(self):
        tree=ast.parse((Path(__file__).parent/'wildos_console.py').read_text(encoding='utf8'))
        prepare=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='prepare')
        assignment=next(n for n in ast.walk(prepare) if isinstance(n,ast.Assign)
            and any(isinstance(t,ast.Name) and t.id=='conflicting' for t in n.targets))
        self.assertIn('continuous_graph_path_bridge',ast.literal_eval(assignment.value))
