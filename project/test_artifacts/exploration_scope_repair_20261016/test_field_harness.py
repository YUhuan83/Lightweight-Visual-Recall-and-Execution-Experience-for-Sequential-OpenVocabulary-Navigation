"""Regression for exact diagnostic process identity at the real preflight boundary."""
import ast
from pathlib import Path, PurePosixPath
import sys
import types
import unittest
from unittest.mock import patch

FIELD = Path(__file__).parent / 'field'
REMOTE = '/home/ly/exploration_scope_repair_20261016'

class HarnessIdentityTest(unittest.TestCase):
    def test_spawned_vision_matches_preflight_role_and_config(self):
        calls = []
        class Owned:
            output = PurePosixPath('/unused-test-output')
            def spawn(self, role, argv, env=None, cwd=None):
                calls.append((role, argv, env, cwd))
                return argv
        module = types.SimpleNamespace(OwnedProcesses=Owned,
            PhysicalBackend=type('Backend', (), {'begin': lambda *a: None}))
        tree = ast.parse((FIELD/'sensor_console.py').read_text(encoding='utf8'))
        assert isinstance(tree.body[-1], ast.Raise)
        tree.body.pop()
        namespace = {}
        with patch.dict(sys.modules, wildos_console=module):
            exec(compile(tree, str(FIELD/'sensor_console.py'), 'exec'), namespace)
        owner = Owned()
        argv = owner.spawn('vision', ['/venv/python', '-m', 'visual_navigation.wildos.nav',
            '--config', '/installed/config.yaml', '--do_object_search', 'true', '--memory_in_process_only'])
        self.assertEqual(argv[argv.index('--config')+1], REMOTE+'/field/baseline_no_memory.yaml')
        self.assertIn('--memory_in_process_only', argv)
        preflight_argv = owner.spawn('preflight', ['/python', '/original/preflight.py', '--require-model'])
        self.assertEqual(preflight_argv[1], REMOTE+'/field/audit_preflight.py')
        self.assertIn('--require-model', preflight_argv)
        self.assertIn('--require-visualization', preflight_argv)
        preflight = ast.parse((FIELD/'audit_preflight.py').read_text(encoding='utf8'))
        role_path = next(ast.literal_eval(n.value.args[0]) for n in preflight.body
            if isinstance(n, ast.Assign) and isinstance(n.targets[0], ast.Name) and n.targets[0].id=='p')
        self.assertEqual(argv[1], role_path)
        self.assertEqual(calls[0][0], 'execution_observer')
        self.assertEqual(calls[1][2]['WILDOS_AUDIT_FILE'], '/unused-test-output/vision_participation.jsonl')

if __name__ == '__main__':
    unittest.main()
