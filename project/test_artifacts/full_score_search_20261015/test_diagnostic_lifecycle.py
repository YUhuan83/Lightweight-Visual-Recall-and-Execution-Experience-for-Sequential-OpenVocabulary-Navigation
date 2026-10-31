"""Exercise the added recorder handoff; production navigation is not imported."""
import ast
from pathlib import Path
import tempfile
import types
import unittest


class RecorderHandoffTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.output = Path(self.tmp.name)
        self.calls = []
        self.backend = types.SimpleNamespace(output=self.output)
        self.backend.owned = types.SimpleNamespace(children=[])
        tree = ast.parse(Path(__file__).with_name('sensor_console.py').read_text(encoding='utf-8'))
        functions = ast.Module(body=[n for n in tree.body if isinstance(n, ast.FunctionDef)], type_ignores=[])

        def spawn(owned, role, argv, env=None, cwd=None):
            self.calls.append((role, argv))
            child = types.SimpleNamespace(poll=lambda: None)
            owned.children.append((role, child, {}))
            if role == 'sensor_bag':
                (self.output/'sensor_bag.log').write_text('All requested topics are subscribed')
            return child

        def begin(backend, query, folder):
            self.calls.append(('begin', query, folder))
            return self.marker

        self.marker = object()
        self.namespace = dict(original=spawn, original_begin=begin, Path=Path,
                              time=__import__('time'), os=__import__('os'))
        exec(compile(functions, 'diagnostic_functions', 'exec'), self.namespace)

    def test_vision_start_does_not_launch_recorder_before_preflight(self):
        owned = types.SimpleNamespace(children=[], output=self.output)
        self.namespace['spawn'](owned, 'vision', ['python', '-m', 'model', '--config', 'config'])
        self.assertEqual([r[0] for r in self.calls], ['execution_observer', 'vision'])

    def test_ready_recorder_preserves_begin_arguments_and_result(self):
        folder = self.output/'goal'
        result = self.namespace['begin'](self.backend, 'green residential house', folder)
        self.assertIs(result, self.marker)
        self.assertEqual(self.calls[-1], ('begin', 'green residential house', folder))

    def test_subsequent_goal_reuses_owned_recorder(self):
        self.namespace['begin'](self.backend, 'first', self.output/'one')
        self.namespace['begin'](self.backend, 'second', self.output/'two')
        self.assertEqual(sum(row[0] == 'sensor_bag' for row in self.calls), 1)

    def test_recorder_exit_prevents_original_begin(self):
        def exited(*args):
            return types.SimpleNamespace(poll=lambda: 1)
        self.namespace['original'] = exited
        with self.assertRaisesRegex(RuntimeError, 'recorder exited'):
            self.namespace['begin'](self.backend, 'goal', self.output/'goal')
        self.assertFalse(any(row[0] == 'begin' for row in self.calls))


if __name__ == '__main__':
    unittest.main()
