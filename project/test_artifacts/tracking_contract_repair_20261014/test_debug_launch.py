"""The diagnostic prefix may not change frozen launch parameters/roles."""
import ast,unittest
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2];HERE=Path(__file__).parent
class DebugLaunchContract(unittest.TestCase):
    def test_only_controller_prefix_differs(self):
        original=ast.parse((ROOT/'nav2_adapter/launch/nav2_official_control.launch.py').read_text())
        diagnostic=ast.parse((HERE/'nav2_debug_control.launch.py').read_text())
        prefixes=[]
        for n in ast.walk(diagnostic):
            if isinstance(n,ast.Call) and isinstance(n.func,ast.Name) and n.func.id=='Node':
                names={k.arg:k for k in n.keywords}
                if 'prefix' in names:
                    self.assertEqual(ast.literal_eval(names['executable'].value),'controller_server')
                    prefixes.append(names['prefix']);n.keywords.remove(names['prefix'])
        self.assertEqual(len(prefixes),1)
        self.assertEqual(ast.dump(original),ast.dump(diagnostic))
    def test_prefix_has_no_vehicle_control_or_shell_commands(self):
        tree=ast.parse((HERE/'controller_debug_prefix.py').read_text())
        imports={a.name for n in ast.walk(tree) if isinstance(n,(ast.Import,ast.ImportFrom)) for a in n.names}
        self.assertNotIn('beamngpy',imports)
        self.assertNotIn('rclpy',imports)
        self.assertNotIn('shell=True',(HERE/'controller_debug_prefix.py').read_text())
if __name__=='__main__':unittest.main()
