"""Use the real physical launch, isolating its path-action owner for a component test."""
import argparse
import ast
import hashlib
import json
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchService
from launch.actions import SetLaunchConfiguration

parser = argparse.ArgumentParser()
parser.add_argument('--vehicle-port', required=True, type=int)
parser.add_argument('--output', required=True, type=Path)
args = parser.parse_args()
path = Path(get_package_share_directory('wildos_nav2_adapter')) / 'launch/physical_vehicle_control.launch.py'
source = path.read_text()
tree = ast.parse(source)


class IsolateActionOwner(ast.NodeTransformer):
    removed = 0

    def visit_List(self, node):
        kept = []
        for item in node.elts:
            if (isinstance(item, ast.Call) and isinstance(item.func, ast.Name)
                    and item.func.id == 'Node' and any(k.arg == 'name'
                    and isinstance(k.value, ast.Constant)
                    and k.value.value == 'wildos_graph_path_bridge' for k in item.keywords)):
                self.removed += 1
            else:
                kept.append(self.visit(item))
        node.elts = kept
        return node


isolation = IsolateActionOwner()
tree = ast.fix_missing_locations(isolation.visit(tree))
assert isolation.removed == 1
namespace = dict(__name__='_isolated_execution_launch', __file__=str(path))
exec(compile(tree, str(path), 'exec'), namespace)
description = namespace['generate_launch_description']()
args.output.write_text(json.dumps(dict(original_launch=str(path),
    source_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
    removed_action_owners=['wildos_graph_path_bridge'],
    native_physics_and_parameters_unchanged=True, complete_navigation_passed=False), indent=2) + '\n')
service = LaunchService()
service.include_launch_description(LaunchDescription([
    SetLaunchConfiguration('vehicle_port', str(args.vehicle_port)),
    *description.entities,
]))
raise SystemExit(service.run())
