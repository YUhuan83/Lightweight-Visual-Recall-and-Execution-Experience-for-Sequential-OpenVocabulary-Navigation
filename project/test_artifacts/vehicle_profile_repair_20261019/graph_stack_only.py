"""Launch only the graph-stack include from the actual physical entry point."""
from pathlib import Path
import importlib.util
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchService
from launch.actions import IncludeLaunchDescription

path = Path(get_package_share_directory('wildos_nav2_adapter')) / 'launch/physical_vehicle_control.launch.py'
spec = importlib.util.spec_from_file_location('_actual_physical_launch', path)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
description = module.generate_launch_description()
includes = [e for e in description.entities if isinstance(e, IncludeLaunchDescription)]
assert len(includes) == 2
# The first include is official_graph_stack; the second is Nav2. Native,
# speed-loop and graph bridge are top-level nodes and are not launched here.
service = LaunchService()
service.include_launch_description(LaunchDescription([includes[0]]))
raise SystemExit(service.run())
