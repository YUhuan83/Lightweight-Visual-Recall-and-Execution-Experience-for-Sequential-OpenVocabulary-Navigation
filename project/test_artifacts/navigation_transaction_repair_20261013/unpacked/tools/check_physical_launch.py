"""Expand the real ROS launch tree without starting any process or vehicle.

Humble Node substitutions are inspected before ExecuteProcess runs. This is
an integration check of include arguments, not DDS or motion acceptance.
"""
import argparse
import json
from pathlib import Path
import yaml


def main():
    from ament_index_python.packages import get_package_share_directory
    from launch import LaunchDescription, LaunchService
    from launch.actions import IncludeLaunchDescription
    from launch.launch_description_sources import PythonLaunchDescriptionSource
    from launch.utilities import perform_substitutions
    from launch_ros.actions import Node
    from launch_ros.substitutions import ExecutableInPackage

    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    records = []
    original = Node.execute
    class LaunchYamlLoader(yaml.SafeLoader):
        pass
    # Humble normalizes parameter sequences to tuples and tags them when
    # writing its own temporary files. Accept only this sequence tag.
    LaunchYamlLoader.add_constructor('tag:yaml.org,2002:python/tuple',
                                    lambda loader, node: loader.construct_sequence(node))

    def capture(action, context):
        # Same substitution step used by Humble Node.execute, with the
        # ExecuteProcess step deliberately omitted. No transport is opened.
        action._perform_substitutions(context)
        executable = action.node_executable
        if not isinstance(executable, str):
            executable = perform_substitutions(context, executable)
        # Expansion alone does not resolve libexec permissions. Use the same
        # resolver as Node's process launch, while still suppressing execution.
        resolved = ExecutableInPackage(package=action.node_package,
                                       executable=executable).perform(context)
        parameters = {}
        files = []
        for path, is_file in getattr(action, '_Node__expanded_parameter_arguments', []) or []:
            if not is_file:
                raise RuntimeError('Unexpected scalar parameter form in physical launch')
            body = yaml.load(Path(path).read_text(), Loader=LaunchYamlLoader)
            files.append(str(path))
            # Generated dictionaries select the node name, or wildcard when
            # launch does not specify a name. Nav2 files have multiple roots.
            key = action.node_name.lstrip('/')
            root = body.get(action.node_name, body.get(key,
                body.get(action.node_name.rsplit('/', 1)[-1], body.get('/**', {}))))
            parameters.update(root.get('ros__parameters', {}))
        records.append(dict(executable=executable, resolved_executable=resolved, name=action.node_name,
                            parameters=parameters, files=files))
        return None

    Node.execute = capture
    try:
        share = Path(get_package_share_directory('wildos_nav2_adapter'))
        service = LaunchService()
        service.include_launch_description(LaunchDescription([
            IncludeLaunchDescription(PythonLaunchDescriptionSource(
                str(share/'launch/physical_vehicle_control.launch.py')))]))
        returncode = service.run()
    finally:
        Node.execute = original

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(dict(scope='Launch expansion before assertions',
        passed=False, nodes=records, launch_returncode=returncode), indent=2), encoding='utf-8')
    if returncode:
        print(json.dumps(dict(passed=False, launch_returncode=returncode)))
        return returncode

    def one(executable):
        found = [r for r in records if r['executable'] == executable]
        if len(found) != 1:
            raise AssertionError(executable + ': expected one expanded node')
        return found[0]

    checks = {}
    bridge = one('graph_path_bridge')['parameters']
    checks['physical_bridge'] = (bridge.get('carrier_model') == 'ackermann'
        and bridge.get('planner_id') == 'CarLike' and bridge.get('minimum_turning_radius') == 8.
        and bridge.get('follow_path') is False)
    speed = one('cmd_vel_to_control')['parameters']
    checks['native_speed_loop'] = (speed.get('enable') is False and speed.get('allow_reverse') is True
        and speed.get('state_topic') == '/vehicles/ego/sensors/state'
        and speed.get('control_topic') == '/wildos/native_control')
    checks['physical_adapter'] = one('beamng_wildos_adapter')['parameters'].get('carrier_model') == 'ackermann'
    goal = one('goal_standoff')['parameters']
    checks['physical_standoff'] = goal.get('carrier_model') == 'ackermann' and goal.get('vehicle_front_extent') == 3.4
    cloud = one('cloud_filter')['parameters']
    checks['self_returns_filter'] = (cloud.get('filter_footprint_returns') is True
        and [cloud.get(k) for k in ('footprint_front_extent', 'footprint_rear_extent', 'footprint_half_width')]
        == [3.4, 1.05, 1.05])
    planner = one('planner_server')
    controller = one('controller_server')
    checks['physical_nav2_config'] = (planner['parameters'].get('planner_plugins') == ['CarLike']
        and controller['parameters'].get('FollowPath', {}).get('use_rotate_to_heading') is False
        and controller['parameters'].get('FollowPath', {}).get('use_collision_detection') is True)
    checks['native_transport_present'] = bool(one('native_vehicle_control'))
    checks['no_fixed_carrier'] = not any('kinematic_vehicle_control' in r['executable'] for r in records)
    report = dict(scope='Real Humble launch expansion; Node process execution suppressed',
                  checks=checks, nodes=records, launch_returncode=returncode,
                  passed=returncode == 0 and all(checks.values()))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps(dict(passed=report['passed'], checks=checks)))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
