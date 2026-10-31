"""Actual physical-entry -> launch -> Mapper parameters -> DDS profile; no vehicle/control."""
from pathlib import Path
import hashlib
import json
import math
import os
import signal
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import parameter_value_to_python
from rcl_interfaces.srv import GetParameters
from nav_msgs.msg import Odometry
from graphnav_msgs.msg import NavigationGraph
from ament_index_python.packages import get_package_prefix, get_package_share_directory
import yaml
from wildos_nav2_adapter.runtime_contract import graph_geometry_profile
import wildos_nav2_adapter.runtime_contract as contract_module

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT / 'candidate/tools'))
from wildos_console import OwnedProcesses

assert os.environ['ROS_DOMAIN_ID'] == '203'
OUT = ROOT / ('profile_dds_' + time.strftime('%Y%m%d_%H%M%S'))
OUT.mkdir()
owned = OwnedProcesses(OUT, ROOT)
rclpy.init()
node = Node('physical_profile_contract_probe')
graph = None


def on_graph(message):
    global graph
    graph = message


node.create_subscription(NavigationGraph, '/spot1/nav_graph', on_graph, 10)
odom_pub = node.create_publisher(Odometry, '/spot1/odom', 10)
report = dict(scope=__doc__, passed=False, errors=[], beamng_started=False,
              native_started=False, motion_goals=0, complete_navigation_passed=False)


def spin(seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        owned.healthy()
        rclpy.spin_once(node, timeout_sec=.02)


def get_fields(owner, names):
    client = node.create_client(GetParameters, owner + '/get_parameters')
    try:
        assert client.wait_for_service(timeout_sec=12.), owner
        future = client.call_async(GetParameters.Request(names=names))
        end = time.monotonic() + 5.
        while not future.done():
            assert time.monotonic() < end, owner
            spin(.02)
        assert not future.exception()
        return dict(zip(names, [parameter_value_to_python(v) for v in future.result().values]))
    finally:
        node.destroy_client(client)


try:
    manifest = json.loads((ROOT / 'frozen_source_manifest.json').read_text())
    actual = Path(contract_module.__file__)
    assert actual.is_relative_to(ROOT / 'install')
    assert hashlib.sha256(actual.read_bytes()).hexdigest() == manifest['members']['nav2_adapter/wildos_nav2_adapter/runtime_contract.py']
    for name in ('graphnav_mapper', 'graphnav_planner', 'wildos_nav2_adapter', 'beamng_wildos_adapter'):
        assert Path(get_package_prefix(name)) == ROOT / 'install' / name
    config = yaml.safe_load((Path(get_package_share_directory('wildos_nav2_adapter')) / 'config/nav2_ackermann_control.yaml').read_text())
    expected = graph_geometry_profile(config)
    child = owned.spawn('graph_stack', [sys.executable, str(ROOT / 'graph_stack_only.py')])
    params = get_fields('/spot1/graphnav_mapper', list(expected) + ['robot_profile_id', 'use_occupancy_input'])
    for name, want in expected.items():
        assert params[name] == want, (name, params[name], want)
    assert params['robot_profile_id'] == 'default' and params['use_occupancy_input'] is False
    planner_params = get_fields('/spot1/graphnav_planner', ['robot_profile_id'])
    assert planner_params['robot_profile_id'] == params['robot_profile_id']
    report['actual_mapper_parameters'] = params
    report['actual_planner_parameters'] = planner_params
    end = time.monotonic() + 8.
    while graph is None or not graph.robot_profiles:
        assert time.monotonic() < end, 'No actual Mapper DDS profile'
        odom = Odometry()
        odom.header.frame_id = 'spot1/odom'
        odom.header.stamp = node.get_clock().now().to_msg()
        odom.pose.pose.orientation.w = 1.
        odom_pub.publish(odom)
        spin(.15)
    profile = graph.robot_profiles[0]
    assert profile.profile_id == 'default'
    assert math.isclose(profile.body_size.x, 4.45, abs_tol=1.e-6)
    assert math.isclose(profile.body_size.y, 2.1, abs_tol=1.e-6)
    assert profile.body_size.z == profile.ground_clearance == -1.
    assert profile.holonomic is False and profile.min_turning_radius == 8.
    assert profile.max_linear_velocity == profile.max_linear_acceleration == profile.max_linear_deceleration == profile.max_angular_velocity == -1.
    properties = {item.key: list(item.value) for item in profile.properties}
    assert math.isclose(properties['minimum_horizontal_clearance'][0], 1.06, abs_tol=1.e-6)
    assert len(properties['footprint_xy']) == 8
    report['dds_profile'] = dict(body_size=[profile.body_size.x, profile.body_size.y, profile.body_size.z],
        ground_clearance=profile.ground_clearance, holonomic=profile.holonomic,
        min_turning_radius=profile.min_turning_radius, properties=properties)
    libraries = {}
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit():
            continue
        try:
            if os.getpgid(int(proc.name)) != child.pid:
                continue
            for line in (proc / 'maps').read_text().splitlines():
                name = line.split()[-1]
                if name.startswith('/') and Path(name).name in ('libgraphnav_mapper_component.so', 'libgraphnav_planner_component.so'):
                    path = Path(name)
                    assert path.is_relative_to(ROOT / 'install')
                    libraries[str(path)] = hashlib.sha256(path.read_bytes()).hexdigest()
        except (FileNotFoundError, ProcessLookupError):
            continue
    assert len(libraries) == 2, libraries
    report['runtime_libraries'] = libraries
    assert node.count_publishers('/wildos/native_control_command') == 0
    assert node.count_publishers('/wildos/cmd_vel_stamped') == 0
    report['passed'] = True
except BaseException as exc:
    report['errors'].append(repr(exc))
finally:
    report['errors'].extend(owned.close())
    node.destroy_node()
    rclpy.shutdown()
    report['passed'] = report['passed'] and not report['errors']
    (OUT / 'result.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(dict(output=str(OUT), **report), indent=2))
raise SystemExit(0 if report['passed'] else 1)
