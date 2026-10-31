"""Check installed full graph entry and production 10s setting; no vehicle or actuation."""
from pathlib import Path
import hashlib
import importlib
import json
import signal
import sys
import time
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rcl_interfaces.srv import GetParameters
from ament_index_python.packages import get_package_share_directory
import yaml
from wildos_console import OwnedProcesses, verify_sources
from wildos_nav2_adapter.runtime_contract import navigation_runtime_contract, runtime_parameter_value

ROOT=Path(__file__).resolve().parent
OUT=ROOT/('entry_'+time.strftime('%Y%m%d_%H%M%S'));OUT.mkdir()
owned=OwnedProcesses(OUT,ROOT);report=dict(passed=False,errors=[],beamng_started=False,complete_navigation_passed=False)
rclpy.init(signal_handler_options=SignalHandlerOptions.NO);node=Node('frontier_missing_entry_probe')
try:
    manifest=json.loads((ROOT/'frozen_source_manifest.json').read_text())
    assert all(hashlib.sha256((ROOT/'candidate'/n).read_bytes()).hexdigest()==h for n,h in manifest['members'].items())
    identity=json.loads(Path('/home/ly/continuous_goal_repair_20261020/physical/verified_install_sources.json').read_text())
    identity['libraries']['libgraphnav_planner_component.so']=hashlib.sha256((ROOT/'install/graphnav_planner/lib/libgraphnav_planner_component.so').read_bytes()).hexdigest()
    report['verified_install_sources']=verify_sources({'library_sha256':identity['libraries']})
    report['python_identity']={}
    for name in ('graph_path_bridge','runtime_contract','continuous_path'):
        path=Path(importlib.import_module('wildos_nav2_adapter.'+name).__file__).resolve()
        assert path.is_relative_to(ROOT/'install')
        report['python_identity'][str(path)]=hashlib.sha256(path.read_bytes()).hexdigest()
    launch=Path(get_package_share_directory('beamng_wildos_adapter'))/'launch/official_graph_stack.launch.py'
    report['graph_entry']=dict(path=str(launch),sha256=hashlib.sha256(launch.read_bytes()).hexdigest())
    child=owned.spawn('graph_stack',[sys.executable,str(ROOT/'graph_stack_only.py')])
    client=node.create_client(GetParameters,'/spot1/graphnav_planner/get_parameters')
    assert client.wait_for_service(timeout_sec=10.)
    config=yaml.safe_load((ROOT/'candidate/nav2_adapter/config/nav2_ackermann_control.yaml').read_text())
    expected=navigation_runtime_contract(config,'ackermann')['/spot1/graphnav_planner']
    future=client.call_async(GetParameters.Request(names=list(expected)))
    deadline=time.monotonic()+5.
    while not future.done() and time.monotonic()<deadline:
        owned.healthy();rclpy.spin_once(node,timeout_sec=.05)
    assert future.done()
    actual={k:runtime_parameter_value(v,expected[k]) for k,v in zip(expected,future.result().values)}
    assert actual==expected;report['actual_parameters']=actual
    assert node.count_publishers('/wildos/native_control_command')==0
    assert node.count_publishers('/wildos/cmd_vel_stamped')==0
    report['passed']=True
except BaseException as error:
    report['errors'].append(repr(error))
finally:
    report['errors'].extend(owned.close());report['passed']=report['passed'] and not report['errors']
    node.destroy_node();rclpy.shutdown()
    (OUT/'result.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(dict(output=str(OUT),**report),indent=2))
raise SystemExit(not report['passed'])
