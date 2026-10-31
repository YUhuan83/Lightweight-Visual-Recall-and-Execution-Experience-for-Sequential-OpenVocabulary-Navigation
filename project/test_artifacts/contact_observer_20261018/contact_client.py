"""Validate diagnostic events under closed motion gates; never publishes controls."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import time
from uuid import uuid4


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--vehicle-port', type=int, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    assert not args.output.exists()
    from beamngpy import BeamNGpy
    from beamngpy.connection import Connection
    import rclpy
    from rclpy.node import Node
    from rcl_interfaces.srv import GetParameters
    from std_msgs.msg import String
    from wildos_nav2_adapter.physical_vehicle import read_physics_snapshot, physical_snapshot_errors
    import contact_chunks
    rclpy.init()
    node = Node('contact_observer_brake_only_validator')
    controls = []; native = []
    snapshots_pub = node.create_publisher(String, '/wildos/diagnostic/contact_snapshot', 10)
    node.create_subscription(String, '/wildos/control_status', lambda m: controls.append(json.loads(m.data)), 10)
    node.create_subscription(String, '/wildos/native_control_status', lambda m: native.append(json.loads(m.data)), 10)
    report = dict(scope=__doc__, passed=False, errors=[], samples=[], video=False,
                  movement_goal_sent=False, control_messages_published=False,
                  instance_id='audit_'+uuid4().hex,
                  diagnostic_source_sha256=hashlib.sha256(Path(__file__).with_name('contact_observer.lua').read_bytes()).hexdigest())
    bng = vehicle = None; installed = False
    def spin_for(seconds):
        until = time.monotonic()+seconds
        while time.monotonic() < until: rclpy.spin_once(node, timeout_sec=.02)
    def closed_gates():
        for name, parameter in (('/wildos_cmd_vel_to_control', 'enable'), ('/wildos_graph_path_bridge', 'follow_path')):
            client = node.create_client(GetParameters, name+'/get_parameters')
            assert client.wait_for_service(timeout_sec=2.), name
            future = client.call_async(GetParameters.Request(names=[parameter]))
            rclpy.spin_until_future_complete(node, future, timeout_sec=3.)
            assert future.done() and future.result().values[0].type == 1
            assert future.result().values[0].bool_value is False, name+' gate armed'
        spin_for(.3)
        assert controls and native and controls[-1]['enabled'] is False
        assert controls[-1]['reason'] == 'disabled' and native[-1]['sent']['throttle'] == 0
        assert native[-1]['sent']['brake'] >= .9 and native[-1]['speed'] < .05
    try:
        until = time.monotonic()+45
        while not controls or not native:
            assert time.monotonic() < until, 'Native/control status readiness'
            rclpy.spin_once(node, timeout_sec=.1)
        closed_gates()
        bng = BeamNGpy('127.0.0.1', 25252, quit_on_close=False, socket_timeout=3).open(launch=False)
        report['physics_before'] = read_physics_snapshot(bng)
        assert not physical_snapshot_errors(report['physics_before'])
        vehicle = bng.vehicles.get_current()['ego']
        vehicle.connection = Connection('127.0.0.1', args.vehicle_port, timeout=3)
        vehicle.connect(bng)
        report['install'] = json.loads(vehicle.queue_lua_command(contact_chunks.install(report['instance_id']), response=True))
        installed = True
        assert report['install']['installed'] is True
        # Observe original collision physics while existing disabled Native owns braking.
        bng.control.resume()
        for _ in range(15):
            spin_for(.2)
            began = time.time_ns()
            value = json.loads(vehicle.queue_lua_command(contact_chunks.snapshot(report['instance_id']), response=True))
            ended = time.time_ns()
            assert value['protocol'] == contact_chunks.PROTOCOL and value['instance_id'] == report['instance_id']
            assert value['window_sequence'] == len(report['samples'])+1
            sample = dict(request_wall_ns=began, receipt_wall_ns=ended, contacts=value,
                          vehicle_id=report['physics_before']['vehicleID'])
            report['samples'].append(sample)
            snapshots_pub.publish(String(data=json.dumps(sample)))
            assert controls[-1]['enabled'] is False and controls[-1]['reason'] == 'disabled'
            assert native[-1]['speed'] < .05 and native[-1]['sent']['throttle'] == 0
        report['event_totals'] = {kind: sum(s['contacts']['class_events'][kind] for s in report['samples'])
                                  for kind in ('body', 'wheel', 'unknown')}
        assert report['event_totals']['wheel'] > 0, 'No real wheel collision callback observed'
        assert report['samples'][-1]['contacts']['end_sim_time'] > report['samples'][0]['contacts']['begin_sim_time']
        closed_gates()
        report['physics_after'] = read_physics_snapshot(bng)
        assert not physical_snapshot_errors(report['physics_after'])
        for key in ('nodeCount', 'beamCount', 'massKg', 'bodyCollisionNodeCount', 'wheelCount'):
            assert report['physics_before'][key] == report['physics_after'][key], key
        report['passed'] = True
    except BaseException as exc:
        report['errors'].append(repr(exc))
    finally:
        if vehicle:
            try:
                report['unload'] = json.loads(vehicle.queue_lua_command(contact_chunks.uninstall(report['instance_id']), response=True))
                assert report['unload']['unloaded'] is True
            except Exception as exc: report['errors'].append('unload: '+repr(exc))
            vehicle.disconnect()
        if bng: bng.disconnect()
        report['control_samples'] = len(controls); report['native_samples'] = len(native)
        report['maximum_native_speed'] = max((v['speed'] for v in native), default=None)
        report['positive_drive_commands'] = sum(v['sent']['throttle'] > 0 for v in native)
        report['watchdog_samples'] = sum(v['watchdog_stop'] for v in native)
        report['passed'] = report['passed'] and not report['errors']
        node.destroy_node(); rclpy.shutdown()
        args.output.write_text(json.dumps(report, indent=2)+'\n')
        print(json.dumps({k:v for k,v in report.items() if k not in ('samples', 'physics_before', 'physics_after')}, indent=2))
    return 0 if report['passed'] else 1


if __name__ == '__main__': raise SystemExit(main())
