"""Brake-held BeamNG/DDS freshness fault injection; no navigation or video.

Synthetic speed-loop commands go to a private topic with no actuator. The
real native actuator receives only zero throttle, neutral and full brake.
"""
import argparse
import json
import math
import os
from pathlib import Path
import threading
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--vehicle-port', type=int, required=True)
    args = parser.parse_args()
    if args.output.exists(): raise FileExistsError(args.output)
    if os.environ.get('ROS_DOMAIN_ID') != '79' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise RuntimeError('Requires the inspected localhost BeamNG domain 79')
    from beamngpy import BeamNGpy
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.parameter import Parameter
    from std_msgs.msg import String
    from geometry_msgs.msg import Twist
    from beamng_msgs.msg import StateSensor, VehicleControl
    from wildos_nav2_adapter.native_vehicle_control import NativeVehicleControl
    from wildos_nav2_adapter.cmd_vel_to_control import CmdVelToControl
    from wildos_nav2_adapter.source_control import SourceControlSender, COMMAND_TOPIC
    from wildos_nav2_adapter.physical_vehicle import read_physics_snapshot, physical_snapshot_errors
    from server_test_preflight import process_rows
    owned_names = {'native_vehicle_control', 'cmd_vel_to_control', 'graph_path_bridge',
                   'planner_server', 'controller_server'}
    competitors = [r for r in process_rows() if r['domain'] == '79' and any(
        Path(t).name in owned_names for t in r['tokens'])]
    if competitors: raise RuntimeError('Prior motion clients remain: ' + str(competitors))
    report = dict(passed=False, errors=[], phases={}, fault_requests=[], map_reload_count=0,
                  vehicle_teleports=0, moving_test=False, video_recording_enabled=False)
    bng = BeamNGpy('127.0.0.1', 25252, socket_timeout=5).open(launch=False)
    node = native = relay = executor = worker = None
    records = []; phase = 'initial'; faults = {'control': False, 'poll': False}
    guard = threading.Lock(); origin = time.monotonic()
    try:
        report['before'] = read_physics_snapshot(bng)
        assert not physical_snapshot_errors(report['before']) and report['before']['speed'] < .05
        rclpy.init(args=['--ros-args', '-p', f'vehicle_port:={args.vehicle_port}',
            '-p', 'state_topic:=/wildos/freshness_probe_state',
            '-p', 'wildos_cmd_vel_to_control:control_topic:=/wildos/freshness_probe_control',
            '-p', 'cmd_vel_topic:=/wildos/freshness_probe_twist'])
        native = NativeVehicleControl(); relay = CmdVelToControl()
        node = rclpy.create_node('native_freshness_probe')
        executor = MultiThreadedExecutor(num_threads=4)
        for n in (native, relay, node): executor.add_node(n)
        brake = node.create_publisher(String, COMMAND_TOPIC, 10)
        brake_source = SourceControlSender()
        state = node.create_publisher(StateSensor, '/wildos/freshness_probe_state', 10)
        command = node.create_publisher(Twist, '/wildos/freshness_probe_twist', 10)
        def publish():
            brake.publish(String(data=json.dumps(brake_source.issue(
                dict(throttle=0., brake=1., steering=0., gear=0, parkingbrake=0., clutch=0.),
                time.time_ns(), time.monotonic_ns()))))
            msg = StateSensor(); msg.header.stamp = node.get_clock().now().to_msg()
            msg.header.frame_id = 'freshness_probe'; msg.dir.x = msg.up.z = 1.
            state.publish(msg)
            msg = Twist(); msg.linear.x = 1.2; msg.angular.z = .15; command.publish(msg)
        node.create_timer(.025, publish)
        def receive(kind, msg):
            records.append(dict(phase=phase, kind=kind, elapsed=time.monotonic()-origin,
                                data=json.loads(msg.data)))
        for topic, kind in (('/wildos/native_control_status', 'native'),
                            ('/wildos/control_status', 'consumer')):
            node.create_subscription(String, topic, lambda m, k=kind: receive(k, m), 10)
        control = native.actuator.control; poll = native.vehicle.sensors.poll
        def inject(name, operation, *a, **kw):
            began = time.monotonic()
            result = operation(*a, **kw)
            with guard:
                delay = faults[name]; faults[name] = False
            if delay:
                time.sleep(.65)
                report['fault_requests'].append(dict(phase=phase, operation=name,
                    duration_ms=(time.monotonic()-began)*1000))
            return result
        native.actuator.control = lambda **kw: inject('control', control, **kw)
        native.vehicle.sensors.poll = lambda: inject('poll', poll)
        worker = threading.Thread(target=executor.spin, daemon=True); worker.start()
        bng.control.resume()
        time.sleep(1.5)
        assert not node.get_subscriptions_info_by_topic('/wildos/freshness_probe_control'), 'Synthetic throttle has an actuator subscriber'
        assert all(r.successful for r in relay.set_parameters([Parameter('enable', value=True)]))
        for name, fault in (('baseline', None), ('delayed_control', 'control'),
                            ('control_recovered', None), ('delayed_poll', 'poll'),
                            ('poll_recovered', None)):
            phase = name
            if fault:
                with guard: faults[fault] = True
            time.sleep(1.6)
            rows = [r for r in records if r['phase'] == name]
            report['phases'][name] = rows
            natives = [r['data'] for r in rows if r['kind'] == 'native']
            consumers = [r['data'] for r in rows if r['kind'] == 'consumer']
            assert natives and consumers, 'Missing DDS samples: ' + name
            assert all(n['control_acknowledged'] and n['actuator_ok'] for n in natives)
            assert all(n['sent']['throttle'] == 0. and n['sent']['brake'] == 1. for n in natives)
            if fault:
                assert any(r['phase'] == name and r['operation'] == fault
                           and r['duration_ms'] > 650 for r in report['fault_requests']), name
                assert any(not n['telemetry_ok'] for n in natives), name
                if fault == 'control':
                    assert any((n.get('control_rpc_ms') or 0.) > 650
                               and not n['telemetry_ok'] for n in natives), name
                assert any(c['reason'] == 'native_control_unavailable' and c['brake'] == 1. for c in consumers), name
            else:
                assert consumers[-1]['reason'] == 'tracking' and consumers[-1]['native_health_ok'], name
        report['passed'] = True
    except BaseException as exc:
        report['errors'].append(type(exc).__name__ + ': ' + str(exc))
    finally:
        if relay:
            relay.enabled = False; relay.tick()
        if executor:
            executor.shutdown(timeout_sec=5.)
        if worker: worker.join(timeout=5.)
        if native:
            native.actuator.control = control
            native.vehicle.sensors.poll = poll
            try: native.stop()
            except Exception as exc: report['errors'].append('stop: ' + repr(exc))
        try:
            deadline = time.monotonic()+5.; quiet = None
            while True:
                snapshot = read_physics_snapshot(bng); now = time.monotonic()
                quiet = (now if quiet is None else quiet) if snapshot['speed'] < .05 else None
                if quiet is not None and now-quiet >= .5: break
                if now >= deadline: raise RuntimeError('Final vehicle stop not verified')
                time.sleep(.1)
            report['after'] = snapshot
            assert not physical_snapshot_errors(snapshot)
            assert snapshot['vehicleID'] == report['before']['vehicleID']
            assert snapshot['mission'] == report['before']['mission']
            assert math.dist(snapshot['position'], report['before']['position']) < .5
        except Exception as exc: report['errors'].append('final_physics: ' + repr(exc))
        try: bng.control.pause(); report['simulation_paused'] = True
        except Exception as exc: report['errors'].append('pause: ' + repr(exc))
        bng.disconnect()
        for n in (native, relay, node):
            if n: n.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
        report['passed'] = report['passed'] and not report['errors']
        report['elapsed'] = time.monotonic()-origin
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2), encoding='utf8')
    print(json.dumps(dict(passed=report['passed'], errors=report['errors'])))
    return 0 if report['passed'] else 1


if __name__ == '__main__': raise SystemExit(main())
