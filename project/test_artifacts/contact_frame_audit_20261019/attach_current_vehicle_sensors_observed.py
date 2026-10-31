"""Attach the saved rig to the current physical vehicle, without loading a map.

The vehicle endpoint is requested from the current simulator instance. This
process owns only the sensors it creates and removes them on shutdown.
"""
import argparse
import copy
import json
import math
import os
from pathlib import Path
import select
import signal
import sys
import threading

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'nav2_adapter'))


class EngineLifetime:
    """Pin one local engine incarnation; a recycled PID/port is not its owner."""

    def __init__(self, pid, port):
        # pidfd identifies the process itself, even if /proc later reuses PID.
        self.lock = threading.Lock()
        self.fd = os.pidfd_open(pid)
        try:
            self.poller = select.poll()
            self.poller.register(self.fd, select.POLLIN)
            self.require_alive()
            tokens = Path(f'/proc/{pid}/cmdline').read_bytes().decode().split('\0')
            if (Path(tokens[0]).name != 'BeamNG.tech.x64' or '-tport' not in tokens
                    or tokens[tokens.index('-tport') + 1] != str(port)):
                raise ValueError('Engine PID does not identify the requested BeamNG listener')
            self.require_alive()
        except BaseException:
            self.close()
            raise

    def alive(self):
        # GE/VE SDK requests and the lifetime loop run on different threads.
        # CPython rejects concurrent poll() on the same poll object.
        with self.lock:
            return self.fd is not None and not self.poller.poll(0)

    def require_alive(self):
        if not self.alive():
            raise ConnectionError('Owning BeamNG engine exited; refusing another incarnation')

    def close(self):
        with self.lock:
            if self.fd is not None:
                os.close(self.fd)
                self.fd = None


def guard_engine_requests(connection_class, lifetime):
    """All SDK requests, including removal/reconnect, retain engine ownership."""
    original = connection_class.send

    def send(connection, data):
        lifetime.require_alive()
        return original(connection, data)

    connection_class.send = send
    return original


def current_endpoint(response, vehicle_id):
    if not isinstance(response, dict) or response.get('vid') != vehicle_id:
        raise ValueError('Vehicle endpoint belongs to another or missing vehicle')
    value = response.get('result')
    if (isinstance(value, bool) or not isinstance(value, (int, float))
            or not math.isfinite(value) or value != int(value) or not 1 <= value <= 65535):
        raise ValueError('Current vehicle did not return a valid TCP endpoint')
    return int(value)


def rig_for_vehicle(document, vehicle_id):
    matches = [v for v in document.get('vehicles', []) if v.get('name') == vehicle_id]
    if len(matches) != 1:
        raise ValueError('Rig must identify exactly one current vehicle')
    rig = copy.deepcopy(matches[0].get('sensors', []))
    names = [s.get('name') for s in rig]
    if not all(isinstance(n, str) and n for n in names) or len(set(names)) != len(names):
        raise ValueError('Sensor names must be unique and nonempty')
    types = [s.get('type', '') for s in rig]
    if not any(t.startswith('Camera.') for t in types) or not any(t.startswith('Lidar.') for t in types):
        raise ValueError('Saved rig must contain a camera and LiDAR')
    return rig


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--rig', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=25252)
    parser.add_argument('--vehicle', default='ego')
    parser.add_argument('--engine-pid', type=int, required=True,
                        help='PID of the local BeamNG instance owning this rig')
    parser.add_argument('--resume', action='store_true')
    args = parser.parse_args()
    if args.vehicle != 'ego':
        parser.error('Physical snapshot contract currently checks ego only')
    if args.host not in ('127.0.0.1', 'localhost', '::1'):
        parser.error('Engine PID ownership requires a local BeamNG host')
    if args.output.exists():
        raise FileExistsError('Use a new evidence path')
    document = json.loads(args.rig.read_text())
    rig = rig_for_vehicle(document, args.vehicle)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    report = dict(ready=False, map_reload_count=0, rig=str(args.rig), sensors=copy.deepcopy(rig))
    stopped = threading.Event()
    for sig in (signal.SIGTERM, signal.SIGINT):
        signal.signal(sig, lambda *_: stopped.set())
    bridge = vehicle = None
    owned = []
    node_owns_sensors = False
    exit_code = 1
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from rclpy.parameter import Parameter
    from beamng_ros2.beamng import BeamNGBridge
    from beamng_ros2.scenario_loader import decode_sensors
    from beamngpy.connection import Connection
    from beamngpy.connection.prefixed_length_socket import PrefixedLengthSocket
    from wildos_nav2_adapter.fail_closed_transport import install_fail_closed_reads
    from wildos_nav2_adapter.physical_vehicle import read_physics_snapshot, physical_snapshot_errors
    from restore_physical_vehicle import active_vehicle_clients
    install_fail_closed_reads(PrefixedLengthSocket)
    lifetime = EngineLifetime(args.engine_pid, args.port)
    original_send = guard_engine_requests(Connection, lifetime)
    report['engine_pid'] = args.engine_pid
    # Our handlers request ordered teardown. ROS's default SIGTERM handler
    # would invalidate its context before BeamNGBridge cleans owned nodes.
    initialized = False
    try:
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        initialized = True
        clients = active_vehicle_clients()
        report['prior_vehicle_clients'] = clients
        if clients:
            raise RuntimeError('Stop existing sensor/motion clients before attaching a rig')
        bridge = BeamNGBridge()
        bridge.set_parameters([Parameter('launch', value=False),
            Parameter('host', value=args.host), Parameter('port', value=args.port)])
        bng = bridge.beamng()
        before = read_physics_snapshot(bng)
        report['before'] = before
        errors = physical_snapshot_errors(before, require_sensors=False)
        if errors or before.get('speed', 99.) > .15:
            raise RuntimeError('Stationary original physics required: ' + '; '.join(errors))
        if before.get('cameras') or before.get('lidars'):
            raise RuntimeError('Existing sensor rig present; refusing to replace foreign sensors')
        vehicle = bng.vehicles.get_current()[args.vehicle]
        response = bng.vehicles.start_connection(vehicle, vehicle.extensions)
        report['endpoint_response'] = response
        port = current_endpoint(response, vehicle.vid)
        vehicle.connection = Connection(args.host, port, timeout=6)
        vehicle.connect(bng, tries=1)
        owned = decode_sensors(rig)
        for sensor in owned:
            sensor.pre_scenario_start(vehicle)
            sensor.post_scenario_start(vehicle)
        bridge._spawn_vehicle_node(vehicle, dict(ros_sensors=owned))
        node_owns_sensors = True
        node = bridge._vehicles[args.vehicle]
        # Read the bridge's already established clock anchor; do not reset it.
        report['bridge_start_time_ros_ns'] = int(node.start_time.nanoseconds)
        report['bridge_delay_offset_sec'] = float(node.DELAY_OFFSET_SEC)
        report['clock_anchor_observation'] = 'Read existing vehicle node after initialization; no clock change'

        node.set_parameters([Parameter('update_sec', value=.1)])
        node.destroy_timer(node.timer)
        node.reset_timer()
        node.health_check()
        after = read_physics_snapshot(bng)
        report['after'] = after
        errors = physical_snapshot_errors(after)
        if after['vehicleID'] != before['vehicleID'] or after['mission'] != before['mission']:
            errors.append('Vehicle instance or loaded map changed during attachment')
        if errors:
            raise RuntimeError('; '.join(errors))
        bridge.mt_executor.add_node(bridge)
        bridge.running = True
        bridge.executor_thread.start()
        if args.resume:
            bng.control.resume()
        report.update(ready=True, resumed=args.resume, vehicle_port=port)
        args.output.write_text(json.dumps(report, indent=2), encoding='utf-8')
        print(json.dumps(dict(ready=True, vehicle_port=port, map_reload_count=0)), flush=True)
        while rclpy.ok() and not stopped.wait(.2):
            lifetime.require_alive()
        exit_code = 0
    except Exception as exc:
        report['error'] = f'{type(exc).__name__}: {exc}'[:500]
        print(report['error'], flush=True)
    finally:
        # Stop polling before disconnect/removal; a timer must not reuse an
        # owned sensor after it has been closed. Leave the map and car intact.
        if bridge is not None:
            try:
                bridge.mt_executor.shutdown(timeout_sec=10.)
                if bridge.executor_thread.is_alive():
                    bridge.executor_thread.join(timeout=10.)
            except Exception as exc:
                report['executor_cleanup_error'] = str(exc)[:240]
                exit_code = 1
        if not node_owns_sensors:
            for sensor in owned:
                try:
                    sensor.remove()
                except Exception as exc:
                    report.setdefault('sensor_cleanup_errors', []).append(str(exc)[:240])
                    exit_code = 1
            if vehicle is not None:
                vehicle.disconnect()
        if bridge is not None:
            try:
                bridge.destroy_node()
            except Exception as exc:
                report['bridge_cleanup_error'] = str(exc)[:240]
                exit_code = 1
        if initialized and rclpy.ok():
            rclpy.shutdown()
        report['stopped'] = True
        report['owning_engine_alive_at_cleanup'] = lifetime.alive()
        args.output.write_text(json.dumps(report, indent=2), encoding='utf-8')
        Connection.send = original_send
        lifetime.close()
    return exit_code


if __name__ == '__main__':
    raise SystemExit(main())
