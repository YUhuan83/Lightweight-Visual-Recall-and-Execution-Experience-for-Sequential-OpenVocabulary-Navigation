"""Repair cleanup/reconnect in an existing BeamNG ROS bridge source tree.

No simulator or network is accessed. Inspect with --check, apply locally with
--workspace PATH; a timestamped source backup is made before replacing a file.
Only sensor publishers already owned by that VehicleNode are removed.
"""
import argparse
import ast
from pathlib import Path
import shutil
import time

DESTROY = '''    def destroy_node(self):
        # wildos_owned_sensor_lifecycle_v1
        if self.destroyed:
            return False
        self.destroyed = True
        if self.timer:
            try:
                self.destroy_timer(self.timer)
            except Exception as exc:
                self.logger.warning(f"Could not stop vehicle timer: {exc!r}")
            self.timer = None
        for sensor in tuple(self.sensors):
            try:
                sensor.remove()
            except Exception as exc:
                # A failed camera close must not leak every later sensor.
                self.logger.warning(f"Owned sensor cleanup failed: {exc!r}")
        self.sensors = []
        if self._coupling_publisher:
            for cleanup in (self._stop_cosimulation, self._coupling_publisher.stop):
                try:
                    cleanup()
                except Exception as exc:
                    self.logger.warning(f"Cosimulation cleanup failed: {exc!r}")
            self._coupling_publisher = None
        try:
            return super().destroy_node()
        finally:
            try:
                self.vehicle.disconnect()
            except Exception as exc:
                self.logger.warning(f"Vehicle disconnect failed: {exc!r}")
'''

RECONNECT = '''    def _attempt_reconnect(self):
        # wildos_owned_sensor_lifecycle_v1
        if self.destroyed:
            return
        bng = self.node.beamng()
        host = self.node.get_parameter("host").get_parameter_value().string_value
        known_port = getattr(getattr(self.vehicle, "connection", None), "port", None)
        try:
            self.vehicle.disconnect()
        except Exception:
            pass
        try:
            from beamngpy.connection import Connection
            endpoint = bng.vehicles.start_connection(self.vehicle, self.vehicle.extensions)
            port = endpoint.get("result") if isinstance(endpoint, dict) else None
            if port is None:
                if known_port is None:
                    raise RuntimeError("No vehicle port or previously verified listener")
                port = known_port
            self.vehicle.connection = Connection(host, int(port), timeout=30.0)
            self.vehicle.connect(bng, tries=1)
            self.vehicle.sensors.poll()
            # Keep the same auto-sensor objects. Reconnecting the vehicle does
            # not authorize creating another camera/LiDAR with the same name.
            for sensor in self.sensors:
                sensor.poll()
            self._manual_control_enabled = False
            self.failures = 0
            self.logger.info(f"Reconnected vehicle {self.vehicle.vid}; owned sensors reused")
        except Exception as exc:
            self.logger.error(f"Reconnect failed for {self.vehicle.vid}: {exc!r}")
'''


def patched_source(source):
    tree = ast.parse(source)
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'VehicleNode')
    functions = {n.name: n for n in cls.body if isinstance(n, ast.FunctionDef)}
    lines = source.splitlines(keepends=True)
    for name, body in sorted((('destroy_node', DESTROY), ('_attempt_reconnect', RECONNECT)),
                             key=lambda item: functions[item[0]].lineno, reverse=True):
        function = functions[name]
        lines[function.lineno-1:function.end_lineno] = [body]
    result = ''.join(lines)
    ast.parse(result)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workspace', type=Path, required=True,
                        help='beamng-ros2-integration repository directory')
    parser.add_argument('--check', action='store_true')
    args = parser.parse_args()
    path = args.workspace.expanduser().resolve()/'beamng_ros2/beamng_ros2/vehicle.py'
    source = path.read_text(encoding='utf8')
    replacement = patched_source(source)
    if source == replacement:
        print('Owned sensor lifecycle repair already applied')
    elif args.check:
        print('Compatible source; cleanup/reconnect repair required')
    else:
        backup = path.with_name(path.name + '.pre_sensor_lifecycle_' + time.strftime('%Y%m%d-%H%M%S') + '.bak')
        if not backup.exists():
            shutil.copy2(path, backup)
        path.write_text(replacement, encoding='utf8')
        print('Applied owned sensor cleanup/reconnect repair:', path)


if __name__ == '__main__':
    main()
