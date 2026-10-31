"""Upgrade an existing bridge reconnect helper for BeamNG.tech 0.39.4."""
from pathlib import Path
import argparse
import shutil
import time


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--workspace', type=Path, required=True)
    args = parser.parse_args()
    root = args.workspace.expanduser().resolve() / 'beamng_ros2' / 'beamng_ros2'
    vehicle = root / 'vehicle.py'
    if not vehicle.is_file():
        raise FileNotFoundError(vehicle)
    text = vehicle.read_text(encoding='utf-8')
    if 'known_port = getattr(getattr(self.vehicle' in text:
        print('v2 reconnect fallback already applied')
        return 0
    start = text.find('    def _attempt_reconnect(self):\n')
    end = text.find('    def _start_cosimulation(', start)
    if start < 0 or end < 0:
        raise RuntimeError('Existing reconnect helper not found')
    old = text[start:end]
    if 'start_connection(self.vehicle' not in old:
        raise RuntimeError('Unexpected reconnect helper body')
    new = '''    def _attempt_reconnect(self):
        """Re-establish the BeamNG vehicle socket after a polling failure."""
        bng = self.node.beamng()
        host = self.node.get_parameter("host").get_parameter_value().string_value
        # BeamNG.tech 0.39.4 may omit `result` after a vehicle respawn.
        # Reuse only this vehicle object's known listener; never guess a port.
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
                    raise RuntimeError("BeamNG returned no vehicle port and no known listener exists")
                port = known_port
                self.logger.warning("BeamNG returned no vehicle port; reusing known listener %s", port)
            self.vehicle.connection = Connection(host, int(port), timeout=30.0)
            self.vehicle.connect(bng, tries=1)
            self.vehicle.sensors.poll()
            self.failures = 0
            self.logger.info(f"Reconnected vehicle {self.vehicle.vid}")
        except Exception as exc:
            self.logger.error(
                f"Reconnect failed for {self.vehicle.vid}: {exc!r}"
            )

'''
    stamp = time.strftime('%Y%m%d-%H%M%S')
    shutil.copy2(vehicle, vehicle.with_name(vehicle.name + f'.pre_reconnect_v2_{stamp}.bak'))
    vehicle.write_text(text[:start] + new + text[end:], encoding='utf-8')
    print(f'v2 reconnect fallback applied under {root}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
