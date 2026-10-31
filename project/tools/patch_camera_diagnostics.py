"""Add bounded camera and sensor-frame diagnostics to the BeamNG ROS2 bridge."""
from __future__ import annotations

import argparse
import shutil
import time
from pathlib import Path


SENSORS_OLD = '''    def __init__(self, name: str, config: Dict[str, Any]) -> None:
        super().__init__(name, sensors.Camera, config)
        self._publishers: Dict[str, Publisher] = {}
'''
SENSORS_NEW = '''    def __init__(self, name: str, config: Dict[str, Any]) -> None:
        super().__init__(name, sensors.Camera, config)
        self._publishers: Dict[str, Publisher] = {}
        self._diag_polls = 0
'''

CAM_OLD = '''        if self.sensor.is_streaming:
            data = self.sensor.stream_raw()
        else:
            data = self.sensor.poll_raw()
        key = "colour"
'''
CAM_NEW = '''        if self.sensor.is_streaming:
            data = self.sensor.stream_raw()
        else:
            data = self.sensor.poll_raw()
        if self._diag_polls < 8:
            self._diag_polls += 1
            fields = {}
            for _key, _value in (data or {}).items():
                try:
                    fields[_key] = None if _value is None else len(_value)
                except TypeError:
                    fields[_key] = type(_value).__name__
            self._node.get_logger().info(
                f"camera_diag name={self.name} streaming={self.sensor.is_streaming} "
                f"render_colour={self.sensor.is_render_colours} "
                f"resolution={self.sensor.resolution} fields={fields}"
            )
        key = "colour"
'''

VEHICLE_OLD = '''        self.sensors: List[SensorPublisher] = ros_sensors

        self.vehicle.sensors.poll()
'''
VEHICLE_NEW = '''        self.sensors: List[SensorPublisher] = ros_sensors
        for _sensor in self.sensors:
            self.logger.info(
                f"sensor_diag name={_sensor.name} type={type(_sensor).__name__} "
                f"custom_frame={getattr(_sensor, 'custom_frame', None)!r} "
                f"config_keys={sorted(getattr(_sensor, 'config', {}).keys())}"
            )

        self.vehicle.sensors.poll()
'''


def patch_one(path: Path, old: str, new: str, tag: str) -> bool:
    text = path.read_text(encoding="utf-8")
    if tag in text:
        return False
    if text.count(old) != 1:
        raise RuntimeError(f"unexpected source for {path}: found {text.count(old)} matches")
    stamp = time.strftime("%Y%m%d-%H%M%S")
    shutil.copy2(path, path.with_name(path.name + f".pre_camera_diag_{stamp}.bak"))
    path.write_text(text.replace(old, new, 1), encoding="utf-8")
    return True


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--workspace", type=Path, required=True)
    args = parser.parse_args()
    root = args.workspace.expanduser().resolve() / "beamng_ros2" / "beamng_ros2"
    sensors = root / "publishers" / "sensors.py"
    vehicle = root / "vehicle.py"
    changed = []
    if patch_one(sensors, SENSORS_OLD, SENSORS_NEW, "self._diag_polls = 0"):
        changed.append(str(sensors))
    if patch_one(sensors, CAM_OLD, CAM_NEW, "camera_diag name="):
        changed.append(str(sensors))
    if patch_one(vehicle, VEHICLE_OLD, VEHICLE_NEW, "sensor_diag name="):
        changed.append(str(vehicle))
    print("camera diagnostics applied" if changed else "camera diagnostics already applied")
    for item in changed:
        print(item)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
