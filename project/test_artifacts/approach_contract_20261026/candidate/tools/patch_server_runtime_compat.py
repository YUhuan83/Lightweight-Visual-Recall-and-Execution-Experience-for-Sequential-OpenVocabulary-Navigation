"""Patch server dependency API/permission compatibility, preserving behavior."""
from __future__ import annotations

import argparse
import os
import shutil
import time
from pathlib import Path


def backup(path: Path, stamp: str):
    target = path.with_name(path.name + f".pre_runtime_compat_{stamp}.bak")
    if not target.exists():
        shutil.copy2(path, target)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--workspace", type=Path, required=True)
    args = parser.parse_args()
    workspace = args.workspace.expanduser().resolve()
    stamp = time.strftime("%Y%m%d-%H%M%S")

    bridge = workspace / "src" / "beamng-ros2-integration" / "beamng_ros2" / "beamng_ros2" / "utils.py"
    text = bridge.read_text(encoding="utf-8")
    patched = text.replace(".as_quat(False)", ".as_quat()")
    if patched != text:
        backup(bridge, stamp)
        bridge.write_text(patched, encoding="utf-8")

    vehicle = workspace / "src" / "beamng-ros2-integration" / "beamng_ros2" / "beamng_ros2" / "vehicle.py"
    vehicle_text = vehicle.read_text(encoding="utf-8")
    patched_vehicle = vehicle_text
    if "from beamng_msgs.msg import VehicleControl" not in patched_vehicle:
        patched_vehicle = patched_vehicle.replace(
            "import beamng_msgs.srv as srv\n",
            "import beamng_msgs.srv as srv\nfrom beamng_msgs.msg import VehicleControl\n",
        )
    if "self._control_sub = self.create_subscription" not in patched_vehicle:
        patched_vehicle = patched_vehicle.replace(
            "        self._create_publishers()\n        self._tf_broadcaster",
            "        self._create_publishers()\n"
            "        self._manual_control_enabled = False\n"
            "        self._control_sub = self.create_subscription(\n"
            "            VehicleControl, \"/control\", self._control_callback, 10)\n"
            "        self._tf_broadcaster",
        )
    if "def _control_callback(self, signal: VehicleControl):" not in patched_vehicle:
        patched_vehicle = patched_vehicle.replace(
            "    def health_check(self):\n",
            "    def _control_callback(self, signal: VehicleControl):\n"
            "        try:\n"
            "            if not self._manual_control_enabled:\n"
            "                self.vehicle.ai_set_mode(\"disable\")\n"
            "                self._manual_control_enabled = True\n"
            "            self.vehicle.control(\n"
            "                steering=signal.steering, throttle=signal.throttle,\n"
            "                brake=signal.brake, parkingbrake=signal.parkingbrake,\n"
            "                clutch=signal.clutch, gear=signal.gear)\n"
            "        except Exception as exc:\n"
            "            self.logger.error(f\"Vehicle control error [{self.vehicle.vid}]: {exc!r}\")\n\n"
            "    def health_check(self):\n",
        )
    if patched_vehicle != vehicle_text:
        backup(vehicle, stamp)
        vehicle.write_text(patched_vehicle, encoding="utf-8")

    dummy = workspace / "src" / "nebula2-wildos" / "graphnav_mapper" / "scripts" / "dummy_pose_graph_publisher.py"
    dummy_text = dummy.read_text(encoding="utf-8")
    dummy.write_text(dummy_text, encoding="utf-8", newline="\n")
    mode = dummy.stat().st_mode
    os.chmod(dummy, mode | 0o111)
    print("RUNTIME_COMPAT_OK", bridge, vehicle, oct(dummy.stat().st_mode & 0o777))


if __name__ == "__main__":
    main()
