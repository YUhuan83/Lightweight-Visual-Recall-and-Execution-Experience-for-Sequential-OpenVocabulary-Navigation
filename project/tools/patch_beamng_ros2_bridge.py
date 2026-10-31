"""Patch BeamNG ROS2 bridge lifecycle handling on the Linux host."""
from __future__ import annotations

import argparse
import shutil
import time
from pathlib import Path


def replace_once(text: str, old: str, new: str, label: str) -> str:
    if text.count(old) != 1:
        raise RuntimeError(f"{label}: expected one source match")
    return text.replace(old, new, 1)


def patch_bridge(root: Path) -> None:
    beamng = root / "beamng_ros2" / "beamng_ros2" / "beamng.py"
    vehicle = root / "beamng_ros2" / "beamng_ros2" / "vehicle.py"
    if not beamng.is_file() or not vehicle.is_file():
        raise FileNotFoundError("bridge source files not found")

    stamp = time.strftime("%Y%m%d-%H%M%S")
    for path in (beamng, vehicle):
        shutil.copy2(path, path.with_name(path.name + f".pre_wildos_{stamp}.bak"))

    text = beamng.read_text(encoding="utf-8")
    old = (
        "        if self.running:\n"
        "            self._cleanup_scenario()\n\n"
        "        try:\n"
        "            json_scenario = load_json(request.path_to_scenario_definition)\n"
    )
    new = (
        "        if self.running or self._vehicles:\n"
        "            self.logger.error(\n"
        "                \"Refusing duplicate StartScenario while a vehicle scenario is active.\"\n"
        "            )\n"
        "            return response\n\n"
        "        try:\n"
        "            game_state = beamng.control.get_gamestate()\n"
        "            if game_state.get(\"state\") == \"scenario\":\n"
        "                self.logger.error(\n"
        "                    \"BeamNG already has an active scenario; restart it before StartScenario.\"\n"
        "                )\n"
        "                return response\n"
        "        except Exception as exc:\n"
        "            self.logger.error(f\"Could not verify BeamNG scenario state: {exc}\")\n"
        "            return response\n\n"
        "        try:\n"
        "            json_scenario = load_json(request.path_to_scenario_definition)\n"
    )
    text = replace_once(text, old, new, "duplicate scenario guard")

    old = (
        "        self.publisher_callback_beamng()\n"
        "        self.executor_thread.start()\n"
        "        self.logger.info(f'Started scenario \"{scenario.name}\".')\n"
        "        self.running = True\n\n"
        "        response.success = True\n"
        "        return response\n"
    )
    new = (
        "        try:\n"
        "            for node in self._vehicles.values():\n"
        "                node.health_check()\n"
        "        except Exception as exc:\n"
        "            self.logger.error(f\"Scenario sensor health check failed: {exc}\")\n"
        "            self._cleanup_scenario()\n"
        "            response.success = False\n"
        "            return response\n\n"
        "        self.publisher_callback_beamng()\n"
        "        self.executor_thread.start()\n"
        "        self.logger.info(f'Started scenario \"{scenario.name}\".')\n"
        "        self.running = True\n\n"
        "        response.success = True\n"
        "        return response\n"
    )
    text = replace_once(text, old, new, "scenario health check")
    beamng.write_text(text, encoding="utf-8")

    text = vehicle.read_text(encoding="utf-8")
    old = (
        "    def reset_timer(self):\n"
        "        self.timer = self._create_vehicle_timer()\n\n"
        "    def _create_vehicle_timer(self):\n"
    )
    new = (
        "    def health_check(self):\n"
        "        if not self.vehicle.is_connected():\n"
        "            raise RuntimeError(f\"vehicle {self.vehicle.vid} is not connected\")\n"
        "        self.vehicle.sensors.poll()\n"
        "        state = dict(self.vehicle.state)\n"
        "        if not state.get(\"pos\") or \"time\" not in state:\n"
        "            raise RuntimeError(f\"vehicle {self.vehicle.vid} returned incomplete state\")\n"
        "        for sensor in self.sensors:\n"
        "            sensor.poll()\n\n"
        "    def reset_timer(self):\n"
        "        self.timer = self._create_vehicle_timer()\n\n"
        "    def _create_vehicle_timer(self):\n"
    )
    text = replace_once(text, old, new, "vehicle health check")

    old = (
        "        except Exception as e:\n"
        "            self.logger.error(f\"Fatal error [{self.vehicle.vid}]: {str(e)}\")\n"
    )
    new = (
        "        except Exception as e:\n"
        "            self.logger.error(f\"Fatal vehicle connection error [{self.vehicle.vid}]: {str(e)}\")\n"
        "            if self.timer:\n"
        "                self.destroy_timer(self.timer)\n"
        "                self.timer = None\n"
    )
    text = replace_once(text, old, new, "vehicle timer stop")
    vehicle.write_text(text, encoding="utf-8")
    print("bridge patched; backups created beside source files")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--workspace",
        type=Path,
        default=Path.home() / "beamng_ros2_0394_ws" / "src" / "beamng-ros2-integration",
    )
    patch_bridge(parser.parse_args().workspace.expanduser().resolve())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
