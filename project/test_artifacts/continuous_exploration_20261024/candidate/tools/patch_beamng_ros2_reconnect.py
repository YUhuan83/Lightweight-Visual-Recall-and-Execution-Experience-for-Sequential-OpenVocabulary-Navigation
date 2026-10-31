"""Finish BeamNG ROS2 bridge lifecycle handling: reconnect + relaxed guard.

Applies to a source tree that already received patch_beamng_ros2_bridge.py.
Makes timestamped backups and refuses to patch unexpected source text.
"""
from __future__ import annotations

import argparse
import shutil
import time
from pathlib import Path


def replace_once(text: str, old: str, new: str, label: str) -> str:
    if text.count(old) != 1:
        raise RuntimeError(f"{label}: expected one source match")
    return text.replace(old, new, 1)


def patch_beamng(path: Path) -> None:
    text = path.read_text(encoding="utf-8")
    old = (
        "        try:\n"
        "            game_state = beamng.control.get_gamestate()\n"
        "            if game_state.get(\"state\") == \"scenario\":\n"
        "                self.logger.error(\n"
        "                    \"BeamNG already has an active scenario; restart it before StartScenario.\"\n"
        "                )\n"
        "                return response\n"
        "        except Exception as exc:\n"
        "            self.logger.error(f\"Could not verify BeamNG scenario state: {exc}\")\n"
        "            return response\n"
    )
    new = (
        "        try:\n"
        "            game_state = beamng.control.get_gamestate()\n"
        "            if game_state.get(\"state\") == \"scenario\":\n"
        "                self.logger.warn(\n"
        "                    \"BeamNG already has a loaded scenario; loading the requested one anyway.\"\n"
        "                )\n"
        "        except Exception as exc:\n"
        "            self.logger.error(f\"Could not verify BeamNG scenario state: {exc}\")\n"
    )
    path.write_text(replace_once(text, old, new, "relaxed scenario guard"), encoding="utf-8")


def patch_vehicle(path: Path) -> None:
    text = path.read_text(encoding="utf-8")
    old = (
        "            if self._visualizer:\n"
        "                self._visualizer.publish(time)\n"
        "        except Exception as e:\n"
        "            self.logger.error(f\"Fatal vehicle connection error [{self.vehicle.vid}]: {str(e)}\")\n"
        "            if self.timer:\n"
        "                self.destroy_timer(self.timer)\n"
        "                self.timer = None\n"
    )
    new = (
        "            if self._visualizer:\n"
        "                self._visualizer.publish(time)\n"
        "            self.failures = 0\n"
        "        except Exception as e:\n"
        "            self.failures = getattr(self, \"failures\", 0) + 1\n"
        "            self.logger.error(\n"
        "                f\"Vehicle polling error [{self.vehicle.vid}] #{self.failures}: {e!r}\"\n"
        "            )\n"
        "            try:\n"
        "                import traceback\n"
        "                with open(\"/tmp/beamng_bridge_vehicle_errors.log\", \"a\", encoding=\"utf-8\") as handle:\n"
        "                    handle.write(f\"--- failure #{self.failures} {self.vehicle.vid}\\n\")\n"
        "                    handle.write(traceback.format_exc())\n"
        "            except Exception:\n"
        "                pass\n"
        "            if self.failures in (5, 20, 60):\n"
        "                self._attempt_reconnect()\n"
    )
    text = replace_once(text, old, new, "reconnect error handler")

    marker = "    def _start_cosimulation(self, path_to_cosim_definition: str):\n"
    reconnect = (
        "    def _attempt_reconnect(self):\n"
        "        \"\"\"Re-establish the BeamNG vehicle socket after a polling failure.\"\"\"\n"
        "        bng = self.node.beamng()\n"
        "        host = self.node.get_parameter(\"host\").get_parameter_value().string_value\n"
        "        # BeamNG.tech 0.39.4 may omit result after vehicle respawn.\n"
        "        # Preserve only this vehicle's known listener as a fallback.\n"
        "        known_port = getattr(getattr(self.vehicle, \"connection\", None), \"port\", None)\n"
        "        try:\n"
        "            self.vehicle.disconnect()\n"
        "        except Exception:\n"
        "            pass\n"
        "        try:\n"
        "            from beamngpy.connection import Connection\n"
        "            endpoint = bng.vehicles.start_connection(self.vehicle, self.vehicle.extensions)\n"
        "            port = endpoint.get(\"result\") if isinstance(endpoint, dict) else None\n"
        "            if port is None:\n"
        "                if known_port is None:\n"
        "                    raise RuntimeError(\"BeamNG returned no vehicle port and no known listener exists\")\n"
        "                port = known_port\n"
        "                self.logger.warning(\"BeamNG returned no vehicle port; reusing known listener %s\", port)\n"
        "            self.vehicle.connection = Connection(host, int(port), timeout=30.0)\n"
        "            self.vehicle.connect(bng, tries=1)\n"
        "            self.vehicle.sensors.poll()\n"
        "            self.failures = 0\n"
        "            self.logger.info(f\"Reconnected vehicle {self.vehicle.vid}\")\n"
        "        except Exception as exc:\n"
        "            self.logger.error(\n"
        "                f\"Reconnect failed for {self.vehicle.vid}: {exc!r}\"\n"
        "            )\n\n"
    )
    text = replace_once(text, marker, reconnect + marker, "reconnect helper")
    path.write_text(text, encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--workspace",
        type=Path,
        default=Path.home() / "beamng_ros2_0394_ws" / "src" / "beamng-ros2-integration",
    )
    args = parser.parse_args()
    root = args.workspace.expanduser().resolve() / "beamng_ros2" / "beamng_ros2"
    beamng, vehicle = root / "beamng.py", root / "vehicle.py"
    for path in (beamng, vehicle):
        if not path.is_file():
            raise FileNotFoundError(path)
    if "_attempt_reconnect" in vehicle.read_text(encoding="utf-8"):
        print("reconnect patch already applied")
        return 0
    stamp = time.strftime("%Y%m%d-%H%M%S")
    for path in (beamng, vehicle):
        shutil.copy2(path, path.with_name(path.name + f".pre_reconnect_{stamp}.bak"))
    patch_beamng(beamng)
    patch_vehicle(vehicle)
    print(f"reconnect + relaxed-guard patch applied under {root}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
