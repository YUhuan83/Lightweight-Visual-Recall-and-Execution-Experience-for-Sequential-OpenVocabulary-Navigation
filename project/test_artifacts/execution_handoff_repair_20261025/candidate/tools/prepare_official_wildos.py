"""Prepare upstream WildOS for ROS2 build without changing its algorithms.

Only fixes Python packaging metadata and installs BeamNG deployment configs.
Every modified upstream file receives a timestamped backup.
"""
from __future__ import annotations

import argparse
import os
import shutil
import time
from pathlib import Path


def backup(path: Path, stamp: str) -> None:
    target = path.with_name(path.name + f".pre_beamng_{stamp}.bak")
    if not target.exists():
        shutil.copy2(path, target)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source-root", type=Path, required=True)
    parser.add_argument("--adapter-root", type=Path, required=True)
    args = parser.parse_args()
    root = args.source_root.expanduser().resolve()
    adapter = args.adapter_root.expanduser().resolve()
    visual = root / "visual_navigation"
    setup = visual / "setup.py"
    package = visual / "package.xml"
    if not setup.is_file() or not package.is_file():
        raise FileNotFoundError("visual_navigation package missing")
    stamp = time.strftime("%Y%m%d-%H%M%S")

    text = setup.read_text(encoding="utf-8")
    if "find_namespace_packages" not in text:
        backup(setup, stamp)
        text = text.replace(
            "from setuptools import find_packages, setup",
            "from setuptools import find_namespace_packages, setup",
        )
        text = text.replace(
            "packages=find_packages(exclude=['test']),",
            "packages=find_namespace_packages(include=['visual_navigation*'], exclude=['test*']),",
        )
        setup.write_text(text, encoding="utf-8")

    xml = package.read_text(encoding="utf-8")
    if "<buildtool_depend>ament_python</buildtool_depend>" not in xml:
        backup(package, stamp)
        xml = xml.replace(
            "  <license>Apache-2.0</license>\n",
            "  <license>Apache-2.0</license>\n\n  <buildtool_depend>ament_python</buildtool_depend>\n",
        )
        package.write_text(xml, encoding="utf-8")

    planner_package = root / "graphnav_planner" / "package.xml"
    planner_xml = planner_package.read_text(encoding="utf-8")
    if "<depend>grid_map_msgs</depend>" not in planner_xml:
        backup(planner_package, stamp)
        planner_xml = planner_xml.replace(
            "  <depend>graphnav_msgs</depend>\n",
            "  <depend>graphnav_msgs</depend>\n  <depend>grid_map_msgs</depend>\n",
        )
        planner_package.write_text(planner_xml, encoding="utf-8")

    planner_header = root / "graphnav_planner" / "include" / "graphnav_planner" / "planner.hpp"
    planner_text = planner_header.read_text(encoding="utf-8")
    patched_planner_text = planner_text
    if "#include <queue>" not in patched_planner_text:
        patched_planner_text = patched_planner_text.replace(
            "#include <iomanip>\n", "#include <iomanip>\n#include <queue>\n")
    patched_planner_text = patched_planner_text.replace(
        "float new_dist = dist_map_(x, y) + cost[dir] * resolution_;",
        "float new_dist = dist_map_.coeff(x, y) + cost[dir] * resolution_;",
    )
    if patched_planner_text != planner_text:
        backup(planner_header, stamp)
        planner_header.write_text(patched_planner_text, encoding="utf-8")

    wildos_nav = visual / "visual_navigation" / "wildos" / "nav.py"
    wildos_text = wildos_nav.read_text(encoding="utf-8")
    marker = "Object similarity min="
    if marker not in wildos_text:
        backup(wildos_nav, stamp)
        wildos_text = wildos_text.replace(
            "            if np.sum(binary_mask) > 0:\n",
            "            self.get_logger().info(\n"
            "                f\"Object similarity min={float(np.min(text_sim_spatial)):.6f} \"\n"
            "                f\"max={float(np.max(text_sim_spatial)):.6f} \"\n"
            "                f\"mask_pixels={int(np.sum(binary_mask))} threshold={self.mask_threshold:.6f}\")\n"
            "            if np.sum(binary_mask) > 0:\n",
        )
        wildos_nav.write_text(wildos_text, encoding="utf-8")

    configs = visual / "configs"
    configs.mkdir(parents=True, exist_ok=True)
    for name in ("beamng_wildos.yaml", "beamng_triangulation.yaml"):
        source = adapter / "config" / name
        if not source.is_file():
            raise FileNotFoundError(source)
        shutil.copy2(source, configs / name)

    dummy = root / "graphnav_mapper" / "scripts" / "dummy_pose_graph_publisher.py"
    dummy_text = dummy.read_text(encoding="utf-8")
    dummy.write_text(dummy_text, encoding="utf-8", newline="\n")
    os.chmod(dummy, dummy.stat().st_mode | 0o111)

    print("official WildOS prepared")
    print("setup:", setup)
    print("configs:", configs / "beamng_wildos.yaml", configs / "beamng_triangulation.yaml")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
