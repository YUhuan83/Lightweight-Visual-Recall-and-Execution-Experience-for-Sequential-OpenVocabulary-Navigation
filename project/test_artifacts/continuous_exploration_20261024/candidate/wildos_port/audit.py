"""Static dependency inventory; never import ROS/CUDA or install anything."""
import argparse
import ast
import importlib.metadata
import json
import os
from pathlib import Path
import re
import shutil
import sys
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parent


def audit(source):
    packages, dependencies, cmake = {}, set(), set()
    for path in sorted(source.rglob("package.xml")):
        tree = ET.parse(path).getroot()
        packages[tree.findtext("name")] = path.relative_to(source).as_posix()
        dependencies.update(e.text for e in tree if e.tag.endswith("depend") and e.text)
    for path in source.rglob("CMakeLists.txt"):
        text = re.sub(r"#[^\n]*", "", path.read_text(encoding="utf-8"))
        cmake.update(re.findall(r"find_package\(\s*([^\s)]+)", text))
    prefixes = [Path(p) for p in os.environ.get("AMENT_PREFIX_PATH", "").split(os.pathsep) if p]
    required_ros = {"rclpy", "cv_bridge", "sensor_msgs_py", "message_filters", "tf2_ros",
                    "ament_index_python", "nav_msgs", "sensor_msgs", "geometry_msgs",
                    "visualization_msgs", "launch_ros"}
    availability = {}
    for name in sorted(dependencies | cmake | required_ros):
        if name in packages:
            availability[name] = "source present; NOT built"
        elif any((p / "share/ament_index/resource_index/packages" / name).is_file() for p in prefixes):
            availability[name] = "ament index present; runtime NOT tested"
        else:
            availability[name] = "not found in source/AMENT_PREFIX_PATH; system/CMake availability unverified"
    python_deps = {}
    requirements = source / "requirements.txt"
    names = [line.strip() for line in requirements.read_text().splitlines()
             if line.strip() and not line.lstrip().startswith("#")]
    names += ["numpy", "scipy", "shapely", "scikit-learn", "Pillow", "plotly"]
    for spec in names:
        name = re.split(r"[<=>\[]", spec)[0]
        try:
            version = importlib.metadata.version(name)
            python_deps[spec] = "installed " + version
            if "==" in spec and version != spec.split("==")[1]:
                python_deps[spec] += " (PIN MISMATCH)"
        except importlib.metadata.PackageNotFoundError:
            python_deps[spec] = "missing in audit interpreter"
    imports = set()
    for path in source.rglob("*.py"):
        for node in ast.walk(ast.parse(path.read_bytes(), filename=str(path))):
            if isinstance(node, ast.Import):
                imports.update(alias.name.split(".")[0] for alias in node.names)
            elif isinstance(node, ast.ImportFrom) and node.module and not node.level:
                imports.add(node.module.split(".")[0])
    return {"python": sys.version, "platform": sys.platform,
            "tools": {name: shutil.which(name) for name in ("ros2", "colcon", "cmake", "g++", "git")},
            "source_packages": packages, "dependency_status": availability,
            "python_distribution_status": python_deps, "absolute_import_roots": sorted(imports),
            "blockers": [
                "Humble build and Python 3.10 / CUDA compatibility NOT tested on this Windows host",
                "graaf_vendor wrapper present; Graaf v1.1.1 headers absent, FetchContent would access network at build",
                "grid_map_core/grid_map_ros/grid_map_msgs, Eigen3, OpenCV, nanoflann require real target dependencies",
                "Python package manifests are incomplete; import roots are inventory, not an install recipe",
                "No LIO, real pose graph, elevation/unknown/traversability map producer provided",
                "Original mapper launch starts dummy pose graph and gdb; do NOT use as production bringup",
                "Planner needs C++20; mapper now explicitly requires C++17",
                "No BeamNG CameraInfo/extrinsic calibration verified; proposed frame contract only",
                "No weights loaded/copied; local RADIO filename must be configured explicitly"]}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, default=ROOT / "workspace/src/nebula2-wildos")
    parser.add_argument("--output", type=Path, default=ROOT / "dependency_audit.json")
    args = parser.parse_args()
    from prepare import safe_path
    output = safe_path(args.output)
    if not output.is_relative_to(ROOT):
        raise ValueError("Audit output must remain inside wildos_port")
    report = audit(args.source)
    output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(f"Audited {len(report['source_packages'])} ROS source packages; report: {output}")
