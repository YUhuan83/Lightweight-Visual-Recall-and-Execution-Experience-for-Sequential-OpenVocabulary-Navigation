from pathlib import Path

from setuptools import Extension, find_packages, setup

root = Path(__file__).resolve().parents[2]
ground_extension = Extension(
    "wildos_nav2_adapter._ground",
    sources=["wildos_nav2_adapter/ground.cpp"],
    include_dirs=[
        str(root / "vendor" / "pybind11" / "include"),
        "/usr/include/eigen3",
    ],
    language="c++",
    extra_compile_args=[
        "-O3",
        "-std=c++17",
        "-DEIGEN_DONT_PARALLELIZE",
        "-Wno-maybe-uninitialized",
    ],
)

setup(
    name="wildos_nav2_adapter",
    version="0.2.0",
    packages=find_packages(),
    ext_modules=[ground_extension],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/wildos_nav2_adapter"]),
        ("share/wildos_nav2_adapter", ["package.xml"]),
        ("share/wildos_nav2_adapter/launch", ["launch/cleanroom.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=False,
    entry_points={
        "console_scripts": [
            "graph_path_bridge = wildos_nav2_adapter.graph_path_bridge:main",
            "cmd_vel_to_control = wildos_nav2_adapter.cmd_vel_to_control:main",
            "native_vehicle_control = wildos_nav2_adapter.native_vehicle_control:main",
            "cloud_filter = wildos_nav2_adapter.cloud_filter:main",
        ]
    },
)
