import os
from glob import glob

from setuptools import find_packages, setup

package_name = "wildos_nav2_adapter"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    package_data={package_name: ['vehicle_control_lease.lua']},
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "config"), glob("config/*.xml")),
        (os.path.join("share", package_name, "config"), glob("config/*.rviz")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    entry_points={
        "console_scripts": [
            "local_dds_run = wildos_nav2_adapter.dds_profile:main",
            "state_odom = wildos_nav2_adapter.state_odom:main",
            "goal_adapter = wildos_nav2_adapter.goal_adapter:main",
            "cloud_filter = wildos_nav2_adapter.cloud_filter:main",
            "path_planner_bridge = wildos_nav2_adapter.path_planner_bridge:main",
            "graph_path_bridge = wildos_nav2_adapter.graph_path_bridge:main",
            "cmd_vel_to_control = wildos_nav2_adapter.cmd_vel_to_control:main",
            "native_vehicle_control = wildos_nav2_adapter.native_vehicle_control:main",
            "kinematic_vehicle_control = wildos_nav2_adapter.kinematic_vehicle_control:main",
            "odom_tf_broadcaster = wildos_nav2_adapter.odom_tf_broadcaster:main",
            "visualization_bridge = wildos_nav2_adapter.visualization_bridge:main",
        ],
    },
)
