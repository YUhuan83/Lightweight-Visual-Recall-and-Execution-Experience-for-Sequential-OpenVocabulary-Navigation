import os
from glob import glob

from setuptools import find_packages, setup

package_name = "beamng_wildos_adapter"
setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    entry_points={"console_scripts": [
        "beamng_wildos_adapter = beamng_wildos_adapter.node:main",
        "seed_goal = beamng_wildos_adapter.seed_goal:main",
        "official_health_check = beamng_wildos_adapter.health_check:main",
        "goal_standoff = beamng_wildos_adapter.goal_standoff:main",
    ]},
)
