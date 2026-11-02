from setuptools import find_packages, setup

setup(
    name="beamng_wildos_adapter",
    version="0.2.0",
    packages=find_packages(),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/beamng_wildos_adapter"],
        ),
        ("share/beamng_wildos_adapter", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    entry_points={
        "console_scripts": ["beamng_wildos_adapter = beamng_wildos_adapter.node:main"]
    },
)
