import os
from glob import glob

from setuptools import find_packages, setup

package_name = "yaml_extractor"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Paul",
    maintainer_email="Pschyb@users.noreply.github.com",
    description="Export ROSE2 room segmentation results to YAML.",
    license="GPL-3.0-only",
    entry_points={
        "console_scripts": [
            "yaml_extractor = yaml_extractor.live_segmented_map_exporter:main",
        ],
    },
)
