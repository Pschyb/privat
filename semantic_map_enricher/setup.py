import os
from glob import glob

from setuptools import find_packages, setup


package_name = "semantic_map_enricher"


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
    description="Assign YOLO object labels to ROSE2 rooms using the Go2 pose.",
    license="GPL-3.0-only",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "go2_object_mapper = "
            "semantic_map_enricher.go2_object_mapper:main",
        ],
    },
)
