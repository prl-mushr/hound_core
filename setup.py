import os
from glob import glob

from setuptools import find_packages, setup

package_name = "hound_core"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
            ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="hound",
    maintainer_email="hound@todo.todo",
    description="D455 + Isaac ROS Visual SLAM bring-up from a single source-of-truth config.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "clipseg_mask_node = hound_core.clipseg_mask_node:main",
            "bag_replay = hound_core.bag_replay:main",
        ],
    },
)
