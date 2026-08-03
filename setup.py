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
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="hound",
    maintainer_email="hound@todo.todo",
    description="HOUND robot interface + SSoT mission orchestrator.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "bag_replay = hound_core.bag_replay:main",
            "wheel_odom_node = hound_core.wheel_odom_node:main",
            "hal_monitor_node = hound_core.hal_monitor_node:main",
        ],
    },
)
