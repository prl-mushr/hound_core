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
    description="D455 + Isaac ROS Visual SLAM bring-up from a single source-of-truth config.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "dora_clipseg_encoder = hound_core.dora_clipseg_encoder:main",
            "dora_seg_refine = hound_core.dora_seg_refine:main",
            "bag_replay = hound_core.bag_replay:main",
            "wheel_odom_node = hound_core.wheel_odom_node:main",
            "hal_monitor_node = hound_core.hal_monitor_node:main",
            "realsense_cuvslam_stereo = hound_core.realsense_cuvslam_stereo:main",
        ],
    },
)
