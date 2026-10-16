import os
from glob import glob

from setuptools import setup

package_name = "leap_teleop"


def tree(src_dir):
    """Install every file under src_dir to share/<pkg>/src_dir, keeping sub-folders."""
    out = []
    for root, _, files in os.walk(src_dir):
        if files:
            out.append((os.path.join("share", package_name, root),
                        [os.path.join(root, f) for f in files]))
    return out


setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/rviz", glob("rviz/*.rviz")),
        ("share/" + package_name + "/config", glob("config/*.json")),
    ] + tree("assets"),
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="bumsu",
    maintainer_email="bumsu@todo.todo",
    description="LEAP Hand teleoperation",
    license="MIT",
    entry_points={
        "console_scripts": [
            "retarget_check = leap_teleop.retarget_check:main",
            "rviz_teleop = leap_teleop.rviz_teleop_node:main",
        ],
    },
)
