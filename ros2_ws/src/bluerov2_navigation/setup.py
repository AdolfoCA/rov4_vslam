from glob import glob

from setuptools import setup

package_name = "bluerov2_navigation"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Andrea Albasini",
    maintainer_email="50945937+andreaalbasini@users.noreply.github.com",
    description="Pier calibration: AQUA-SLAM map in the georeferenced harbour frame",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "pier_calibration = bluerov2_navigation.pier_calibration_node:main",
        ],
    },
)
