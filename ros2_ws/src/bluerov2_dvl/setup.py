from setuptools import setup

package_name = "bluerov2_dvl"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Adolfo Damiano Cafaro",
    maintainer_email="adaca@dtu.dk",
    description="Water Linked DVL-A50 driver for ROS 2",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "dvl_node = bluerov2_dvl.dvl_node:main",
        ],
    },
)
