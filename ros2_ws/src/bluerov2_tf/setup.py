from setuptools import setup

package_name = "bluerov2_tf"

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
    description="Static TF publisher for the BlueROV2 sensor extrinsics",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "static_tf_node = bluerov2_tf.static_tf_node:main",
        ],
    },
)
