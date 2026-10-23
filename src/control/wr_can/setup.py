import os
from glob import glob
from setuptools import find_packages, setup

package_name = "wr_can"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*")),
    ],
    install_requires=["setuptools", "python-can"],
    zip_safe=True,
    maintainer="Wisconsin Robotics",
    maintainer_email="wisconsinrobotics@gmail.com",
    description="Receives CAN message requests and sends them",
    license="MIT",
    tests_require=[],
    entry_points={
        "console_scripts": [
            "can = wr_can.can:main",
        ],
    },
)
