import os
from glob import glob
from setuptools import find_packages, setup

package_name = "wr_controller"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Wisconsin Robotics",
    maintainer_email="wisconsinrobotics@gmail.com",
    description="Reads Xbox controller input for swerve drive and arm.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "drive_controller = wr_controller.drive_controller:main",
        ],
    },
)
