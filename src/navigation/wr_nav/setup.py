from setuptools import find_packages, setup

package_name = "wr_nav"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Wisconsin Robotics",
    maintainer_email="wisconsinrobotics@gmail.com",
    description="TODO: Package description",
    license="MIT",
    entry_points={
        "console_scripts": ["nav = wr_nav.navigation:main"],
    },
)
