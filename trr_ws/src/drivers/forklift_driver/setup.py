from setuptools import find_packages, setup
from glob import glob

package_name = "forklift_driver"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*_launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Barry Walker",
    maintainer_email="bwalker96@gatech.edu",
    description="A driver for controlling the forklift",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "forklift_node = forklift_driver.forklift_node:main"
        ],
    },
)
