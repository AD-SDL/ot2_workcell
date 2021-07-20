from setuptools import setup
import os
from glob import glob

package_name = "ot2_controller"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="nerra",
    maintainer_email="alan.linghao.wang@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "protocol_manager = ot2_controller.protocol_manager:main",
            "ot2_controller = ot2_controller.ot2_controller:main",
        ],
    },
)
