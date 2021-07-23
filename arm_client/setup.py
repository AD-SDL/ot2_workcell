from setuptools import setup

package_name = "arm_client"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Alan Wang",
    maintainer_email="alan.wang@anl.gov",
    description="TODO: Package description",
    license="MIT License",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "transfer_api = arm_client.transfer_api:main_null",
            "publish_arm_state = arm_client.publish_arm_state:main_null",
        ],
    },
)
