from setuptools import setup

package_name = "ot2_client"

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
            "load_run_api = ot2_client.load_run_api:main_null",
            "publish_ot2_state_api = ot2_client.publish_ot2_state_api:main_null",
        ],
    },
)
