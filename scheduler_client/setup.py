from setuptools import setup

package_name = 'scheduler_client'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nerra',
    maintainer_email='alan.linghao.wang@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "publish_scheduler_state = scheduler_client.publish_scheduler_state:main_null",
            "add_blocks_scheduler = scheduler_client.add_blocks_scheduler:main_null",
            "json_scheduler_reader = scheduler_client.json_scheduler_reader:test",
        ],
    },
)
