from setuptools import setup

package_name = 'scheduler_controller'

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
    maintainer='SDL',
    maintainer_email='alan.wang@anl.gov',
    description='TODO: Package description',
    license='MIT licenses',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "scheduler_manager = scheduler.schedulerManager:main"
        ],
    },
)
