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
    maintainer='nerra',
    maintainer_email='alan.wang@anl.gov',
    description='This package contains the scheduler controller nodes.',
    license='MIT Licenses',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'scheduler_manager = scheduler_controller.schedulerManager:main',
            'scheduler_work_adder = scheduler_controller.schedulerWorkAdder:main',
        ],
    },
)
