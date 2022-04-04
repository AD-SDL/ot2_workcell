from setuptools import setup

package_name = 'database'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, 'protocol_handler'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Doga',
    maintainer_email='dozgulbas@anl.gov',
    description='Database and protocol parser files for communication between ROS and database and ROS and internal pis',
    license='MIT Licenses',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'database_functions = database.database_functions:main_null',
            'connect = database.connect:main_null',
            'protocol_parser = protocol_handler.protocol_parser:main_null',
            'protocol_handling_client = protocol_handler.protocol_handling_client:main_null'
        ],
    },
)
