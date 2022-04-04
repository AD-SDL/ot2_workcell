from setuptools import setup

package_name = 'database'

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
    maintainer='Doga',
    maintainer_email='dozgulbas@anl.gov',
    description='Database files for the database',
    license='MIT Licenses',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'database_functions = database.database_functions:main_null',
            'connect = database.connect:main_null',
            'protocol_parser = database.protocol_parser:main_null'
        ],
    },
)
