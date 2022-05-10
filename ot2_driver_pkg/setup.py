from setuptools import setup

package_name = 'ot2_driver_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, 'protocol_handler', 'zeroMQ_OT2', 'vision_pipette', 'ZeroMQ_External', 'doga_tests'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Doga Ozgulbas and Alan Wang',
    maintainer_email='dozgulbas@anl.gov',
    description='Driver for the OT2',
    license='MIT License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ot2_driver = ot2_driver_pkg.ot2_driver:main_null',
            'database_functions = database.database_functions:main_null',
            'connect = database.connect:main_null',
            'protocol_parser = protocol_handler.protocol_parser:main_null',
            'protocol_handling_client = protocol_handler.protocol_handling_client:main_null',
            'protocol_transfer = protocol_handler.protocol_transfer:main_null',
            'execute_command = zeroMQ_OT2.execute_command:main_null', 
            'external_server = zeroMQ_OT2.external_server:main_null',
            'ot2_client = zeroMQ_OT2.ot2_client:main_null',
            'OT2_listener = zeroMQ_OT2.OT2_listener:main_null',
            'stream_camera = zeroMQ_OT2.stream_camera:main_null',
            'camera_server = vision_pipette.camera_server:main',
            'camera_client_test = doga_tests.camera_client_test:camera_client',
            'open_shell_ssh = doga_tests.open_shell_ssh:main',
        ],
    },
)
