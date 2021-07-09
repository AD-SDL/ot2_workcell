from setuptools import setup

package_name = 'rostalker2'

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
            'OT2 = rostalker2.ot2Class:main',
            'master = rostalker2.master:main',
            'retry_functions = rostalker2.retry_functions:main_null',
            'worker_thread = rostalker2.worker_thread:main_null',
            'arm_manager = rostalker2.armManager:main',
            'arm_transfer_handler = rostalker2.armTransferHandler:main',
            'register_functions = rostalker2.register_functions:main_null',
            'worker_info_api = rostalker2.worker_info_api:main_null',
            'transfer_api = rostalker2.transfer_api:main_null',
        ],
    },
)
