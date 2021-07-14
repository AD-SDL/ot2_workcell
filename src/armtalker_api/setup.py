from setuptools import setup

package_name = 'armtalker_api'

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
            'transfer_api = armtalker_api.transfer_api:main_null',
            'publish_arm_state_api = armtalker_api.publish_arm_state_api:main_null',
            'get_transfer_api = armtalker_api.get_transfer_api:main_null',
        ],
    },
)
