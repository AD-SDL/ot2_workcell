from setuptools import setup

package_name = 'ot2talker_api'

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
            'load_run_api = ot2talker_api.load_run_api:main_null',
            'publish_ot2_state_api = ot2talker_api.publish_ot2_state_api:main_null',
        ],
    },
)