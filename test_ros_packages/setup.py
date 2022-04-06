from setuptools import setup

package_name = 'test_ros_packages'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, 'test_ros_packages_2'],
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
            'test_ros_1 = test_ros_packages.print_1_test:main',
            'test_ros_2 = test_ros_packages_2.print_2_test:main'
        ],
    },
)
