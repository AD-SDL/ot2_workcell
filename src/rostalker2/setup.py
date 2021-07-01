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
        ],
    },
)
