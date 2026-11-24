import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'vs_disparity_extender'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*_launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Viktor Smokvina',
    maintainer_email='smokvina.viktor@gmail.com',
    description='Disparity Extender obstacle avoidance ROS 2 package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'disparity_extender = vs_disparity_extender.disparity_extender_node:main',
        ],
    },
)
