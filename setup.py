from setuptools import setup
import os
from glob import glob

package_name = 'navigation_server'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy]*')), # To add launch files
        # (os.path.join('share', package_name), glob('params/*.[yma]*')), # To add yaml files
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='C. Mauricio Arteaga-Escamilla',
    maintainer_email='cmauricioae8@gmail.com',
    description='This package is a bridge between ROS 2 ecosystem and a REST API',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
                'server_node = navigation_server.server_node:main'
        ],
    },
)
