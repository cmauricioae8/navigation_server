from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'navigation_server'

# Create a list of tuples for data_files
data_files_entries = [
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'launch'), glob('launch/*launch.[pxy]*')), # To add launch files
    (os.path.join('share', package_name, 'params'), glob('params/*.[yma]*')),

    (os.path.join('share', package_name, package_name, 'webapp', 'templates'), glob(package_name+'/webapp/templates/*.html')),
    (os.path.join('share', package_name, package_name, 'webapp', 'templates','home'), glob(package_name+'/webapp/templates/home/*.html')),
    

    (os.path.join('share', package_name, package_name, 'webapp', 'static', 'css'), glob(package_name+'/webapp/static/css/*.css')),
    (os.path.join('share', package_name, package_name, 'webapp', 'static', 'img'), glob(package_name+'/webapp/static/img/*.png')),
    (os.path.join('share', package_name, package_name, 'webapp', 'static', 'js'), glob(package_name+'/webapp/static/js/*.js')),

    (os.path.join('share', package_name, package_name, 'webapp', 'static', 'plugins','bootstrap'), glob(package_name+'/webapp/static/plugins/bootstrap/*.*')),
    (os.path.join('share', package_name, package_name, 'webapp', 'static', 'plugins','highlight'), glob(package_name+'/webapp/static/plugins/highlight/*.*')),
    (os.path.join('share', package_name, package_name, 'webapp', 'static', 'plugins','jquery'), glob(package_name+'/webapp/static/plugins/jquery/*.*')),
    (os.path.join('share', package_name, package_name, 'webapp', 'static', 'plugins','socketio'), glob(package_name+'/webapp/static/plugins/socketio/*.*')),
]

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files_entries,
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
