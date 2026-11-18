import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'jetbot_cartographer'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name] if os.path.exists('resource/' + package_name) else []),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='CS532 Robotics Team',
    maintainer_email='cs532@uit.edu.vn',
    description='ROS 2 Google Cartographer 2D SLAM Integration for NVIDIA JetBot CS532',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'jetbot_ros_bridge_node = scripts.jetbot_ros_bridge_node:main',
            'map_bridge_to_web = scripts.map_bridge_to_web:main',
        ],
    },
)
