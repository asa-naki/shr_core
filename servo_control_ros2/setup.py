from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'servo_control_ros2'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/servo_config.yaml']),
        ('share/' + package_name + '/launch', ['launch/servo_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='A ROS2 package for controlling servo motors with position setting functionality.',
    license='License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'servo_node = servo_control_ros2.servo_node:main',
            'test_setpos_simple = servo_control_ros2.test_setpos_simple:main',
        ],
    },
)