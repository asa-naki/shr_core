from setuptools import find_packages, setup

package_name = 'esc_motor_control_python'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/esc_motor_control_python.launch.py']),
        ('share/' + package_name + '/config', ['config/esc_motor_control_python.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='asa-naki',
    maintainer_email='aki.grade2580@outlook.jp',
    description='Python ROS2 package for controlling brushed motors via ESC using PWM on Raspberry Pi',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'esc_motor_control_node = esc_motor_control_python.esc_motor_control_node:main',
            'esc_motor_test_node = esc_motor_control_python.esc_motor_test_node:main',
        ],
    },
)
