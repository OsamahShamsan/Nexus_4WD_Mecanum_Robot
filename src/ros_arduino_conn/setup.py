from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ros_arduino_conn'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mecroka',
    maintainer_email='mecroka@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cmd_vel_to_serial = ros_arduino_conn.cmd_vel_to_serial:main',
            'move_robot_node = ros_arduino_conn.move_robot_node:main',
            'serial_conn_node = ros_arduino_conn.serial_conn_node:main',
            'twist_nexus_node = ros_arduino_conn.twist_nexus_node:main',
            'odom_nexus_node  = ros_arduino_conn.odom_nexus_node:main'
        ],
    },
)
