from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'nexus_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='mecroka',
    maintainer_email='',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'move_with patterns = nexus_control.move_with_patterns:main',
            'move_with_coordinates = nexus_control.move_with_coordinates:main',
            'nexus_twist_node  = nexus_control.nexus_twist_node:main'
        ],
    },
)
