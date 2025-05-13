from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'nexus_serial_conn'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mecroka',
    maintainer_email='54359353+OsamahShamsan@users.noreply.github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'nexus_serial_conn_node = nexus_serial_conn.nexus_serial_conn_node:main'
        ],
    },
)
