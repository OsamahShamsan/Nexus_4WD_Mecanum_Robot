from setuptools import setup
from glob import glob
import os

package_name = 'nexus_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=[],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # URDFs
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf.xacro')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),

        # Meshes
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*.*')),

        # Launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),

        # RViz config
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mecroka',
    maintainer_email='54359353+OsamahShamsan@users.noreply.github.com',
    description='Robot description and visualization assets for Nexus robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Add any console scripts here if needed
        ],
    },
)
