from setuptools import setup
from glob import glob
import os

package_name = 'nexus_description'

def package_files(directory):
    paths = []
    for (path, _, filenames) in os.walk(directory):
        for filename in filenames:
            full_path = os.path.join(path, filename)
            install_path = os.path.join('share', package_name, path)
            paths.append((install_path, [full_path]))
    return paths

setup(
    name=package_name,
    version='0.0.0',
    packages=[],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ]
    + package_files('urdf')
    + package_files('meshes')
    + package_files('launch')
    + package_files('rviz'),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mecroka',
    maintainer_email='54359353+OsamahShamsan@users.noreply.github.com',
    description='Robot description and visualization assets for Nexus robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={},
)
