from setuptools import setup
import os
from glob import glob

package_name = 'webots_lab2_pkg'

data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name, ['package.xml']))
data_files.append((os.path.join('share', package_name, 'launch'), glob('launch/*.py')))
data_files.append((os.path.join('share', package_name, 'worlds'), glob('worlds/*.wbt')))
data_files.append((os.path.join('share', package_name, 'resource'), glob('resource/*.urdf')))

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ilya',
    maintainer_email='iammasterspubg@gmail.com',
    description='Lab 2 Package with Webots and ROS2 metrics collection',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'metrics_collector = webots_lab2_pkg.metrics_collector:main',
        ],
    },
)