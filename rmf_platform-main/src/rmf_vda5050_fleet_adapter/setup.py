"""RECONSTRUCTED ROS 2 package metadata."""

from glob import glob
from setuptools import find_packages, setup


PACKAGE_NAME = 'vda5050_fleet_adapter'

setup(
    name=PACKAGE_NAME,
    version='1.17.0',
    packages=find_packages(exclude=('test', 'tests')),
    data_files=[
        ('share/ament_index/resource_index/packages', [
            'resource/' + PACKAGE_NAME,
        ]),
        ('share/' + PACKAGE_NAME, ['package.xml']),
        ('share/' + PACKAGE_NAME + '/config', glob('config/*.yaml')),
        ('share/' + PACKAGE_NAME + '/map', glob('map/*.yaml')),
    ],
    install_requires=['setuptools', 'PyYAML', 'networkx', 'paho-mqtt'],
    zip_safe=True,
    maintainer='Reconstructed source',
    maintainer_email='reconstructed@example.invalid',
    description='Reconstructed Open-RMF VDA5050 fleet adapter',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'fleet_adapter = '
            'vda5050_fleet_adapter.presentation.main:main',
        ],
    },
)
