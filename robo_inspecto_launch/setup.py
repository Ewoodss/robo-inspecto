import os
from glob import glob
from setuptools import setup

package_name = 'robo_inspecto_launch'

setup(
    name=package_name,
    version='0.0.0',
    data_files=[
        # ... Other data files
        # Include all launch files.
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ewout',
    maintainer_email='we.baars@student.avans.nl',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
        ],
    },
)
