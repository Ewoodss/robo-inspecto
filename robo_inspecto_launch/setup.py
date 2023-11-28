import os
from glob import glob
from setuptools import setup

package_name = 'robo_inspecto_launch'

def gen_share_data(input):
    # input example "launch/*.launch.py"

    # split at the last / to get the dir name
    dir_name = input.rsplit('/', 1)[0]
    # get all files in the dir
    files = glob(input)

    # return a list of tuples (dir_name, [files])
    return (os.path.join('share', package_name, dir_name), files)

setup(
    name=package_name,
    version='0.0.0',
    data_files=[
        # ... Other data files
        # Include all launch files.
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        gen_share_data('launch/*_launch.py'),
        gen_share_data('urdf/*.[urdf][xarco]*'),
        gen_share_data('rviz/*.rviz'),
        gen_share_data('meshes/*.STL'),
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
