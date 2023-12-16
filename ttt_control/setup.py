from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ttt_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ( (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', 'main_programme.xml'))))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='xinzhi',
    maintainer_email='lxzraizer@gmail.com',
    description='Main package to control the robot behavior, including the launch files to launch program',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_master = ttt_control.robot_master:main'
        ],
    },
)

