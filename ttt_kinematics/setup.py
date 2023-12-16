from setuptools import find_packages, setup

package_name = 'ttt_kinematics'

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
    maintainer='xinzhi',
    maintainer_email='lxzraizer@gmail.com',
    description='This package contains nodes to convert between different coordinates system, as well as converting joint angles to relevant data to send to motor',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joints_angle_to_data = ttt_kinematics.joints_angle_to_data:main',
            'inverse_kinematics = ttt_kinematics.inverse_kinematics:main',
            'motion_planner = ttt_kinematics.motion_planner:main'
        ],
    },
)
