from setuptools import find_packages, setup

package_name = 'ttt_perception'

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
    description='A ROS2 package to take in image data and localise the circular pieces in the playing scene of a tictactoe game',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'img_to_pos = ttt_perception.img_to_pos_node:main',
            'playing_field_pub = ttt_perception.play_field_node:main',
            'cv2_to_ros2 = ttt_perception.cv2_to_ros2:main' 
        ],
    },
)
