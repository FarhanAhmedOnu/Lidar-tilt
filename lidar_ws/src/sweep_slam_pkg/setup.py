import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'sweep_slam_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # THIS LINE TELLS ROS 2 TO INSTALL YOUR LAUNCH FILES
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@todo.todo',
    description='3D Sweep SLAM using 2D LiDAR and IMU',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # THIS LINE MAKES YOUR NODE EXECUTABLE
            'imu_publisher_node = sweep_slam_pkg.imu_publisher_node:main',
            'sweep_generator_node = sweep_slam_pkg.sweep_generator_node:main',
        ],
    },
)