from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'rb3_handeye_calib'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='Hand-eye calibration for RB3 + RealSense (eye-in-hand)',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'tcp_pose_publisher = rb3_handeye_calib.tcp_pose_publisher:main',
            'charuco_board_generator = rb3_handeye_calib.charuco_board_generator:main',
            'sample_collector = rb3_handeye_calib.sample_collector:main',
            'handeye_solver = rb3_handeye_calib.handeye_solver:main',
        ],
    },
)
