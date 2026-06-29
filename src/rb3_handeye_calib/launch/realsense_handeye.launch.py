"""
===============================================================================
File        : realsense_handeye.launch.py
Package     : rb3_handeye_calib
Author      : Jeho Yoon, Chaerin Seong, Dahan Lee, Donghyuk Jeong, Deokhui Han, Donggil Lee
Created     : 2026-06-30
Environment : Ubuntu 22.04, ROS2 Humble, Python 3.10

Description
-----------
All-in-one launch file for the RealSense D435 + hand-eye calibration
sample collection pipeline. Starts camera driver, TCP publisher, and
sample collector in a single launch command.

Main Features
-------------
- realsense2_camera  : Intel RealSense D435 camera driver
- tcp_pose_publisher : publishes robot TCP pose as ROS2 topic
- sample_collector   : provides /handeye/capture_sample and /handeye/reset_samples

Required Nodes
--------------
- realsense2_camera package: sudo apt install ros-humble-realsense2-camera
- rbpodo Cobot driver

Notes
-----
Usage: ros2 launch rb3_handeye_calib realsense_handeye.launch.py \
         robot_ip:=10.0.2.7 robot_name:=robot1 session_dir:=<path>
Capture: ros2 service call /handeye/capture_sample std_srvs/srv/Trigger
Reset:   ros2 service call /handeye/reset_samples std_srvs/srv/Trigger

Revision History
----------------

===============================================================================
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument('robot_ip',          default_value='10.0.2.7'),
        DeclareLaunchArgument('robot_name',         default_value='robot1'),
        DeclareLaunchArgument('publish_rate',       default_value='20.0'),
        DeclareLaunchArgument('euler_order',        default_value='xyz'),
        DeclareLaunchArgument('session_dir',        default_value=os.path.expanduser('~/handeye_samples')),
        DeclareLaunchArgument('squares_x',          default_value='7'),
        DeclareLaunchArgument('squares_y',          default_value='5'),
        DeclareLaunchArgument('square_length_mm',   default_value='30.0'),
        DeclareLaunchArgument('marker_length_mm',   default_value='22.0'),
        DeclareLaunchArgument('dictionary',         default_value='DICT_4X4_50'),
        DeclareLaunchArgument('min_corners',        default_value='8'),
        DeclareLaunchArgument('save_annotated',     default_value='true'),
        # RealSense 파라미터
        DeclareLaunchArgument('serial_no',          default_value='332322072441'),
        DeclareLaunchArgument('rgb_camera_profile', default_value='640x480x30'),
    ]

    # ── RealSense D435 드라이버 ─────────────────────────────────────────────
    realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('realsense2_camera'), '/launch/rs_launch.py'
        ]),
        launch_arguments={
            'serial_no'        : LaunchConfiguration('serial_no'),
            'enable_color'     : 'true',
            'enable_depth'     : 'false',
            'enable_infra1'    : 'false',
            'enable_infra2'    : 'false',
            'enable_gyro'      : 'false',
            'enable_accel'     : 'false',
            'rgb_camera.color_profile': LaunchConfiguration('rgb_camera_profile'),
            'align_depth.enable': 'false',
        }.items(),
    )

    # ── TCP pose publisher ──────────────────────────────────────────────────
    tcp_pub = Node(
        package='rb3_handeye_calib',
        executable='tcp_pose_publisher',
        name='tcp_pose_publisher',
        output='screen',
        parameters=[{
            'robot_ip'    : LaunchConfiguration('robot_ip'),
            'robot_name'  : LaunchConfiguration('robot_name'),
            'publish_rate': LaunchConfiguration('publish_rate'),
            'euler_order' : LaunchConfiguration('euler_order'),
        }],
    )

    # ── Sample collector ────────────────────────────────────────────────────
    collector = Node(
        package='rb3_handeye_calib',
        executable='sample_collector',
        name='sample_collector',
        output='screen',
        parameters=[{
            'robot_name'       : LaunchConfiguration('robot_name'),
            'session_dir'      : LaunchConfiguration('session_dir'),
            'squares_x'        : LaunchConfiguration('squares_x'),
            'squares_y'        : LaunchConfiguration('squares_y'),
            'square_length_mm' : LaunchConfiguration('square_length_mm'),
            'marker_length_mm' : LaunchConfiguration('marker_length_mm'),
            'dictionary'       : LaunchConfiguration('dictionary'),
            'min_corners'      : LaunchConfiguration('min_corners'),
            'euler_order'      : LaunchConfiguration('euler_order'),
            'save_annotated'   : LaunchConfiguration('save_annotated'),
        }],
    )

    return LaunchDescription(args + [realsense, tcp_pub, collector])
