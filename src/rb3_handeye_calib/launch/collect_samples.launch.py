"""
===============================================================================
File        : collect_samples.launch.py
Package     : rb3_handeye_calib
Author      : Jeho Yoon, Chaerin Seong, Dahan Lee, Donghyuk Jeong, Deokhui Han, Donggil Lee
Created     : 2026-06-30
Environment : Ubuntu 22.04, ROS2 Humble, Python 3.10

Description
-----------
Launch file for the hand-eye calibration sample collection stage.
Starts camera driver, TCP pose publisher, and sample collector node.

Main Features
-------------
- realsense2_camera  : D435 camera driver (when use_realsense_launch:=true)
- tcp_pose_publisher : publishes robot TCP pose to ROS2 topic
- sample_collector   : waits for /handeye/capture_sample service calls

Required Nodes
--------------
- realsense2_camera package (optional, enabled via use_realsense_launch arg)
- rbpodo Cobot driver (robot TCP reading)

Notes
-----
Usage: ros2 launch rb3_handeye_calib collect_samples.launch.py \
         robot_ip:=10.0.2.7 robot_name:=robot1 use_realsense_launch:=true
Capture: ros2 service call /handeye/capture_sample std_srvs/srv/Trigger

Revision History
----------------

===============================================================================
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument('robot_ip',            default_value='10.0.2.7'),
        DeclareLaunchArgument('robot_name',           default_value='robot1'),
        DeclareLaunchArgument('publish_rate',         default_value='20.0'),
        DeclareLaunchArgument('euler_order',          default_value='xyz'),
        DeclareLaunchArgument('session_dir',          default_value=''),
        DeclareLaunchArgument('squares_x',            default_value='7'),
        DeclareLaunchArgument('squares_y',            default_value='5'),
        DeclareLaunchArgument('square_length_mm',     default_value='30.0'),
        DeclareLaunchArgument('marker_length_mm',     default_value='22.0'),
        DeclareLaunchArgument('dictionary',           default_value='DICT_4X4_50'),
        DeclareLaunchArgument('min_corners',          default_value='8'),
        DeclareLaunchArgument('save_annotated',       default_value='true'),
        DeclareLaunchArgument('use_realsense_launch', default_value='false'),
        DeclareLaunchArgument('serial_no',            default_value='332322072441'),
    ]

    def make_nodes(context):
        nodes = []

        if context.launch_configurations.get('use_realsense_launch', 'false').lower() == 'true':
            # Pass serial_no as a plain Python str so launch_ros writes it as a
            # YAML string — avoiding the integer-type error from rs_launch.py's
            # IncludeLaunchDescription path which loses the type information.
            serial_no = context.launch_configurations.get('serial_no', '332322072441').strip("'\"")
            realsense = Node(
                package='realsense2_camera',
                namespace='camera',
                name='camera',
                executable='realsense2_camera_node',
                output='screen',
                parameters=[{
                    'serial_no':              serial_no,   # str → YAML string, no integer coercion
                    'enable_color':           True,
                    'enable_depth':           False,
                    'enable_infra1':          False,
                    'enable_infra2':          False,
                    'enable_gyro':            False,
                    'enable_accel':           False,
                    'align_depth.enable':     False,
                    'rgb_camera.color_profile': '640,480,30',
                }],
            )
            nodes.append(realsense)

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

        nodes += [tcp_pub, collector]
        return nodes

    return LaunchDescription(args + [OpaqueFunction(function=make_nodes)])
