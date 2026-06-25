from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    camera_name = LaunchConfiguration("camera_name")
    camera_namespace = LaunchConfiguration("camera_namespace")
    serial_no = LaunchConfiguration("serial_no")
    image_topic = LaunchConfiguration("image_topic")
    depth_topic = LaunchConfiguration("depth_topic")
    method = LaunchConfiguration("method")
    depth_combine_mode = LaunchConfiguration("depth_combine_mode")

    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("realsense2_camera"),
                    "launch",
                    "rs_launch.py",
                ]
            )
        ),
        launch_arguments={
            "camera_name": camera_name,
            "camera_namespace": camera_namespace,
            "serial_no": serial_no,
            "enable_color": "true",
            "enable_depth": "true",
            "align_depth.enable": "true",
        }.items(),
    )

    decision_assembly = Node(
        package="vision_assembly_pkg",
        executable="decision_assembly",
        name="decision_assembly",
        output="screen",
        emulate_tty=True,
        arguments=[
            "--image-topic",
            image_topic,
            "--depth-topic",
            depth_topic,
            "--method",
            method,
            "--use-depth",
            "--depth-combine-mode",
            depth_combine_mode,
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("camera_name", default_value="camera"),
            DeclareLaunchArgument("camera_namespace", default_value="camera"),
            DeclareLaunchArgument("serial_no", default_value="_243522075311"),
            DeclareLaunchArgument(
                "image_topic",
                default_value="/camera/camera/color/image_raw",
            ),
            DeclareLaunchArgument(
                "depth_topic",
                default_value="/camera/camera/aligned_depth_to_color/image_raw",
            ),
            DeclareLaunchArgument("method", default_value="cv"),
            DeclareLaunchArgument("depth_combine_mode", default_value="filter"),
            realsense_launch,
            decision_assembly,
        ]
    )
