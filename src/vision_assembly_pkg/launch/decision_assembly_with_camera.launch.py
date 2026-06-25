from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("realsense2_camera"),
                "launch",
                "rs_launch.py",
            ])
        ),
        launch_arguments={
            "camera_name": "camera",
            "camera_namespace": "camera",
            "serial_no": "_243522075311",
            "enable_color": "true",
        }.items(),
    )

    decision_assembly = Node(
        package="vision_assembly_pkg",
        executable="decision_assembly",
        name="decision_assembly",
        output="screen",
        arguments=[
            "--image-topic", "/camera/camera/color/image_raw",
            "--method", "cv",
            "--roi-polygon", "204", "26", "433", "27", "640", "480", "0", "480",
            "--cv-min-area", "500",
            "--show",
        ],
    )

    return LaunchDescription([
        realsense_launch,
        decision_assembly,
    ])
