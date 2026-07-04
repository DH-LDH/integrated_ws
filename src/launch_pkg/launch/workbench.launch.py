"""
Unified workbench launch for assembly and disassembly.

This launch file is intended to replace running main.launch.py and
dis.launch.py at the same time.  Shared robot/control nodes are started once,
and command_node switches the robot1 RealSense vision process automatically
when /wb_task receives PRODUCE or RECYCLE.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _mode_is(mode_config, expected):
    return IfCondition(
        PythonExpression(["'", mode_config, "' == '", expected, "'"])
    )


def generate_launch_description():
    run_robot_node = LaunchConfiguration("run_robot_node")
    run_gripper_node = LaunchConfiguration("run_gripper_node")
    run_robot2_gripper_node = LaunchConfiguration("run_robot2_gripper_node")
    run_command_node = LaunchConfiguration("run_command_node")
    run_decision_assembly_camera = LaunchConfiguration("run_decision_assembly_camera")
    run_birdseye_assembly = LaunchConfiguration("run_birdseye_assembly")
    run_khj_point_node = LaunchConfiguration("run_khj_point_node")
    robot1_vision_mode = LaunchConfiguration("robot1_vision_mode")
    auto_manage_vision = LaunchConfiguration("auto_manage_vision")
    auto_manage_assembly_vision = LaunchConfiguration("auto_manage_assembly_vision")
    vision_start_timeout_sec = LaunchConfiguration("vision_start_timeout_sec")
    vision_stop_timeout_sec = LaunchConfiguration("vision_stop_timeout_sec")
    assembly_vision_start_timeout_sec = LaunchConfiguration(
        "assembly_vision_start_timeout_sec")
    use_xterm_for_command = LaunchConfiguration("use_xterm_for_command")
    use_xterm_for_gripper = LaunchConfiguration("use_xterm_for_gripper")
    vision_dis_camera_serial = LaunchConfiguration("vision_dis_camera_serial")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot1_vision_mode",
                default_value="auto",
                description=(
                    "robot1 camera startup mode: auto, assembly, disassembly, or none. "
                    "auto lets command_node switch vision by PRODUCE/RECYCLE."
                ),
            ),
            DeclareLaunchArgument(
                "auto_manage_vision",
                default_value="true",
                description="Let command_node start/stop robot1 vision by wb_task command",
            ),
            DeclareLaunchArgument(
                "auto_manage_assembly_vision",
                default_value="true",
                description="Let command_node start birdseye/decision only for PRODUCE",
            ),
            DeclareLaunchArgument(
                "vision_dis_camera_serial",
                default_value="332322072441",
                description="RealSense serial used by vision_node_dis",
            ),
            DeclareLaunchArgument(
                "vision_start_timeout_sec",
                default_value="45.0",
                description="Timeout waiting for robot1 vision service after auto start",
            ),
            DeclareLaunchArgument(
                "vision_stop_timeout_sec",
                default_value="8.0",
                description="Timeout waiting for robot1 vision process shutdown",
            ),
            DeclareLaunchArgument(
                "assembly_vision_start_timeout_sec",
                default_value="20.0",
                description="Timeout waiting for birdseye/decision stack after PRODUCE",
            ),
            DeclareLaunchArgument(
                "enable_command_keyboard",
                default_value="false",
                description="Enable command_node's built-in keyboard loop",
            ),
            DeclareLaunchArgument(
                "run_robot_node",
                default_value="true",
                description="Run control_pkg robot_node",
            ),
            DeclareLaunchArgument(
                "run_gripper_node",
                default_value="true",
                description="Run hardware_pkg gripper_node",
            ),
            DeclareLaunchArgument(
                "run_robot2_gripper_node",
                default_value="true",
                description="Run hardware_pkg robot2_gpio_gripper_node",
            ),
            DeclareLaunchArgument(
                "run_command_node",
                default_value="true",
                description="Run control_pkg command_node (/wb_task dispatcher)",
            ),
            DeclareLaunchArgument(
                "run_decision_assembly_camera",
                default_value="false",
                description="Run birdseye RealSense camera and decision_assembly",
            ),
            DeclareLaunchArgument(
                "run_birdseye_assembly",
                default_value="false",
                description="Run vision_assembly_pkg birdseye_assembly",
            ),
            DeclareLaunchArgument(
                "run_khj_point_node",
                default_value="false",
                description="Run control_pkg khj_point_node",
            ),
            DeclareLaunchArgument(
                "use_xterm_for_command",
                default_value="false",
                description="Run command_node in xterm",
            ),
            DeclareLaunchArgument(
                "use_xterm_for_gripper",
                default_value="false",
                description="Run gripper nodes in xterm",
            ),

            Node(
                package="control_pkg",
                executable="robot_node",
                name="robot_node",
                output="screen",
                emulate_tty=True,
                condition=IfCondition(run_robot_node),
            ),

            Node(
                package="hardware_pkg",
                executable="gripper_node",
                name="gripper_node",
                output="screen",
                emulate_tty=True,
                condition=IfCondition(
                    PythonExpression([
                        "'", run_gripper_node, "' == 'true' and '",
                        use_xterm_for_gripper, "' != 'true'",
                    ])
                ),
            ),
            Node(
                package="hardware_pkg",
                executable="gripper_node",
                name="gripper_node",
                output="screen",
                emulate_tty=True,
                prefix="xterm -hold -e",
                condition=IfCondition(
                    PythonExpression([
                        "'", run_gripper_node, "' == 'true' and '",
                        use_xterm_for_gripper, "' == 'true'",
                    ])
                ),
            ),
            Node(
                package="hardware_pkg",
                executable="robot2_gpio_gripper_node",
                name="robot2_gpio_gripper_node",
                output="screen",
                emulate_tty=True,
                condition=IfCondition(
                    PythonExpression([
                        "'", run_robot2_gripper_node, "' == 'true' and '",
                        use_xterm_for_gripper, "' != 'true'",
                    ])
                ),
            ),
            Node(
                package="hardware_pkg",
                executable="robot2_gpio_gripper_node",
                name="robot2_gpio_gripper_node",
                output="screen",
                emulate_tty=True,
                prefix="xterm -hold -e",
                condition=IfCondition(
                    PythonExpression([
                        "'", run_robot2_gripper_node, "' == 'true' and '",
                        use_xterm_for_gripper, "' == 'true'",
                    ])
                ),
            ),

            TimerAction(
                period=2.0,
                actions=[
                    Node(
                        package="vision_pkg",
                        executable="vision_node",
                        name="vision_node",
                        output="screen",
                        emulate_tty=True,
                        condition=_mode_is(robot1_vision_mode, "assembly"),
                    ),
                    Node(
                        package="vision_pkg",
                        executable="vision_node_dis",
                        name="vision_node_dis",
                        output="screen",
                        emulate_tty=True,
                        parameters=[{"camera_serial": vision_dis_camera_serial}],
                        condition=_mode_is(robot1_vision_mode, "disassembly"),
                    ),
                ],
            ),

            TimerAction(
                period=4.0,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            PathJoinSubstitution(
                                [
                                    FindPackageShare("vision_assembly_pkg"),
                                    "launch",
                                    "decision_assembly_with_camera.launch.py",
                                ]
                            )
                        ),
                        condition=IfCondition(run_decision_assembly_camera),
                    ),
                ],
            ),
            TimerAction(
                period=1.0,
                actions=[
                    Node(
                        package="control_pkg",
                        executable="command_node",
                        output="screen",
                        emulate_tty=True,
                        parameters=[
                            {
                                "auto_manage_vision": auto_manage_vision,
                                "auto_manage_assembly_vision": auto_manage_assembly_vision,
                                "vision_dis_camera_serial": ParameterValue(
                                    vision_dis_camera_serial,
                                    value_type=str,
                                ),
                                "vision_start_timeout_sec": vision_start_timeout_sec,
                                "vision_stop_timeout_sec": vision_stop_timeout_sec,
                                "assembly_vision_start_timeout_sec": (
                                    assembly_vision_start_timeout_sec
                                ),
                                "enable_keyboard": LaunchConfiguration(
                                    "enable_command_keyboard"
                                ),
                            }
                        ],
                        condition=IfCondition(run_command_node),
                    ),
                ],
            ),
            TimerAction(
                period=6.0,
                actions=[
                    Node(
                        package="vision_assembly_pkg",
                        executable="birdseye_assembly",
                        name="birdseye_assembly",
                        output="screen",
                        emulate_tty=True,
                        condition=IfCondition(run_birdseye_assembly),
                    ),
                ],
            ),
            TimerAction(
                period=8.0,
                actions=[
                    Node(
                        package="control_pkg",
                        executable="khj_point_node",
                        name="khj_point_node",
                        output="screen",
                        emulate_tty=True,
                        condition=IfCondition(run_khj_point_node),
                    ),
                ],
            ),
        ]
    )
