from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression

from launch_ros.actions import Node


def generate_launch_description():
    run_robot_node         = LaunchConfiguration('run_robot_node')
    run_gripper_node       = LaunchConfiguration('run_gripper_node')
    run_robot2_gripper_node = LaunchConfiguration('run_robot2_gripper_node')
    run_vision_node_dis    = LaunchConfiguration('run_vision_node_dis')
    run_master_node_dis    = LaunchConfiguration('run_master_node_dis')
    run_command_node       = LaunchConfiguration('run_command_node')
    use_xterm_for_command  = LaunchConfiguration('use_xterm_for_command')
    use_xterm_for_gripper  = LaunchConfiguration('use_xterm_for_gripper')

    return LaunchDescription([

        # =========================
        # Launch Arguments
        # =========================

        DeclareLaunchArgument(
            'run_robot_node',
            default_value='true',
            description='Run control_pkg robot_node'
        ),

        DeclareLaunchArgument(
            'run_gripper_node',
            default_value='true',
            description='Run hardware_pkg gripper_node'
        ),

        DeclareLaunchArgument(
            'run_robot2_gripper_node',
            default_value='true',
            description='Run hardware_pkg robot2_gpio_gripper_node'
        ),

        DeclareLaunchArgument(
            'run_vision_node_dis',
            default_value='true',
            description='Run vision_pkg vision_node_dis (pt 파일 기반, /get_target_pose_dis)'
        ),

        DeclareLaunchArgument(
            'run_master_node_dis',
            default_value='true',
            description='Run control_pkg master_node_dis'
        ),

        DeclareLaunchArgument(
            'run_command_node',
            default_value='true',
            description='Run control_pkg command_node'
        ),

        DeclareLaunchArgument(
            'use_xterm_for_command',
            default_value='true',
            description='Run command_node in xterm'
        ),

        DeclareLaunchArgument(
            'use_xterm_for_gripper',
            default_value='true',
            description='Run gripper_node in xterm'
        ),

        # =========================
        # Robot Node
        # =========================

        Node(
            package='control_pkg',
            executable='robot_node',
            name='robot_node',
            output='screen',
            emulate_tty=True,
            condition=IfCondition(run_robot_node),
        ),

        # =========================
        # Gripper Nodes
        # =========================

        Node(
            package='hardware_pkg',
            executable='gripper_node',
            name='gripper_node',
            output='screen',
            emulate_tty=True,
            condition=IfCondition(
                PythonExpression([
                    "'", run_gripper_node, "' == 'true' and '",
                    use_xterm_for_gripper, "' != 'true'",
                ])
            ),
        ),

        Node(
            package='hardware_pkg',
            executable='gripper_node',
            name='gripper_node',
            output='screen',
            emulate_tty=True,
            prefix='xterm -hold -e',
            condition=IfCondition(
                PythonExpression([
                    "'", run_gripper_node, "' == 'true' and '",
                    use_xterm_for_gripper, "' == 'true'",
                ])
            ),
        ),

        Node(
            package='hardware_pkg',
            executable='robot2_gpio_gripper_node',
            name='robot2_gpio_gripper_node',
            output='screen',
            emulate_tty=True,
            condition=IfCondition(run_robot2_gripper_node),
        ),

        # =========================
        # Vision Node (분해 전용, pt 파일 기반)
        # /get_target_pose_dis 서비스 제공
        # =========================

        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='vision_pkg',
                    executable='vision_node_dis',
                    name='vision_node_dis',
                    output='screen',
                    emulate_tty=True,
                    condition=IfCondition(run_vision_node_dis),
                ),
            ],
        ),

        # =========================
        # Master Node Dis (vision 2s 이후 기동)
        # =========================

        TimerAction(
            period=4.0,
            actions=[
                Node(
                    package='control_pkg',
                    executable='master_node_dis',
                    name='master_node_dis',
                    output='screen',
                    emulate_tty=True,
                    condition=IfCondition(run_master_node_dis),
                ),
            ],
        ),

        # =========================
        # Command Node (키보드 입력 xterm 권장)
        # =========================

        TimerAction(
            period=4.0,
            actions=[
                Node(
                    package='control_pkg',
                    executable='command_node',
                    name='command_node',
                    output='screen',
                    emulate_tty=True,
                    condition=IfCondition(
                        PythonExpression([
                            "'", run_command_node, "' == 'true' and '",
                            use_xterm_for_command, "' != 'true'",
                        ])
                    ),
                ),
            ],
        ),

        TimerAction(
            period=4.0,
            actions=[
                Node(
                    package='control_pkg',
                    executable='command_node',
                    name='command_node',
                    output='screen',
                    emulate_tty=True,
                    prefix='xterm -hold -e',
                    condition=IfCondition(
                        PythonExpression([
                            "'", run_command_node, "' == 'true' and '",
                            use_xterm_for_command, "' == 'true'",
                        ])
                    ),
                ),
            ],
        ),
    ])
