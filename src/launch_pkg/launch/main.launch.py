from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 실행 여부 옵션
    run_robot_node = LaunchConfiguration('run_robot_node')
    run_command_node = LaunchConfiguration('run_command_node')
    run_master_node = LaunchConfiguration('run_master_node')
    run_master_node_dis = LaunchConfiguration('run_master_node_dis')

    run_gripper_node = LaunchConfiguration('run_gripper_node')
    run_robot2_gripper_node = LaunchConfiguration('run_robot2_gripper_node')

    run_vision_node = LaunchConfiguration('run_vision_node')
    run_decision_assembly_camera = LaunchConfiguration('run_decision_assembly_camera')

    # command_node는 키보드 입력 때문에 xterm 권장
    use_xterm_for_command = LaunchConfiguration('use_xterm_for_command')

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
            'run_command_node',
            default_value='false',
            description='Run control_pkg command_node'
        ),

        DeclareLaunchArgument(
            'run_master_node',
            default_value='false',
            description='Run control_pkg master_node'
        ),

        DeclareLaunchArgument(
            'run_master_node_dis',
            default_value='false',
            description='Run control_pkg master_node_dis'
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

        # decision_assembly_with_camera.launch.py 안에서 카메라를 켠다면
        # vision_node와 카메라 충돌이 날 수 있으므로 기본값 false
        DeclareLaunchArgument(
            'run_vision_node',
            default_value='true',
            description='Run vision_pkg vision_node'
        ),

        DeclareLaunchArgument(
            'run_decision_assembly_camera',
            default_value='true',
            description='Run vision_assembly_pkg decision_assembly_with_camera.launch.py'
        ),

        DeclareLaunchArgument(
            'use_xterm_for_command',
            default_value='true',
            description='Run command_node in xterm'
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
        # Master Nodes
        # 기본 false.
        # command_node와 master_node를 동시에 켜면 중복 제어될 수 있음.
        # =========================

        Node(
            package='control_pkg',
            executable='master_node',
            name='master_node',
            output='screen',
            emulate_tty=True,
            condition=IfCondition(run_master_node),
        ),

        Node(
            package='control_pkg',
            executable='master_node_dis',
            name='master_node_dis',
            output='screen',
            emulate_tty=True,
            condition=IfCondition(run_master_node_dis),
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
            condition=IfCondition(run_gripper_node),
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
        # Vision Node
        # 필요할 때만 켜기.
        # decision_assembly 쪽 launch에서 카메라를 잡으면 충돌 가능.
        # =========================

        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='vision_pkg',
                    executable='vision_node',
                    name='vision_node',
                    output='screen',
                    emulate_tty=True,
                    condition=IfCondition(run_vision_node),
                ),
            ],
        ),

        # =========================
        # Decision Assembly + Camera Launch
        #
        # 기존 실행 명령:
        # ros2 launch vision_assembly_pkg decision_assembly_with_camera.launch.py
        # =========================

        TimerAction(
            period=4.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        PathJoinSubstitution([
                            FindPackageShare('vision_assembly_pkg'),
                            'launch',
                            'decision_assembly_with_camera.launch.py'
                        ])
                    ),
                    condition=IfCondition(run_decision_assembly_camera),
                ),
            ],
        ),

        # =========================
        # Birdseye Assembly Node
        # decision_assembly(4s) 이후 기동
        # =========================

        TimerAction(
            period=6.0,
            actions=[
                Node(
                    package='vision_assembly_pkg',
                    executable='birdseye_assembly',
                    name='birdseye_assembly',
                    output='screen',
                    emulate_tty=True,
                ),
            ],
        ),

        # =========================
        # KHJ Point Node
        # vision_node(2s) + birdseye_assembly(6s) 이후 기동
        # =========================

        TimerAction(
            period=8.0,
            actions=[
                Node(
                    package='control_pkg',
                    executable='khj_point_node',
                    name='khj_point_node',
                    output='screen',
                    emulate_tty=True,
                ),
            ],
        ),

        # =========================
        # Command Node
        # 키보드 입력 때문에 xterm 실행 권장
        # =========================

        TimerAction(
            period=6.0,
            actions=[
                Node(
                    package='control_pkg',
                    executable='command_node',
                    name='command_node',
                    output='screen',
                    emulate_tty=True,
                    condition=IfCondition(
                        PythonExpression([
                            "'",
                            run_command_node,
                            "' == 'true' and '",
                            use_xterm_for_command,
                            "' != 'true'",
                        ])
                    ),
                ),
            ],
        ),

        TimerAction(
            period=6.0,
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
                            "'",
                            run_command_node,
                            "' == 'true' and '",
                            use_xterm_for_command,
                            "' == 'true'",
                        ])
                    ),
                ),
            ],
        ),
    ])
