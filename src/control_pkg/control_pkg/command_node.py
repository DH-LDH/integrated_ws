"""
===============================================================================
File        : command_node.py
Package     : control_pkg
Author      : Jeho Yoon, Chaerin Seong, Dahan Lee, Donghyuk Jeong, Deokhui Han, Donggil Lee
Created     : 2026-06-30
Environment : Ubuntu 22.04, ROS2 Humble, Python 3.10

Description
-----------
Gateway node that receives wb_task actions from the manager and dispatches
them to the appropriate handler based on work_type.
Replaces the legacy mock_wb_node as the real workbench node while keeping
the same manager-side interface.

Main Features
-------------
- /wb_task Action server (WbTask): receives assembly and disassembly requests
- PRODUCE → MasterNode (master_node) : assembly sequence
- RECYCLE → BatteryDualDisassembly (master_node_dis) : disassembly sequence
- Maintains the same interface as mock_wb_node for manager compatibility

Required Nodes
--------------
- master_node     : MasterNode class (assembly)
- master_node_dis : BatteryDualDisassembly class (disassembly)
- manager node    : publishes /wb_task actions

Notes
-----

Revision History
----------------

===============================================================================
"""

import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from std_srvs.srv import Trigger, SetBool
from sml_msgs.action import WbTask
from srvs_pkg.srv import GetTargetPose

# ──────────────────────────────────────────────────────────
# 통합 ws 패키지: control_pkg
#    master_node.py     -> class MasterNode
#    master_node_dis.py -> class BatteryDualDisassembly
# ──────────────────────────────────────────────────────────
from control_pkg.master_node import MasterNode
from control_pkg.master_node_dis import BatteryDualDisassembly


# product_id -> 조립(build) 함수 이름
PRODUCE_FUNC = {
    34:    'build_battery',
    13:    'build_magnet',
    81:    'build_e_stop',
    442:   'build_carrot',
    241:   'build_traffic_light',
    462:   'build_small_tree',
    711:   'build_hammer',
    4482:  'build_big_carrot',
    8518:  'build_burger',
    48132: 'build_ice_cream',
    46262: 'build_big_tree',
}

# product_id -> 완성체 드롭 관절 (S=소형, M=중형, L=대형)
PRODUCE_DROP_TYPE = {
    34:    'ASSEMBLY_DROP_S',   # 배터리
    13:    'ASSEMBLY_DROP_S',   # 자석
    81:    'ASSEMBLY_DROP_S',   # 비상정지
    442:   'ASSEMBLY_DROP_M',   # 당근
    241:   'ASSEMBLY_DROP_M',   # 신호등
    462:   'ASSEMBLY_DROP_M',   # 작은나무
    711:   'ASSEMBLY_DROP_M',   # 망치
    4482:  'ASSEMBLY_DROP_L',   # 큰당근
    8518:  'ASSEMBLY_DROP_M',   # 버거
    48132: 'ASSEMBLY_DROP_L',   # 아이스크림
    46262: 'ASSEMBLY_DROP_L',   # 큰나무
}

# product_id -> 분해(run_*_once) 함수 이름
RECYCLE_FUNC = {
    34:    'run_battery_once',
    13:    'run_magnet_once',
    81:    'run_estop_once',
    442:   'run_carrot_once',
    241:   'run_traffic_light_once',
    462:   'run_small_tree_once',
    711:   'run_hammer_once',
    4482:  'run_big_carrot_once',
    8518:  'run_burger_once',
    48132: 'run_ice_cream_once',
    46262: 'run_big_tree_once',
}


class WbCommandNode(Node):

    def __init__(self):
        super().__init__('wb_command_node')
        self.cbg = ReentrantCallbackGroup()
        self.command_lock = threading.RLock()

        # 조립/분해 마스터 인스턴스 생성
        # (둘 다 __init__에서 클라이언트만 만들고 HW에는 직접 연결하지 않음)
        self.assembler = MasterNode()
        self.disassembler = BatteryDualDisassembly()

        # 마스터의 동기 호출(spin_until_future_complete)을
        # 외부 executor와 충돌하지 않는 future 폴링 방식으로 교체
        self._patch_sync_call(self.assembler)
        self._patch_sync_call(self.disassembler)

        self._action_server = ActionServer(
            self,
            WbTask,
            'wb_task',
            execute_callback=self._execute_cb,
            callback_group=self.cbg,
        )
        self.get_logger().info('[CMD] wb_task 분배 노드 시작')

        self.keyboard_thread = threading.Thread(
            target=self.keyboard_loop,
            daemon=True,
        )
        self.keyboard_thread.start()

    # ──────────────────────────────────────────────────────
    @staticmethod
    def _patch_sync_call(node):
        """
        master의 call()을 교체한다.
        원본: rclpy.spin_until_future_complete(self, future)  ← 외부 executor와 충돌
        교체: call_async 후 future.done()을 폴링            ← executor가 응답 처리
        (이 노드를 MultiThreadedExecutor로 spin할 때 동작)
        """
        def sync_call(cli, req):
            while not cli.wait_for_service(timeout_sec=1.0):
                node.get_logger().info(f'Waiting for {cli.srv_name}...')
            node.get_logger().info(f'[sync_call] → {cli.srv_name}')
            future = cli.call_async(req)
            elapsed = 0.0
            while rclpy.ok() and not future.done():
                time.sleep(0.005)
                elapsed += 0.005
                if elapsed % 5.0 < 0.005:
                    node.get_logger().warn(
                        f'[sync_call] {cli.srv_name} 응답 대기 중 ({elapsed:.0f}s)...')
            node.get_logger().info(f'[sync_call] ✓ {cli.srv_name} ({elapsed:.2f}s)')
            return future.result()
        node.call = sync_call

    # ──────────────────────────────────────────────────────
    def _execute_cb(self, goal_handle):
        work_type  = goal_handle.request.work_type
        product_id = goal_handle.request.product_id
        self.get_logger().info(
            f'[CMD] goal 수신: work_type={work_type}, product_id={product_id}')

        # 피드백: PROCESSING
        fb = WbTask.Feedback()
        fb.status = 'PROCESSING'
        goal_handle.publish_feedback(fb)

        try:
            with self.command_lock:
                if work_type == 'PRODUCE':
                    ok, reason = self._run_assembly(product_id, goal_handle)
                elif work_type == 'RECYCLE':
                    ok, reason = self._run_disassembly(product_id, goal_handle)
                else:
                    ok, reason = False, f'UNKNOWN_WORK_TYPE:{work_type}'
        except Exception as e:
            self.get_logger().error(f'[CMD] 실행 예외: {e}')
            ok, reason = False, f'EXCEPTION:{e}'

        result = WbTask.Result()
        if ok:
            goal_handle.succeed()
            result.success = True
            result.fail_reason = ''
            self.get_logger().info(
                f'[CMD] 완료: {work_type} product_id={product_id}')
        else:
            goal_handle.abort()
            result.success = False
            result.fail_reason = reason
            self.get_logger().error(
                f'[CMD] 실패: {work_type} product_id={product_id} ({reason})')
        return result

    # ──────────────────────────────────────────────────────
    # PRODUCE -> 조립
    # ──────────────────────────────────────────────────────
    def _run_assembly(self, product_id, goal_handle):
        func_name = PRODUCE_FUNC.get(product_id)
        if func_name is None:
            return False, f'NO_ASSEMBLY_FUNC:{product_id}'

        fb = WbTask.Feedback()
        fb.status = 'PRODUCING'
        goal_handle.publish_feedback(fb)

        a = self.assembler
        # 시작 상태 초기화: robot1 HOME + robot2 assembly_joint + 그리퍼1,2 open 동시
        a.assembly_completed = False
        a.post_action_home_done = False
        a.precision_scan_failed = False
        a.current_floor_count_at_home = None
        a.current_insert_start_count = None
        a.current_insert_verify_after_time = None

        for cli in (a.cli_h, a.cli_r2, a.cli_g, a.cli_g2):
            while not cli.wait_for_service(timeout_sec=1.0):
                a.get_logger().info(f"Waiting for {cli.srv_name}...")

        future_h  = a.cli_h.call_async(Trigger.Request())
        future_r2 = a.cli_r2.call_async(GetTargetPose.Request(target_size="ASSEMBLY_JOINT"))
        future_g  = a.cli_g.call_async(SetBool.Request(data=False))
        future_g2 = a.cli_g2.call_async(SetBool.Request(data=False))

        while rclpy.ok() and not (
            future_h.done() and future_r2.done()
            and future_g.done() and future_g2.done()
        ):
            time.sleep(0.005)

        home_res = future_h.result()
        if home_res is None or not home_res.success:
            return False, 'ASSEMBLY_HOME_FAILED'

        assembly_res = future_r2.result()
        if assembly_res is None or not assembly_res.success:
            return False, 'ROBOT2_ASSEMBLY_JOINT_FAILED'

        time.sleep(0.2)

        # 조립 시작 전: 로봇1&2 HOME 정렬 완료 후 전체 블록 스캔 + birdseye 동결
        a.scan_all_blocks_at_home()

        # 제품별 조립 시퀀스 실행
        getattr(a, func_name)()

        # 완성체 내려놓기
        if a.assembly_completed:
            drop_type = PRODUCE_DROP_TYPE.get(product_id)
            if drop_type:
                a.drop_assembly(drop_type)

        # 마무리: HOME 복귀 + 그리퍼 열기 + robot1/2 동시 END
        a.call(a.cli_h, Trigger.Request())
        time.sleep(0.3)
        a.call(a.cli_g, SetBool.Request(data=False))

        future_e1 = a.cli_r.call_async(GetTargetPose.Request(target_size="END"))
        future_e2 = a.cli_r2.call_async(GetTargetPose.Request(target_size="END"))
        while rclpy.ok() and not (future_e1.done() and future_e2.done()):
            time.sleep(0.005)

        end_res2 = future_e2.result()
        if end_res2 is None or not end_res2.success:
            return False, 'ROBOT2_END_FAILED'

        if a.precision_scan_failed:
            return False, 'PRECISION_SCAN_FAILED'
        if not a.assembly_completed:
            return False, 'ASSEMBLY_INCOMPLETE'
        return True, ''

    # ──────────────────────────────────────────────────────
    # RECYCLE -> 분해
    # ──────────────────────────────────────────────────────
    def _run_disassembly(self, product_id, goal_handle):
        func_name = RECYCLE_FUNC.get(product_id)
        if func_name is None:
            return False, f'NO_DISASSEMBLY_FUNC:{product_id}'

        fb = WbTask.Feedback()
        fb.status = 'RECYCLING'
        goal_handle.publish_feedback(fb)

        d = self.disassembler
        # 시작 상태 초기화 (master_node_dis.run() 시작부와 동일): 양팔 HOME
        d.move_both_home_pose()

        # 제품별 분해 시퀀스 실행
        ret = getattr(d, func_name)()

        # run_*_once 계열은 실패 시 False를 반환한다 (None/True는 성공)
        if ret is False:
            return False, 'DISASSEMBLY_FAILED'

        if not d.move_both_end_pose():
            return False, 'DISASSEMBLY_END_FAILED'

        return True, ''

    # ──────────────────────────────────────────────────────
    # Keyboard manual pose commands
    # ──────────────────────────────────────────────────────
    def keyboard_loop(self):
        print("\n[WB Command Keyboard]")
        print("  home / h             : robot1 HOME")
        print("  end / e              : robot1 END")
        print("  home2 / h2           : robot2 HOME")
        print("  end2 / e2            : robot2 END")
        print("  assembly_joint / aj  : robot2 ASSEMBLY_JOINT")
        print("  separation / sep     : robot1 SEPARATION (분리 조인트)")
        print("  separation2 / sep2   : robot2 SEPARATION (분리 조인트)")
        print("  home_all / ha        : both HOME")
        print("  end_all / ea         : both END")
        print("  quit / q             : shutdown command node")

        while rclpy.ok():
            try:
                command = input("wb-command> ").strip()
            except EOFError:
                return

            if not command:
                continue

            cmd_lower = command.lower()

            try:
                with self.command_lock:
                    if cmd_lower in ("home", "h"):
                        self.get_logger().info("[KEYBOARD] robot1 HOME")
                        self.assembler.call(
                            self.assembler.cli_h,
                            Trigger.Request(),
                        )
                    elif cmd_lower in ("end", "e"):
                        self.get_logger().info("[KEYBOARD] robot1 END")
                        self.assembler.move_robot_end()
                    elif cmd_lower in ("home2", "h2"):
                        self.get_logger().info("[KEYBOARD] robot2 HOME")
                        self.disassembler.call(
                            self.disassembler.cli_h2,
                            Trigger.Request(),
                        )
                    elif cmd_lower in ("end2", "e2"):
                        self.get_logger().info("[KEYBOARD] robot2 END")
                        self.disassembler.send_pose(
                            self.disassembler.cli_r2,
                            "END",
                        )
                    elif cmd_lower in ("assembly_joint", "assembly", "aj"):
                        self.get_logger().info("[KEYBOARD] robot2 ASSEMBLY_JOINT")
                        self.assembler.move_robot2_assembly_joint()
                    elif cmd_lower in ("separation", "sep", "s1"):
                        self.get_logger().info("[KEYBOARD] robot1 SEPARATION")
                        self.assembler.call(
                            self.assembler.cli_r,
                            GetTargetPose.Request(target_size="SEPARATION"),
                        )
                    elif cmd_lower in ("separation2", "sep2", "s2"):
                        self.get_logger().info("[KEYBOARD] robot2 SEPARATION")
                        self.assembler.call(
                            self.assembler.cli_r2,
                            GetTargetPose.Request(target_size="SEPARATION"),
                        )
                    elif cmd_lower in ("home_all", "homeall", "ha"):
                        self.get_logger().info("[KEYBOARD] both HOME")
                        self.disassembler.move_both_home_pose()
                    elif cmd_lower in ("end_all", "endall", "ea"):
                        self.get_logger().info("[KEYBOARD] both END")
                        self.disassembler.move_both_end_pose()
                    elif cmd_lower in ("quit", "exit", "q", "종료"):
                        self.get_logger().info("[KEYBOARD] shutdown requested")
                        rclpy.shutdown()
                        return
                    else:
                        print("Use: home/end/home2/end2/aj/sep/sep2/ha/ea  또는  quit")
            except Exception as e:
                self.get_logger().error(f"[KEYBOARD] command failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    cmd = WbCommandNode()

    # 세 노드(분배 + 조립 + 분해)를 하나의 MultiThreadedExecutor로 spin.
    # 블로킹 시퀀스가 executor를 막지 않도록 충분한 스레드 확보.
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(cmd)
    executor.add_node(cmd.assembler)
    executor.add_node(cmd.disassembler)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        cmd.assembler.destroy_node()
        cmd.disassembler.destroy_node()
        cmd.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
