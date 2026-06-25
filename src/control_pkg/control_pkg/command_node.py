"""
command_node.py
manager의 wb_task(Action)를 받아 work_type으로 조립/분해를 분배하는 노드.

  PRODUCE  -> MasterNode            (master_node)      조립
  RECYCLE  -> BatteryDualDisassembly(master_node_dis)  분해

manager 쪽 인터페이스(wb_task)는 mock_wb_node와 동일하게 유지한다.
즉, 이 노드는 기존 mock_wb_node를 대체하는 "진짜 워크벤치 노드"다.
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
            future = cli.call_async(req)
            while rclpy.ok() and not future.done():
                time.sleep(0.005)
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
        # 시작 상태 초기화 (master_node.run() 시작부와 동일): robot1 HOME + robot2 assembly_joint + 그리퍼 열기
        home_res = a.call(a.cli_h, Trigger.Request())
        if home_res is None or not home_res.success:
            return False, 'ASSEMBLY_HOME_FAILED'

        assembly_res = a.move_robot2_assembly_joint()
        if assembly_res is None or not assembly_res.success:
            return False, 'ROBOT2_ASSEMBLY_JOINT_FAILED'

        a.call(a.cli_g, SetBool.Request(data=False))
        time.sleep(1.0)

        # 제품별 조립 시퀀스 실행
        getattr(a, func_name)()

        # 마무리: HOME 복귀
        a.call(a.cli_h, Trigger.Request())
        time.sleep(1.0)
        a.call(a.cli_g, SetBool.Request(data=False))
        time.sleep(a.WAIT_TIME)
        a.move_robot_end()
        robot2_end_res = a.move_robot2_end()
        if robot2_end_res is None or not robot2_end_res.success:
            return False, 'ROBOT2_END_FAILED'

        # build_* 계열은 반환값이 없어(성공/실패 미구분) 예외만 없으면 성공으로 본다.
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
        print("  home / h        : robot1 HOME")
        print("  end / e         : robot1 END")
        print("  home2 / h2      : robot2 HOME")
        print("  end2 / e2       : robot2 END")
        print("  assembly_joint / aj : robot2 ASSEMBLY_JOINT")
        print("  home_all / ha   : both HOME")
        print("  end_all / ea    : both END")
        print("  quit / q        : shutdown command node")

        while rclpy.ok():
            try:
                command = input("wb-command> ").strip().lower()
            except EOFError:
                return

            if not command:
                continue

            try:
                with self.command_lock:
                    if command in ("home", "h"):
                        self.get_logger().info("[KEYBOARD] robot1 HOME")
                        self.assembler.call(
                            self.assembler.cli_h,
                            Trigger.Request(),
                        )
                    elif command in ("end", "e"):
                        self.get_logger().info("[KEYBOARD] robot1 END")
                        self.assembler.move_robot_end()
                    elif command in ("home2", "h2"):
                        self.get_logger().info("[KEYBOARD] robot2 HOME")
                        self.disassembler.call(
                            self.disassembler.cli_h2,
                            Trigger.Request(),
                        )
                    elif command in ("end2", "e2"):
                        self.get_logger().info("[KEYBOARD] robot2 END")
                        self.disassembler.send_pose(
                            self.disassembler.cli_r2,
                            "END",
                        )
                    elif command in ("assembly_joint", "assembly", "aj"):
                        self.get_logger().info("[KEYBOARD] robot2 ASSEMBLY_JOINT")
                        self.assembler.move_robot2_assembly_joint()
                    elif command in ("home_all", "homeall", "ha"):
                        self.get_logger().info("[KEYBOARD] both HOME")
                        self.disassembler.move_both_home_pose()
                    elif command in ("end_all", "endall", "ea"):
                        self.get_logger().info("[KEYBOARD] both END")
                        self.disassembler.move_both_end_pose()
                    elif command in ("quit", "exit", "q", "종료"):
                        self.get_logger().info("[KEYBOARD] shutdown requested")
                        rclpy.shutdown()
                        return
                    else:
                        print("Use: home/end/home2/end2/assembly_joint/home_all/end_all/quit")
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
