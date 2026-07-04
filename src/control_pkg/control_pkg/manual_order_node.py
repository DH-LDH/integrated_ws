"""
===============================================================================
File        : manual_order_node.py
Package     : control_pkg

Description
-----------
manager_node가 아직 붙지 않았거나 꺼져 있을 때, 키보드로 직접 wb_task(Action)
goal을 보내는 수동 테스트 노드.
command_node가 열어둔 'wb_task' 액션 서버(robocup_pkg/action/WbTask)로 직접
goal을 보낸다. manager_node와 동일한 인터페이스를 그대로 재사용하므로
command_node 쪽 코드는 전혀 건드리지 않는다.

사용법
------
  1) command_node 를 먼저 실행 (launch 또는 `ros2 run control_pkg command_node`)
  2) 이 노드를 별도 터미널에서 실행: `ros2 run control_pkg manual_order_node`
     (launch 파일에는 포함하지 않음 — 항상 수동으로만 실행)
  3) order_type(1: 조립 / 2: 분해) 입력 후 product_id 입력

Notes
-----
- launch_pkg/main.launch.py 에는 등록하지 않는다.

===============================================================================
"""

import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor

from robocup_pkg.action import WbTask


# product_id -> 이름  (command_node.py의 PRODUCE_FUNC / RECYCLE_FUNC과 동일한 키)
PRODUCT_TABLE = [
    (34,    'Battery'),
    (13,    'Magnet'),
    (81,    'E-Stop'),
    (442,   'Carrot'),
    (241,   'Traffic Light'),
    (462,   'Small Tree'),
    (711,   'Hammer'),
    (4482,  'Big Carrot'),
    (8518,  'Burger'),
    (48132, 'Ice Cream'),
    (46262, 'Big Tree'),
]
PID_TO_NAME = {pid: name for (pid, name) in PRODUCT_TABLE}


class ManualOrderNode(Node):

    def __init__(self):
        super().__init__('manual_order_node')
        self.client = ActionClient(self, WbTask, 'wb_task')
        self.get_logger().info('[MANUAL] wb_task 클라이언트 시작')

    # ──────────────────────────────────────────────────────
    def feedback_cb(self, feedback_msg):
        self.get_logger().info(
            f'[MANUAL] 진행 상태: {feedback_msg.feedback.status}')

    def send_and_wait(self, work_type, product_id):
        """goal 전송 후 결과까지 동기적으로 대기 (executor는 백그라운드 spin)."""
        if not self.client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('[MANUAL] wb_task 서버 없음 (command_node 실행 확인)')
            return

        goal = WbTask.Goal()
        goal.work_type = work_type
        goal.product_id = product_id
        self.get_logger().info(
            f'[MANUAL] goal 전송: work_type={work_type}, product_id={product_id}')

        send_future = self.client.send_goal_async(
            goal, feedback_callback=self.feedback_cb)
        while rclpy.ok() and not send_future.done():
            time.sleep(0.01)
        goal_handle = send_future.result()

        if not goal_handle.accepted:
            self.get_logger().error('[MANUAL] goal 거절됨')
            return

        result_future = goal_handle.get_result_async()
        while rclpy.ok() and not result_future.done():
            time.sleep(0.01)
        result = result_future.result().result

        if result.success:
            self.get_logger().info('[MANUAL] 완료 (success=True)')
        else:
            self.get_logger().error(
                f'[MANUAL] 실패: {result.fail_reason}')


# ──────────────────────────────────────────────────────────
# 키보드 입력 루프 (메인 스레드)
# ──────────────────────────────────────────────────────────
def input_loop(node):
    while rclpy.ok():
        print('\n========================================')
        print('order_type 선택  (1: 조립 / 2: 분해 / q: 종료)')
        ot = input('order_type > ').strip().lower()

        if ot in ('q', 'quit', 'exit'):
            print('종료합니다.')
            rclpy.shutdown()
            break

        if ot == '1':
            work_type = 'PRODUCE'
            print('\n[조립] 만들 product_id를 입력하세요')
        elif ot == '2':
            work_type = 'RECYCLE'
            print('\n[분해] 분해할 product_id를 입력하세요')
        else:
            print('  → 1 또는 2를 입력하세요.')
            continue

        # product_id 목록 출력
        for (pid, name) in PRODUCT_TABLE:
            print(f'  {pid:>6} : {name}')

        sel = input('product_id > ').strip()
        if not sel.isdigit() or int(sel) not in PID_TO_NAME:
            print('  → 위 목록의 product_id를 입력하세요.')
            continue

        product_id = int(sel)
        print(f'  선택: {product_id} ({PID_TO_NAME[product_id]})')
        node.send_and_wait(work_type, product_id)


def main(args=None):
    rclpy.init(args=args)
    node = ManualOrderNode()

    # 백그라운드에서 spin (action 응답 처리용)
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        input_loop(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
