import json
import rclpy
from rclpy.node import Node

from srvs_pkg.srv import GetTargetPose
from std_srvs.srv import SetBool, Trigger
from std_msgs.msg import Int32, String

import time
import math


# --------------------------------------------------------------------- #
# 모듈 레벨 기본값
# --------------------------------------------------------------------- #

Z_OFF_DEFAULT        = -100.0   # 카메라 Z → 로봇 툴 Z 변환 오프셋 (mm)
Z_MARGIN_DEFAULT     =  22.0   # APPROACH 후 최종 하강 여유 거리 (mm)
BLOCK_H_DEFAULT      =  19.0   # 듀플로/레고 블록 한 층 높이 (mm)
WAIT_TIME_DEFAULT    =   0.7 # 모션 완료 후 대기 시간 (s)
PRE_XY_LOWER_DEFAULT = 80.0   # 정밀 재촬영을 위한 중간 Z 높이 (mm)
WRIST_OFFSET_DEFAULT =   0.0   # 손목 추가 회전 각도 (deg)
HOME_X_SEARCH_ENABLE_DEFAULT = True
HOME_X_SEARCH_STEP_M_DEFAULT = 0.150

# 정밀 재촬영 시, 대상 블록 중심에서 global y축 방향으로 이동할 거리.
# 현재 기본값 +0.100m = global y축 +100mm 방향
PRECISION_SCAN_GLOBAL_Y_OFFSET_DEFAULT = -0.100

# 처음 인식한 pose.x가 음수인 블록만 정밀 재촬영 위치를 추가 보정할지 여부
NEGATIVE_X_SCAN_EXTRA_ENABLE_DEFAULT = True

# pose.x < 0일 때 global x축 방향으로 추가 이동할 거리
# -0.050m = global x축 -50mm 방향
NEGATIVE_X_EXTRA_GLOBAL_X_OFFSET_DEFAULT = 0.050

# 호환용 legacy 파라미터. 현재 조립 검증은 재촬영 이동이 아니라
# decision_assembly block_count 감소로 판단한다.
VERIFY_SCAN_VIA_HOME_DEFAULT = True

STUD_PITCH = 0.016  # 스터드 간격 (m)
SAME_CLASS_VERIFY_XY_TOLERANCE_M = 0.040
POSE_EXCLUSION_XY_TOLERANCE_M = 0.045
DECISION_BLOCK_COUNT_TOPIC_DEFAULT = "/decision_assembly/block_count"

# KHJ 버드아이뷰 offset_cm → 로봇 팔 좌표 변환 계수
# [실측 결론] 로봇1=남쪽, 버드아이=서쪽 구조:
#   offset_cm.x  → robot_x (동일 부호)
#   offset_cm.y  → robot_y (반전:  robot_y = -offset_cm.y * scale)
#
# 이전 파라미터명(khj_y_to_robot_x, khj_x_to_robot_y)은 축이 잘못 명명되어
# 아래 두 파라미터로 대체한다.
KHJ_X_TO_ROBOT_X_DEFAULT =  0.01   # offset_cm.x → robot_x  (cm → m 변환, birdseye 실제 단위 기준)
KHJ_Y_TO_ROBOT_Y_DEFAULT =  0.01   # offset_cm.y → robot_y  (cm → m 변환, birdseye 실제 단위 기준)


# 기존 CLASS_TO_TARGET_ID 매핑 유지
CLASS_TO_TARGET_ID = {
    "2x2_red":    "1",
    "2x2_green":  "2",
    "2x2_blue":   "3",
    "2x2_yellow": "4",

    "4x2_red":    "5",
    "4x2_green":  "6",
    "4x2_blue":   "7",
    "4x2_yellow": "8",

    "2x4_red":    "5",
    "2x4_green":  "6",
    "2x4_blue":   "7",
    "2x4_yellow": "8",

    "assembly":   "999",
}


class MasterNode(Node):
    def __init__(self):
        super().__init__("master_node")

        # ------------------------------------------------------------- #
        # 기존 ROS 서비스 연결 구조 유지
        # ------------------------------------------------------------- #
        self.cli_v = self.create_client(GetTargetPose, "/get_target_pose")
        self.cli_r = self.create_client(GetTargetPose, "/robot1/robot_move_step")
        self.cli_r2 = self.create_client(GetTargetPose, "/robot2/robot_move_step")
        self.cli_g = self.create_client(SetBool, "/control_gripper")
        self.cli_h = self.create_client(Trigger, "/robot1/robot_home")
        self.cli_scan_all_blocks = self.create_client(Trigger, "/scan_all_blocks")
        self.cli_lock_positions = self.create_client(Trigger, "/lock_positions")

        # ------------------------------------------------------------- #
        # 기존 ROS 파라미터 구조 유지
        # ------------------------------------------------------------- #
        self.Z_OFF = float(
            self.declare_parameter("robot1_z_off", Z_OFF_DEFAULT).value
        )
        self.Z_MARGIN = float(
            self.declare_parameter("robot1_z_margin", Z_MARGIN_DEFAULT).value
        )
        self.BLOCK_H = float(
            self.declare_parameter("block_h", BLOCK_H_DEFAULT).value
        )
        self.WAIT_TIME = float(
            self.declare_parameter("wait_time", WAIT_TIME_DEFAULT).value
        )
        self.PRE_XY_LOWER = float(
            self.declare_parameter("pre_xy_lower_mm", PRE_XY_LOWER_DEFAULT).value
        )
        self.WRIST_OFFSET = float(
            self.declare_parameter("wrist_offset_deg", WRIST_OFFSET_DEFAULT).value
        )
        self.HOME_X_SEARCH_ENABLE = bool(
            self.declare_parameter(
                "home_x_search_enable",
                HOME_X_SEARCH_ENABLE_DEFAULT,
            ).value
        )
        self.HOME_X_SEARCH_STEP_M = float(
            self.declare_parameter(
                "home_x_search_step_m",
                HOME_X_SEARCH_STEP_M_DEFAULT,
            ).value
        )

        # ------------------------------------------------------------- #
        # 정밀 재촬영 global y offset
        # ------------------------------------------------------------- #
        self.PRECISION_SCAN_GLOBAL_Y_OFFSET_M = float(
            self.declare_parameter(
                "precision_scan_global_y_offset_m",
                PRECISION_SCAN_GLOBAL_Y_OFFSET_DEFAULT,
            ).value
        )

        # ------------------------------------------------------------- #
        # 추가 파라미터:
        # 처음 인식한 pose.x가 음수인 블록만 재촬영 위치를 global x축으로
        # 추가 이동시킨다.
        #
        # 기본 동작:
        #   pose.x < 0이면 scan_x = pose.x - 0.050
        #   pose.x >= 0이면 scan_x = pose.x
        # ------------------------------------------------------------- #
        self.NEGATIVE_X_SCAN_EXTRA_ENABLE = bool(
            self.declare_parameter(
                "negative_x_scan_extra_enable",
                NEGATIVE_X_SCAN_EXTRA_ENABLE_DEFAULT,
            ).value
        )

        self.NEGATIVE_X_EXTRA_GLOBAL_X_OFFSET_M = float(
            self.declare_parameter(
                "negative_x_extra_global_x_offset_m",
                NEGATIVE_X_EXTRA_GLOBAL_X_OFFSET_DEFAULT,
            ).value
        )

        self.VERIFY_SCAN_VIA_HOME = bool(
            self.declare_parameter(
                "verify_scan_via_home",
                VERIFY_SCAN_VIA_HOME_DEFAULT,
            ).value
        )
        self.DECISION_BLOCK_COUNT_TOPIC = str(
            self.declare_parameter(
                "decision_block_count_topic",
                DECISION_BLOCK_COUNT_TOPIC_DEFAULT,
            ).value
        )

        self.last_perfect_pose = None
        self.home_x_search_skip_z_classes = set()
        self.precision_scan_requests = {}
        self.last_precision_scan_request = None
        self.assembly_completed = False
        # legacy retry pose 저장소. 현재 block_count 검증은 새 retry pose를 만들지 않는다.
        self.last_verify_visible_poses = {}
        self.held_class = None
        # 마지막 조립 검증 성공 후 HOME 이동을 이미 수행했는지 표시한다.
        # run()에서 불필요한 중복 HOME 이동을 줄이기 위한 플래그이다.
        self.post_action_home_done = False
        self.latest_decision_block_count = None
        self.latest_decision_block_count_time = None
        self.current_insert_start_count = None
        self.current_insert_verify_after_time = None
        self.current_floor_count_at_home = None

        self.decision_count_sub = self.create_subscription(
            Int32,
            self.DECISION_BLOCK_COUNT_TOPIC,
            self.decision_block_count_cb,
            10,
        )

        # ------------------------------------------------------------- #
        # KHJ 버드아이뷰 포인트 구독
        # /khj_point 로 들어오는 offset_cm을 로봇 재촬영 XY 기준점으로 변환해
        # calc_precision_scan_xy()에서 pose.x/y 대신 사용할 수 있게 한다.
        #
        # [실측 확인된 올바른 매핑]
        #   robot_x =  offset_cm.x * KHJ_X_TO_ROBOT_X
        #   robot_y = -offset_cm.y * KHJ_Y_TO_ROBOT_Y  (부호 반전)
        # ------------------------------------------------------------- #
        self.KHJ_X_TO_ROBOT_X = float(
            self.declare_parameter("khj_x_to_robot_x", KHJ_X_TO_ROBOT_X_DEFAULT).value
        )
        self.KHJ_Y_TO_ROBOT_Y = float(
            self.declare_parameter("khj_y_to_robot_y", KHJ_Y_TO_ROBOT_Y_DEFAULT).value
        )
        self._khj_data: dict = {}
        self.khj_sub = self.create_subscription(
            String, "/khj_point", self._khj_cb, 10
        )

        self.get_logger().info(
            "[PARAM] robot1_z_off=%.1fmm, robot1_z_margin=%.1fmm, "
            "block_h=%.1fmm, wait_time=%.2fs, pre_xy_lower_mm=%.1fmm, "
            "wrist_offset_deg=%.1fdeg, precision_scan_global_y_offset_m=%.4fm, "
            "negative_x_scan_extra_enable=%s, "
            "negative_x_extra_global_x_offset_m=%.4fm, "
            "verify_scan_via_home=%s, "
            "home_x_search_enable=%s, home_x_search_step_m=%.4fm, "
            "decision_block_count_topic=%s"
            % (
                self.Z_OFF,
                self.Z_MARGIN,
                self.BLOCK_H,
                self.WAIT_TIME,
                self.PRE_XY_LOWER,
                self.WRIST_OFFSET,
                self.PRECISION_SCAN_GLOBAL_Y_OFFSET_M,
                self.NEGATIVE_X_SCAN_EXTRA_ENABLE,
                self.NEGATIVE_X_EXTRA_GLOBAL_X_OFFSET_M,
                self.VERIFY_SCAN_VIA_HOME,
                self.HOME_X_SEARCH_ENABLE,
                self.HOME_X_SEARCH_STEP_M,
                self.DECISION_BLOCK_COUNT_TOPIC,
            )
        )

    # ------------------------------------------------------------------ #
    # 공통 서비스 호출 유틸리티
    # ------------------------------------------------------------------ #

    def call(self, cli, req):
        """
        ROS2 서비스 호출 공통 함수.
        기존 코드처럼 서비스가 준비될 때까지 기다린 뒤 call_async()를 호출한다.
        """
        while not cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f"Waiting for {cli.srv_name}...")

        future = cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def decision_block_count_cb(self, msg):
        self.latest_decision_block_count = int(msg.data)
        self.latest_decision_block_count_time = time.monotonic()

    def wait_decision_block_count(self, timeout_sec=2.0, after_time=None):
        """
        decision_assembly.py가 publish하는 바닥 블록 개수를 기다린다.
        command_node에서는 executor가 subscriber를 처리하고, master_node 단독 실행에서는
        여기서 짧게 spin_once를 돌려 최신 topic을 받는다.
        """
        deadline = time.monotonic() + timeout_sec

        while rclpy.ok() and time.monotonic() < deadline:
            count = self.latest_decision_block_count
            stamp = self.latest_decision_block_count_time
            if count is not None and (after_time is None or stamp >= after_time):
                return count

            if getattr(self, "executor", None) is None:
                rclpy.spin_once(self, timeout_sec=0.05)
            else:
                time.sleep(0.05)

        return None

    def read_floor_count_at_home(self, label, timeout_sec=3.0):
        count = self.wait_decision_block_count(timeout_sec=timeout_sec)
        if count is None:
            self.get_logger().error(
                f"[COUNT] HOME에서 {label} 기준 block_count를 받지 못했습니다. "
                f"topic={self.DECISION_BLOCK_COUNT_TOPIC}"
            )
            return None

        self.get_logger().info(
            f"[COUNT] HOME 기준 {label} 바닥 블록 개수={count}"
        )
        return count

    def verify_floor_count_drop_after_home(self, label, before_count):
        """
        HOME으로 이동한 뒤 decision_assembly block_count가 정확히 1개 줄었는지 확인한다.
        성공하면 다음 단계의 기준 count를 갱신한다.
        """
        if before_count is None:
            self.get_logger().error(f"[VERIFY][COUNT] {label} 이전 HOME count가 없습니다.")
            return False

        self.get_logger().info(
            f"[VERIFY][COUNT] {label}: HOME 이동 후 바닥 블록 개수 감소 확인 시작 "
            f"(before={before_count})"
        )

        home_res = self.call(self.cli_h, Trigger.Request())
        if home_res is None or not home_res.success:
            self.get_logger().error(f"[VERIFY][COUNT] {label}: HOME 이동 실패")
            return False

        after_time = time.monotonic()
        time.sleep(self.WAIT_TIME)
        after_count = self.wait_decision_block_count(
            timeout_sec=3.0,
            after_time=after_time,
        )
        if after_count is None:
            self.get_logger().error(
                f"[VERIFY][COUNT] {label}: HOME 이동 후 새 block_count를 받지 못했습니다."
            )
            return False

        expected_count = before_count - 1
        self.get_logger().info(
            f"[VERIFY][COUNT] {label}: before={before_count}, after={after_count}, "
            f"expected={expected_count}"
        )

        if after_count != expected_count:
            self.get_logger().warn(
                f"[VERIFY][COUNT] {label}: 바닥 블록 개수가 정확히 1개 줄지 않았습니다. "
                "실패로 판단합니다."
            )
            return False

        self.current_floor_count_at_home = after_count
        self.current_insert_start_count = after_count
        self.current_insert_verify_after_time = None
        self.get_logger().info(
            f"[VERIFY][COUNT] {label}: 바닥 블록 개수가 1개 줄었습니다. 정상으로 판단합니다."
        )
        return True

    def to_vision_target_id(self, target):
        """
        사용자가 넣은 class name 또는 숫자 ID를 vision_node용 target_id로 변환한다.
        기존 CLASS_TO_TARGET_ID 매핑 유지.
        """
        target = str(target).strip()

        for prefix in ("count_", "far_"):
            if target.startswith(prefix):
                target = target[len(prefix):]

        if target.isdigit():
            return target

        target_id = CLASS_TO_TARGET_ID.get(target)
        if target_id is None:
            self.get_logger().error(f"vision_node.py ID 매핑 없음: {target}")
            return target

        return target_id

    def request_target_pose(self, target, local_id=0):
        """
        /get_target_pose 서비스에 인식 요청을 보낸다.
        """
        target_id = self.to_vision_target_id(target)
        req = GetTargetPose.Request(
            target_color=target_id
        )
        return self.call(self.cli_v, req)

    def move_robot_end(self):
        """
        robot_node.py에 정의된 end_joint로 이동한다.
        HOME 이동 후 최종 대기 자세로 보내는 용도이다.
        """
        self.get_logger().info("[END] robot1 end_joint 이동")
        return self.call(
            self.cli_r,
            GetTargetPose.Request(target_size="END"),
        )

    def move_robot2_end(self):
        """
        robot_node.py에 정의된 robot2 end_joint로 이동한다.
        조립 완료 후 robot2도 최종 대기 자세로 보내는 용도이다.
        """
        self.get_logger().info("[END] robot2 end_joint 이동")
        return self.call(
            self.cli_r2,
            GetTargetPose.Request(target_size="END"),
        )

    def move_robot2_assembly_joint(self):
        """
        robot_node.py에 정의된 robot2 assembly_joint로 이동한다.
        조립 시작 시 decision_assembly 카메라 시야/작업 공간을 비우기 위한 자세이다.
        """
        self.get_logger().info("[INIT] robot2 assembly_joint 이동")
        return self.call(
            self.cli_r2,
            GetTargetPose.Request(target_size="ASSEMBLY_JOINT"),
        )

    def scan_all_blocks_at_home(self):
        """
        조립 시작 전 전면 카메라 ID 매핑을 갱신하고 birdseye 위치를 동결한다.
        실패해도 조립 자체는 기존 vision fallback을 사용할 수 있으므로 경고만 남긴다.
        """
        def call_trigger_if_available(cli, label, service_timeout_sec=2.0, response_timeout_sec=15.0):
            if not cli.wait_for_service(timeout_sec=service_timeout_sec):
                self.get_logger().warn(f"[INIT] {label} 서비스 없음. 건너뜁니다.")
                return None
            future = cli.call_async(Trigger.Request())
            deadline = time.monotonic() + response_timeout_sec
            while rclpy.ok() and not future.done() and time.monotonic() < deadline:
                time.sleep(0.02)
            if not future.done():
                self.get_logger().warn(
                    f"[INIT] {label} 응답 타임아웃({response_timeout_sec:.1f}s). 건너뜁니다."
                )
                return None
            return future.result()

        self.get_logger().info("[INIT] 전체 블록 스캔 요청: /scan_all_blocks")
        scan_res = call_trigger_if_available(
            self.cli_scan_all_blocks,
            "/scan_all_blocks",
            response_timeout_sec=20.0,
        )
        if scan_res is None or not scan_res.success:
            self.get_logger().warn("[INIT] /scan_all_blocks 실패. 기존 인식 경로로 계속 진행합니다.")
            return False

        time.sleep(1.0)

        self.get_logger().info("[INIT] birdseye 위치 동결 요청: /lock_positions")
        lock_res = call_trigger_if_available(
            self.cli_lock_positions,
            "/lock_positions",
            response_timeout_sec=5.0,
        )
        if lock_res is None or not lock_res.success:
            self.get_logger().warn("[INIT] /lock_positions 실패. birdseye 최신값으로 계속 진행합니다.")
            return False

        return True

    def drop_assembly(self, drop_target_size):
        """
        완성된 조립체를 지정된 관절 위치로 이동한 뒤 그리퍼를 열어 내려놓고 HOME으로 복귀한다.
        drop_target_size: "ASSEMBLY_DROP_S" / "ASSEMBLY_DROP_M" / "ASSEMBLY_DROP_L"
        """
        self.get_logger().info(f"[ASSEMBLY DROP] {drop_target_size} 위치로 이동")
        res = self.call(
            self.cli_r,
            GetTargetPose.Request(target_size=drop_target_size),
        )
        if res is None or not res.success:
            self.get_logger().error(f"[ASSEMBLY DROP] {drop_target_size} 이동 실패")
            return False
        time.sleep(self.WAIT_TIME)

        self.get_logger().info("[ASSEMBLY DROP] gripper open")
        self.call(self.cli_g, SetBool.Request(data=False))
        self.held_class = None
        time.sleep(self.WAIT_TIME)

        self.get_logger().info("[ASSEMBLY DROP] HOME 복귀")
        self.call(self.cli_h, Trigger.Request())
        time.sleep(self.WAIT_TIME)
        return True

    def is_excluded_pose(self, pose, exclude_poses, tolerance_m=POSE_EXCLUSION_XY_TOLERANCE_M):
        if pose is None or not exclude_poses:
            return False

        for excluded in exclude_poses:
            if excluded is None:
                continue
            dist_xy = math.hypot(pose.x - excluded.x, pose.y - excluded.y)
            if dist_xy <= tolerance_m:
                self.get_logger().warn(
                    f"[VISION][EXCLUDE] {getattr(pose, 'class_name', '')} pose가 "
                    f"이미 사용한 pose와 {dist_xy * 1000.0:.1f}mm 이내라 제외합니다."
                )
                return True

        return False

    def find_target_with_retry(
        self,
        color,
        enable_home_x_search=True,
        local_id=0,
        exclude_poses=None,
    ):
        """
        먼저 현재 시야에서 1회 인식한다.
        실패하면 HOME 기준 x축 -/+ 방향으로 이동하며 추가 인식한다.
        """
        p = self.request_target_pose(color, local_id=local_id)

        if p is not None and p.success and not self.is_excluded_pose(p, exclude_poses):
            self.get_logger().info(
                f"[VISION] {color} local_id={local_id} 인식 성공: "
                f"x={p.x:.4f}m, y={p.y:.4f}m, z={p.z:.4f}m, "
                f"yaw={p.yaw:.1f}deg, class={getattr(p, 'class_name', '')}"
            )
            return p

        if not (enable_home_x_search and self.HOME_X_SEARCH_ENABLE):
            self.get_logger().error(f"[{color}] 타겟 인식 실패 또는 제외된 pose만 감지")
            return None

        self.get_logger().warn(
            f"[{color}] 현재 시야에서 인식 실패. "
            f"HOME 기준 x축 +/-{self.HOME_X_SEARCH_STEP_M * 1000.0:.0f}mm 탐색을 시작합니다."
        )

        for x_offset in (-self.HOME_X_SEARCH_STEP_M, self.HOME_X_SEARCH_STEP_M):
            self.get_logger().info(
                f"[HOME X SEARCH] HOME 복귀 후 z={self.PRE_XY_LOWER:.1f}mm 낮춘 뒤 "
                f"x_offset={x_offset:.4f}m 위치에서 {color} 탐색"
            )
            self.call(self.cli_h, Trigger.Request())
            time.sleep(self.WAIT_TIME)

            self.call(
                self.cli_r,
                GetTargetPose.Request(
                    z=self.PRE_XY_LOWER,
                    target_size="Z",
                ),
            )
            time.sleep(self.WAIT_TIME)

            self.call(
                self.cli_r,
                GetTargetPose.Request(
                    x=x_offset,
                    y=0.0,
                    z=0.0,
                    yaw=0.0,
                    target_size="HOME_X_SEARCH",
                ),
            )
            time.sleep(self.WAIT_TIME)

            p = self.request_target_pose(color, local_id=local_id)
            if p is not None and p.success and not self.is_excluded_pose(p, exclude_poses):
                self.home_x_search_skip_z_classes.add(str(color).strip())
                self.get_logger().info(
                    f"[HOME X SEARCH] {color} local_id={local_id} 인식 성공: "
                    f"x_offset={x_offset:.4f}m, "
                    f"x={p.x:.4f}m, y={p.y:.4f}m, z={p.z:.4f}m, "
                    f"yaw={p.yaw:.1f}deg, class={getattr(p, 'class_name', '')}"
                )
                return p

        self.get_logger().warn(
            f"[HOME X SEARCH] {color} 좌우 탐색 실패. HOME으로 복귀합니다."
        )
        self.call(self.cli_h, Trigger.Request())
        time.sleep(self.WAIT_TIME)
        self.get_logger().error(f"[{color}] 타겟 인식 실패")
        return None

    # ------------------------------------------------------------------ #
    # yaw / 좌표 계산 유틸리티
    # ------------------------------------------------------------------ #

    def normalize_yaw(self, yaw):
        """
        yaw를 -90도 ~ +90도 범위로 정리한다.

        180도 강제 뒤집기 보정은 하지 않는다.
        단순히 로봇 손목이 너무 큰 각도로 돌지 않도록 최소 정규화만 수행한다.
        """
        while yaw > 90.0:
            yaw -= 180.0
        while yaw < -90.0:
            yaw += 180.0
        return yaw

    def is_2x2_pose(self, pose):
        """
        pose.class_name이 2x2_로 시작하는지 확인한다.
        """
        return str(getattr(pose, "class_name", "")).startswith("2x2_")

    def fold_2x2_yaw(self, yaw):
        """
        2x2 블록은 회전 대칭성이 크므로 yaw를 더 작은 범위로 접는다.

        180도 강제 뒤집기 로직은 아니다.
        yaw를 안정적으로 줄이기 위한 최소 보정만 수행한다.
        """
        yaw = self.normalize_yaw(yaw)

        if yaw > 45.0:
            yaw -= 90.0
        elif yaw < -45.0:
            yaw += 90.0

        return yaw

    def get_wrist_yaw(self, pose, yaw_offset=0.0):
        """
        실제 로봇 손목에 넣을 yaw를 계산한다.

        - 2x2는 fold_2x2_yaw 적용
        - 그 외 블록은 normalize_yaw만 적용
        - scan_yaw를 180도 돌리는 로직은 없음
        """
        raw_yaw = pose.yaw + yaw_offset

        if self.is_2x2_pose(pose):
            target_yaw = self.fold_2x2_yaw(raw_yaw)
            self.get_logger().info(
                f"[YAW][2x2 FOLD] {getattr(pose, 'class_name', '')}: "
                f"vision_yaw={pose.yaw:.1f}deg, yaw_offset={yaw_offset:.1f}deg "
                f"-> wrist_yaw={target_yaw:.1f}deg"
            )
            return target_yaw

        target_yaw = self.normalize_yaw(raw_yaw)
        self.get_logger().info(
            f"[YAW] {getattr(pose, 'class_name', '')}: "
            f"vision_yaw={pose.yaw:.1f}deg, yaw_offset={yaw_offset:.1f}deg "
            f"-> wrist_yaw={target_yaw:.1f}deg"
        )
        return target_yaw

    def pose_yaw_for_xy_offset(self, pose):
        """
        스터드 단위 offset을 실제 x/y로 변환할 때 사용할 yaw.

        기존 offset 구조는 유지한다.
        단, 2x2 블록은 fold된 yaw를 사용한다.
        """
        if self.is_2x2_pose(pose):
            return self.fold_2x2_yaw(pose.yaw)

        return pose.yaw

    def calc_target_xy(self, pose, offset_studs_x=0.0, offset_studs_y=0.0):
        """
        블록 중심 pose에서 stud offset을 적용한 실제 목표 x/y를 계산한다.

        offset_studs_x, offset_studs_y는 기존 build_* 시퀀스에서 쓰던
        블록 기준 offset 구조를 유지하기 위한 값이다.

        주의:
        - 이 함수는 조립 목표 위치 계산용이다.
        - 정밀 재촬영 위치 계산에는 사용하지 않는다.
        """
        dx = offset_studs_x * STUD_PITCH
        dy = offset_studs_y * STUD_PITCH

        yaw_rad = math.radians(self.pose_yaw_for_xy_offset(pose))

        real_offset_x = dx * math.cos(yaw_rad) - dy * math.sin(yaw_rad)
        real_offset_y = dx * math.sin(yaw_rad) + dy * math.cos(yaw_rad)

        target_x = pose.x + real_offset_x
        target_y = pose.y + real_offset_y

        self.get_logger().info(
            f"[OFFSET] center=({pose.x:.4f}, {pose.y:.4f})m, "
            f"stud_offset=({offset_studs_x:.2f}, {offset_studs_y:.2f}), "
            f"target=({target_x:.4f}, {target_y:.4f})m"
        )

        return target_x, target_y

    def _khj_cb(self, msg: String):
        try:
            self._khj_data = json.loads(msg.data)
        except Exception as e:
            self.get_logger().warn(f"[KHJ] 파싱 실패: {e}")

    def get_khj_scan_base_xy(self, class_name: str, local_id: int = 0):
        """
        /khj_point 에서 class_name 이 일치하는 local_id 번째 항목의
        offset_cm → 로봇 팔 XY (m) 를 반환한다.
        매칭이 없으면 None 반환.
        """
        if not self._khj_data:
            return None

        matches = sorted(
            [
                (id_str, v)
                for id_str, v in self._khj_data.items()
                if str(v.get("class_name", "")).strip() == str(class_name).strip()
            ],
            key=lambda kv: int(kv[0]) if kv[0].isdigit() else 0,
        )

        if not matches or local_id >= len(matches):
            self.get_logger().warn(
                f"[KHJ] '{class_name}' local_id={local_id} 매칭 없음 "
                f"(khj 항목 수={len(self._khj_data)})"
            )
            return None

        _, block = matches[local_id]
        offset_cm = block.get("offset_cm", {})
        ox = float(offset_cm.get("x", 0.0))
        oy = float(offset_cm.get("y", 0.0))

        # 세션 이전 부호 그대로 유지
        robot_x = ox * self.KHJ_X_TO_ROBOT_X
        robot_y = -oy * self.KHJ_Y_TO_ROBOT_Y

        self.get_logger().info(
            f"[KHJ] '{class_name}' local_id={local_id}: "
            f"offset_cm=({ox:.2f}, {oy:.2f}) → "
            f"robot_x=ox*{self.KHJ_X_TO_ROBOT_X:.5f}={robot_x:.4f}m, "
            f"robot_y=oy*{self.KHJ_Y_TO_ROBOT_Y:.5f}={robot_y:.4f}m"
        )
        return robot_x, robot_y

    def calc_precision_scan_xy(self, pose, local_id=0):
        """
        정밀 재촬영 위치 계산.

        기본 재촬영 위치는 블록 yaw와 관계없이 global 좌표계 기준으로만 계산한다.

            scan_x = pose.x
            scan_y = pose.y + precision_scan_global_y_offset_m

        /khj_point에 해당 블록 데이터가 있으면 pose.x / pose.y 대신
        KHJ offset_cm을 로봇 좌표로 변환한 base_x / base_y를 기준으로 사용한다.

        추가 보정:
            base_x > 0이면 scan_x에 negative_x_extra_global_x_offset_m을 더한다.
        """
        class_name = str(getattr(pose, "class_name", "")).strip()
        khj_xy = self.get_khj_scan_base_xy(class_name, local_id=local_id) if class_name else None
        if khj_xy is not None:
            base_x, base_y = khj_xy
            base_source = "KHJ"
        else:
            base_x, base_y = pose.x, pose.y
            base_source = "vision_pose"

        # 1) 기본 재촬영 위치: 블록 중심 기준 global y offset만 적용
        scan_x = base_x
        scan_y = base_y + self.PRECISION_SCAN_GLOBAL_Y_OFFSET_M

        negative_x_extra_applied = False

        # 2) 기준 x가 양수인 경우에만 global x축 추가 보정 적용
        if self.NEGATIVE_X_SCAN_EXTRA_ENABLE and base_x > 0.0:
            scan_x += self.NEGATIVE_X_EXTRA_GLOBAL_X_OFFSET_M
            negative_x_extra_applied = True

            self.get_logger().info(
                "[PRECISION][EXTRA X] 기준 x가 양수이므로 "
                "global x 추가 이동 적용: "
                f"base_source={base_source}, base_x={base_x:.4f}m, "
                f"extra_global_x={self.NEGATIVE_X_EXTRA_GLOBAL_X_OFFSET_M:.4f}m"
            )
        else:
            self.get_logger().info(
                "[PRECISION][NORMAL X] 기준 x가 양수가 아니므로 "
                "global x 추가 이동 없음: "
                f"base_source={base_source}, base_x={base_x:.4f}m"
            )

        self.get_logger().info(
            "[PRECISION] 재촬영 위치: yaw 무시, "
            f"base_source={base_source}, base=({base_x:.4f}, {base_y:.4f})m, "
            f"global_y_offset={self.PRECISION_SCAN_GLOBAL_Y_OFFSET_M:.4f}m, "
            f"negative_x_extra_applied={negative_x_extra_applied} -> "
            f"scan_x={scan_x:.4f}m, scan_y={scan_y:.4f}m"
        )

        return scan_x, scan_y

    def calc_approach_from_pose(
        self,
        pose,
        layer_index=0,
        yaw_offset=0.0,
        offset_studs_x=0.0,
        offset_studs_y=0.0,
    ):
        """
        pose 하나를 기준으로 실제 APPROACH 목표를 계산한다.

        반환값:
            target_x, target_y, z_approach, target_yaw

        z 계산은 기존 구조를 유지한다.

            z_move = pose.z * 1000 + Z_OFF - BLOCK_H * layer_index
            z_approach = z_move - Z_MARGIN
        """
        target_x, target_y = self.calc_target_xy(
            pose,
            offset_studs_x=offset_studs_x,
            offset_studs_y=offset_studs_y,
        )

        target_yaw = self.get_wrist_yaw(
            pose,
            yaw_offset=yaw_offset,
        )

        z_move = (pose.z * 1000.0 + self.Z_OFF) - (
            self.BLOCK_H * layer_index
        )
        z_approach = z_move - self.Z_MARGIN

        self.get_logger().info(
            f"[APPROACH CALC] class={getattr(pose, 'class_name', '')}, "
            f"layer={layer_index}, x={target_x:.4f}m, y={target_y:.4f}m, "
            f"z_move={z_move:.1f}mm, z_approach={z_approach:.1f}mm, "
            f"yaw={target_yaw:.1f}deg"
        )

        return target_x, target_y, z_approach, target_yaw

    # ------------------------------------------------------------------ #
    # 모션 프리미티브
    # ------------------------------------------------------------------ #

    def move_fast_from_pose(
        self,
        pose,
        layer_index=0,
        yaw_offset=0.0,
        offset_studs_x=0.0,
        offset_studs_y=0.0,
        enable_precision_scan=True,
        do_final_lower=True,
        local_id=0,
    ):
        """
        인식된 pose를 기준으로 로봇을 목표 위치까지 이동시키는 핵심 함수.

        동작 순서:

        1. 처음 인식된 pose로부터 조립/파지 목표 APPROACH 좌표를 계산한다.
           이 값은 정밀 재촬영 실패 시 fallback으로 사용한다.

        2. enable_precision_scan=True이고 pose.class_name이 있으면,
           대상 블록 중심 바로 위가 아니라 global offset이 적용된 위치로 이동한다.

           기본:
               scan_x = pose.x
               scan_y = pose.y + precision_scan_global_y_offset_m

           추가:
               pose.x < 0이면 scan_x에 negative_x_extra_global_x_offset_m 적용

        3. 재촬영 위치로 이동한 뒤 같은 class_name으로 /get_target_pose를 다시 호출한다.

        4. 재촬영 성공 시 refined_pose 기준으로 APPROACH 좌표를 다시 계산한다.

        5. 재촬영 실패 시 처음 pose 기준 APPROACH 절대좌표로 fallback한다.
           기존처럼 scan 위치와 목표 위치 차이를 계산해서 APPROACH_DELTA로 보내지 않는다.
           안정성을 위해 항상 target_size='APPROACH' 절대좌표 이동을 사용한다.

        6. APPROACH 위치로 이동한 뒤 필요하면 Z 방향으로 self.Z_MARGIN만큼 최종 하강한다.
        """
        if pose is None:
            self.get_logger().error("[MOVE] pose가 None입니다.")
            return False

        original_class_name = str(getattr(pose, "class_name", "")).strip()
        skip_precision_z = original_class_name in self.home_x_search_skip_z_classes
        if skip_precision_z:
            self.home_x_search_skip_z_classes.discard(original_class_name)
            self.get_logger().info(
                "[PRECISION] HOME X SEARCH에서 이미 재촬영 높이로 인식한 pose라 "
                "정밀 재촬영 이동 시 Z 이동만 생략합니다."
            )

        self.get_logger().info(
            f"[MOVE START] class={original_class_name}, "
            f"raw_pose: x={pose.x:.4f}m, y={pose.y:.4f}m, "
            f"z={pose.z:.4f}m, yaw={pose.yaw:.1f}deg"
        )

        # ------------------------------------------------------------- #
        # 1. 첫 인식 pose 기준 목표 계산
        #    재촬영 실패 시 이 값을 그대로 사용한다.
        # ------------------------------------------------------------- #
        target_x, target_y, z_approach, target_yaw = self.calc_approach_from_pose(
            pose,
            layer_index=layer_index,
            yaw_offset=yaw_offset,
            offset_studs_x=offset_studs_x,
            offset_studs_y=offset_studs_y,
        )

        used_refined_pose = False

        # ------------------------------------------------------------- #
        # 2. 정밀 재촬영
        # ------------------------------------------------------------- #
        if enable_precision_scan and original_class_name:
            scan_x, scan_y = self.calc_precision_scan_xy(pose, local_id=local_id)
            scan_request = {
                "x": scan_x,
                "y": scan_y,
                "z": self.PRE_XY_LOWER,
                "target_size": "Z_then_XY",
                "skip_z": skip_precision_z,
                "expected_x": pose.x,
                "expected_y": pose.y,
                "local_id": int(local_id),
            }
            self.precision_scan_requests[original_class_name] = scan_request
            self.last_precision_scan_request = scan_request

            if not skip_precision_z:
                self.get_logger().info(
                    f"[PRECISION] Z 먼저 하강: z={self.PRE_XY_LOWER:.1f}mm"
                )
                self.call(
                    self.cli_r,
                    GetTargetPose.Request(z=self.PRE_XY_LOWER, target_size="Z"),
                )
                time.sleep(0.5)

            self.get_logger().info(
                f"[PRECISION] XY 이동: x={scan_x:.4f}m, y={scan_y:.4f}m"
            )
            self.call(
                self.cli_r,
                GetTargetPose.Request(x=scan_x, y=scan_y, z=0.0, target_size="XY"),
            )
            time.sleep(0.5)

            self.get_logger().info(
                f"[PRECISION] 재촬영 요청: class={original_class_name}"
            )

            refined_pose = self.find_target_with_retry(
                original_class_name,
                enable_home_x_search=False,
                local_id=local_id,
            )

            if refined_pose:
                self.get_logger().info(
                    f"[PRECISION] 재촬영 성공. refined pose 기준으로 목표 재계산."
                )

                (
                    target_x,
                    target_y,
                    z_approach,
                    target_yaw,
                ) = self.calc_approach_from_pose(
                    refined_pose,
                    layer_index=layer_index,
                    yaw_offset=yaw_offset,
                    offset_studs_x=offset_studs_x,
                    offset_studs_y=offset_studs_y,
                )

                used_refined_pose = True
                scan_request["expected_x"] = refined_pose.x
                scan_request["expected_y"] = refined_pose.y

            else:
                self.get_logger().warn(
                    "[PRECISION] 재촬영 실패. "
                    "처음 인식한 pose 기준 APPROACH 절대좌표로 fallback합니다. "
                    "APPROACH_DELTA는 사용하지 않습니다."
                )

        else:
            self.get_logger().info(
                "[PRECISION] class_name이 없거나 enable_precision_scan=False라서 "
                "재촬영 없이 처음 pose 기준으로 접근합니다."
            )

        # ------------------------------------------------------------- #
        # 3. 최종 APPROACH 절대좌표 이동
        # ------------------------------------------------------------- #
        source_text = "refined_pose" if used_refined_pose else "original_pose"

        self.get_logger().info(
            f"[APPROACH] source={source_text}, "
            f"x={target_x:.4f}m, y={target_y:.4f}m, "
            f"z_approach={z_approach:.1f}mm, yaw={target_yaw:.1f}deg, "
            f"target_size=APPROACH"
        )

        self.call(
            self.cli_r,
            GetTargetPose.Request(
                x=target_x,
                y=target_y,
                z=z_approach,
                yaw=target_yaw,
                target_size="APPROACH",
            ),
        )
        time.sleep(0.5)

        if do_final_lower:
            # ------------------------------------------------------------- #
            # 4. 최종 Z 하강
            # ------------------------------------------------------------- #
            self.get_logger().info(
                f"[LOWER] Z 방향 최종 하강: z={self.Z_MARGIN:.1f}mm, target_size=Z"
            )

            self.call(
                self.cli_r,
                GetTargetPose.Request(
                    z=self.Z_MARGIN,
                    target_size="Z",
                ),
            )
            time.sleep(0.5)
        else:
            self.get_logger().info(
                "[LOWER] do_final_lower=False. APPROACH 후 추가 Z 하강을 생략합니다."
            )

        return True

    def verify_insert_from_saved_scan(self, target_color, home_after_success=False):
        """
        조립 후 HOME으로 이동해 decision_assembly.py가 publish하는 바닥 블록 개수 변화로
        조립 성공 여부를 판단한다.

        판정 기준:
            - 이전 HOME 기준 block_count보다 조립 후 HOME block_count가 정확히 1개 줄면 성공
            - 같거나 다른 값이면 실패
        """
        target_color = str(target_color).strip()
        before_count = self.current_floor_count_at_home
        if before_count is None:
            before_count = self.current_insert_start_count

        ok = self.verify_floor_count_drop_after_home(
            f"{target_color} 조립",
            before_count,
        )
        if ok:
            self.last_verify_visible_poses.pop(target_color, None)
            if home_after_success:
                self.post_action_home_done = True
        return ok

    def pick_target(
        self,
        color,
        layer_index=0,
        offset_studs_x=0.0,
        offset_studs_y=0.0,
        local_id=0,
    ):
        """
        바닥에 있는 target 블록을 인식하고 파지한다.

        이전 조립 검증에서는 성공 전까지 그리퍼를 열지 않으므로,
        다음 파지를 시작할 때는 항상 gripper open 상태를 먼저 보장한다.
        """
        self.get_logger().info(f"--- PICK TARGET: [{color.upper()}] ---")

        home_res = self.call(self.cli_h, Trigger.Request())
        if home_res is None or not home_res.success:
            self.get_logger().error("[PICK][COUNT] 파지 전 HOME 이동 실패")
            return False
        time.sleep(0.5)

        before_count = self.read_floor_count_at_home(f"{color} 파지 전")
        if before_count is None:
            return False

        self.get_logger().info("[GRIPPER] ensure open before pick")
        self.call(self.cli_g, SetBool.Request(data=False))
        self.held_class = None

        p = self.find_target_with_retry(color, local_id=local_id)

        if not p:
            return False

        if not self.move_fast_from_pose(
            p,
            layer_index=layer_index,
            yaw_offset=self.WRIST_OFFSET,
            offset_studs_x=offset_studs_x,
            offset_studs_y=offset_studs_y,
            local_id=local_id,
        ):
            return False

        self.get_logger().info("[GRIPPER] close")
        self.call(self.cli_g, SetBool.Request(data=True))
        self.held_class = str(color).strip()

        self.get_logger().info(
            "[PICK] 파지 완료. HOME으로 이동해 decision_assembly block_count 1개 감소를 확인합니다."
        )

        if not self.verify_floor_count_drop_after_home(f"{color} 파지", before_count):
            self.get_logger().warn(f"[PICK][COUNT] {color} 파지 검증 실패")
            return False

        return True

    def recover_after_failed_insert(self, held_before_insert, release_gripper):
        """
        조립 검증 실패 후 다음 재시도를 위한 상태를 만든다.

        변경된 검증 방식에서는 조립 직후 그리퍼를 열지 않는다.
        따라서 실패로 판단되더라도 블록/조립체는 아직 그리퍼가 잡고 있다고 보고,
        HOME 경유 없이 같은 held 상태로 현재 검증 위치에서 재시도한다.
        """
        if held_before_insert:
            self.held_class = held_before_insert
            self.get_logger().warn(
                f"[VERIFY] 조립 실패. 그리퍼를 열지 않았으므로 "
                f"들고 있는 {held_before_insert} 상태 그대로 재시도합니다."
            )
        else:
            self.get_logger().warn(
                "[VERIFY] 조립 실패. 직전 held_class 정보는 없지만 "
                "그리퍼를 열지 않은 상태로 현재 위치에서 재시도합니다."
            )

        self.get_logger().info(
            "[VERIFY] 이미 HOME에서 block_count를 확인한 상태입니다. "
            "그리퍼를 닫은 채 다음 재조립 시퀀스를 다시 실행합니다."
        )
        return True

    def visual_insert(
        self,
        target_color,
        layer_index,
        release_gripper=True,
        yaw_offset=0.0,
        offset_studs_x=0.0,
        offset_studs_y=0.0,
        do_final_lower=True,
        local_id=0,
        base_pose=None,
        pre_khj_scan=False,
    ):
        """
        조립 대상 블록을 카메라로 인식한 뒤 그 위에 현재 들고 있는 블록을 적층한다.

        핵심 흐름:
        - 조립 직후에는 release_gripper=True여도 그리퍼를 열지 않는다.
        - HOME 이동 전 별도 Z 상승은 하지 않는다. Z_MARGIN은 접근/최종 하강에만 쓴다.
        - 조립 전후 /decision_assembly/block_count가 줄어들면 성공으로 판단한다.

        pre_khj_scan=True:
          매 시도마다 KHJ 스캔 위치로 먼저 이동한 뒤 그 자리에서 비전을 수행한다.
          move_fast_from_pose 내부의 precision scan(재이동)은 생략한다.
        """
        target_color = str(target_color).strip()

        self.get_logger().info(
            f"--- VISUAL STACK: [{target_color.upper()}] "
            f"(Layer +{layer_index}, X Offset: {offset_studs_x}, "
            f"Y Offset: {offset_studs_y}, local_id={local_id}) ---"
        )
        time.sleep(1.0)

        attempt = 1
        retry_pose = None
        while rclpy.ok():
            self.get_logger().info(
                f"[INSERT ATTEMPT] {target_color} visual_insert {attempt}회차"
            )

            # ── 바닥 블록 개수 읽기 (HOME 위치, 스캔 이동 전) ──────────────
            # pre_khj_scan 사용 시 KHJ 스캔 위치로 이동하기 전에 읽어야
            # 로봇 팔이 시야를 가리지 않는 상태의 정확한 count를 얻는다.
            if self.current_floor_count_at_home is None:
                self.current_floor_count_at_home = self.read_floor_count_at_home(
                    f"{target_color} 조립 전"
                )
                if self.current_floor_count_at_home is None:
                    return False
            # ─────────────────────────────────────────────────────────────

            use_verify_pose = retry_pose is not None
            if use_verify_pose:
                p = retry_pose
                retry_pose = None
                self.get_logger().info(
                    "[RETRY WITHOUT HOME] 저장된 retry pose로 바로 재조립합니다."
                )
            elif base_pose is not None:
                p = base_pose
            else:
                if pre_khj_scan:
                    # KHJ 스캔 위치로 먼저 이동한 뒤 그 자리에서 비전
                    # 순서: Z 먼저 하강 → XY 이동
                    khj_xy = self.get_khj_scan_base_xy(target_color, local_id=local_id)
                    if khj_xy is not None:
                        base_x, base_y = khj_xy
                        scan_x = base_x
                        if self.NEGATIVE_X_SCAN_EXTRA_ENABLE and base_x > 0.0:
                            scan_x += self.NEGATIVE_X_EXTRA_GLOBAL_X_OFFSET_M
                        scan_y = base_y + self.PRECISION_SCAN_GLOBAL_Y_OFFSET_M
                        self.get_logger().info(
                            f"[PRE-KHJ-SCAN] '{target_color}': "
                            f"Z 하강 {self.PRE_XY_LOWER:.1f}mm → "
                            f"XY 이동 ({scan_x:.4f}m, {scan_y:.4f}m)"
                        )
                        self.call(
                            self.cli_r,
                            GetTargetPose.Request(z=self.PRE_XY_LOWER, target_size="Z"),
                        )
                        time.sleep(self.WAIT_TIME)
                        self.call(
                            self.cli_r,
                            GetTargetPose.Request(
                                x=scan_x, y=scan_y, z=0.0, target_size="XY"
                            ),
                        )
                        time.sleep(self.WAIT_TIME)
                    else:
                        self.get_logger().warn(
                            f"[PRE-KHJ-SCAN] '{target_color}' KHJ 데이터 없음. "
                            "현재 위치에서 비전 진행합니다."
                        )
                    p = self.find_target_with_retry(
                        target_color,
                        enable_home_x_search=False,
                        local_id=local_id,
                    )
                else:
                    p = self.find_target_with_retry(target_color, local_id=local_id)
            if not p:
                return False

            self.last_perfect_pose = p
            held_before_insert = self.held_class
            self.current_insert_start_count = self.current_floor_count_at_home
            self.current_insert_verify_after_time = None
            self.get_logger().info(
                f"[VERIFY][COUNT] {target_color} 조립 기준 HOME count="
                f"{self.current_insert_start_count}"
            )

            # pre_khj_scan=True이면 이미 스캔 위치에서 비전을 마쳤으므로 내부 precision scan 생략
            skip_precision = use_verify_pose or (pre_khj_scan and base_pose is None)
            if not self.move_fast_from_pose(
                p,
                layer_index=layer_index,
                yaw_offset=yaw_offset + self.WRIST_OFFSET,
                offset_studs_x=offset_studs_x,
                offset_studs_y=offset_studs_y,
                enable_precision_scan=not skip_precision,
                do_final_lower=do_final_lower,
                local_id=local_id,
            ):
                return False
            self.current_insert_verify_after_time = time.monotonic()

            if release_gripper:
                self.get_logger().info(
                    "[GRIPPER] release_gripper=True이지만 검증 전이므로 open하지 않습니다. "
                    "그리퍼를 닫은 상태로 block_count 검증까지 유지합니다."
                )
            else:
                self.get_logger().info(
                    "[GRIPPER] release_gripper=False, 그리퍼 유지 상태로 block_count 검증합니다."
                )

            self.get_logger().info(
                "[VERIFY] 조립 직후 HOME으로 이동해 "
                "decision_assembly block_count 1개 감소를 검증합니다."
            )

            if self.verify_insert_from_saved_scan(
                target_color,
                home_after_success=True,
            ):
                if release_gripper:
                    # 검증 성공은 base 블록이 조립체에 의해 가려졌거나 함께 들렸다는 뜻이다.
                    # 실제 gripper open 명령은 보내지 않고, 다음 pick 시작 시 open한다.
                    self.held_class = None
                else:
                    # 중간 조립 단계는 조립체를 계속 들고 가야 하므로 held 상태를 유지한다.
                    self.held_class = held_before_insert
                return True

            retry_pose = self.last_verify_visible_poses.pop(target_color, None)
            if retry_pose is not None:
                self.get_logger().info(
                    "[RETRY WITHOUT HOME] HOME 복귀 없이 현재 검증 위치에서 "
                    "방금 확인한 base pose를 다음 재조립에 사용합니다."
                )
            if not self.recover_after_failed_insert(
                held_before_insert,
                release_gripper,
            ):
                return False

            attempt += 1

        return False

    def blind_insert(
        self,
        base_pose,
        layer_index,
        yaw_offset=0.0,
        release_gripper=True,
        offset_studs_x=0.0,
        offset_studs_y=0.0,
    ):
        """
        이미 저장해 둔 base_pose를 기준으로 적층한다.

        이름은 blind_insert지만, 기존 코드 흐름처럼 move_fast_from_pose()를 통과하므로
        class_name이 살아 있으면 정밀 재촬영을 한 번 시도한다.
        재촬영 실패 시에는 저장된 base_pose 기준 절대좌표로 fallback한다.

        visual_insert()와 동일하게:
        - 검증 전에는 그리퍼를 열지 않는다.
        - HOME 이동 전 별도 Z 상승은 하지 않는다. Z_MARGIN은 접근/최종 하강에만 쓴다.
        - 조립 전후 /decision_assembly/block_count가 줄어들면 성공으로 판단한다.
        """
        self.get_logger().info(
            f"--- BLIND STACK / MEMORY POSE: "
            f"Layer {layer_index}, X Offset: {offset_studs_x}, "
            f"Y Offset: {offset_studs_y} ---"
        )
        time.sleep(1.0)

        target_color = str(getattr(base_pose, "class_name", "")).strip()
        attempt = 1
        retry_pose = None

        while rclpy.ok():
            self.get_logger().info(
                f"[INSERT ATTEMPT] blind_insert {attempt}회차 "
                f"(target={target_color or 'unknown'})"
            )
            held_before_insert = self.held_class

            use_verify_pose = retry_pose is not None
            if use_verify_pose:
                current_base_pose = retry_pose
                retry_pose = None
                self.get_logger().info(
                    "[RETRY WITHOUT HOME] 저장된 retry pose로 바로 재조립합니다."
                )
            else:
                current_base_pose = base_pose

            if self.current_floor_count_at_home is None:
                self.current_floor_count_at_home = self.read_floor_count_at_home(
                    f"{target_color or 'unknown'} 조립 전"
                )
                if self.current_floor_count_at_home is None:
                    return False
            self.current_insert_start_count = self.current_floor_count_at_home
            self.current_insert_verify_after_time = None
            self.get_logger().info(
                f"[VERIFY][COUNT] {target_color or 'unknown'} 조립 기준 HOME count="
                f"{self.current_insert_start_count}"
            )

            if not self.move_fast_from_pose(
                current_base_pose,
                layer_index=layer_index,
                yaw_offset=yaw_offset + self.WRIST_OFFSET,
                offset_studs_x=offset_studs_x,
                offset_studs_y=offset_studs_y,
                enable_precision_scan=not use_verify_pose,
            ):
                return False
            self.current_insert_verify_after_time = time.monotonic()

            if release_gripper:
                self.get_logger().info(
                    "[GRIPPER] release_gripper=True이지만 검증 전이므로 open하지 않습니다. "
                    "그리퍼를 닫은 상태로 block_count 검증까지 유지합니다."
                )
            else:
                self.get_logger().info(
                    "[GRIPPER] release_gripper=False, 그리퍼 유지 상태로 block_count 검증합니다."
                )

            self.get_logger().info(
                "[VERIFY] 조립 직후 HOME으로 이동해 "
                "decision_assembly block_count 1개 감소를 검증합니다."
            )

            if not target_color:
                self.get_logger().warn(
                    "[VERIFY] base_pose.class_name이 없어 blind_insert 검증을 생략합니다. "
                    "검증 전 gripper open은 수행하지 않았습니다."
                )
                if release_gripper:
                    self.held_class = None
                else:
                    self.held_class = held_before_insert
                return True

            if self.verify_insert_from_saved_scan(
                target_color,
                home_after_success=True,
            ):
                if release_gripper:
                    self.held_class = None
                else:
                    self.held_class = held_before_insert
                return True

            retry_pose = self.last_verify_visible_poses.pop(target_color, None)
            if retry_pose is not None:
                self.get_logger().info(
                    "[RETRY WITHOUT HOME] HOME 복귀 없이 현재 검증 위치에서 "
                    "방금 확인한 base pose를 다음 재조립에 사용합니다."
                )
            if not self.recover_after_failed_insert(
                held_before_insert,
                release_gripper,
            ):
                return False

            attempt += 1

        return False

    # ------------------------------------------------------------------ #
    # 조립 시퀀스
    # ------------------------------------------------------------------ #

    def build_battery(self):
        self.get_logger().info("[배터리] 노란색(Pick) -> 파란색(Base)")

        if not self.pick_target("2x2_yellow"):
            return

        self.call(self.cli_h, Trigger.Request())

        if self.visual_insert("2x2_blue", layer_index=0.7):
            self.assembly_completed = True
            self.get_logger().info("[완료] 배터리")

    def build_magnet(self):
        self.get_logger().info("[자석] 파란색(Pick) -> 빨간색(Base)")

        if not self.pick_target("2x2_blue"):
            return

        self.call(self.cli_h, Trigger.Request())

        if self.visual_insert("2x2_red", layer_index=0.7):
            self.assembly_completed = True
            self.get_logger().info("[완료] 자석")

    def build_e_stop(self):
        self.get_logger().info("[비상정지] 빨간색(Pick) -> 노란색4x2(Base)")

        if not self.pick_target("2x2_red"):
            return

        self.call(self.cli_h, Trigger.Request())

        if self.visual_insert("4x2_yellow", layer_index=0.7, yaw_offset=0.0):
            self.assembly_completed = True
            self.get_logger().info("[완료] 비상정지")

    def build_carrot(self):
        self.get_logger().info("[당근] 초록(Pick) -> 노랑(그리퍼 유지) -> 노랑(Base)")

        if not self.pick_target("2x2_green"):
            return

        self.call(self.cli_h, Trigger.Request())

        if not self.visual_insert(
            "2x2_yellow",
            layer_index=0.7,
            release_gripper=False,
            pre_khj_scan=True,
            local_id=0,
        ):
            return

        self.call(self.cli_h, Trigger.Request())

        if self.visual_insert(
            "2x2_yellow",
            layer_index=1.7,
            pre_khj_scan=True,
            local_id=1,
        ):
            self.assembly_completed = True
            self.get_logger().info("[완료] 당근")

    def build_traffic_light(self):
        self.get_logger().info("[신호등] 빨강(Pick) -> 노랑(그리퍼 유지) -> 초록(Base)")

        if not self.pick_target("2x2_red"):
            return

        self.call(self.cli_h, Trigger.Request())

        if not self.visual_insert(
            "2x2_yellow",
            layer_index=0.7,
            release_gripper=False,
        ):
            return

        self.call(self.cli_h, Trigger.Request())

        if self.visual_insert("2x2_green", layer_index=1.7):
            self.assembly_completed = True
            self.get_logger().info("[완료] 신호등")

    def build_small_tree(self):
        self.get_logger().info(
            "[작은 나무] 초록2x2(Pick) -> 초록4x2(그리퍼 유지) -> 노랑2x2(Base)"
        )

        if not self.pick_target("2x2_green"):
            return

        self.call(self.cli_h, Trigger.Request())

        if not self.visual_insert(
            "4x2_green",
            layer_index=0.7,
            release_gripper=False,
        ):
            return

        self.call(self.cli_h, Trigger.Request())

        if self.visual_insert("2x2_yellow", layer_index=1.5):
            self.assembly_completed = True
            self.get_logger().info("[완료] 작은 나무")

    def build_hammer(self):
        self.get_logger().info(
            "[망치] 파랑4x2(Pick) -> 빨강2x2(그리퍼 유지) -> 빨강2x2(Base)"
        )

        if not self.pick_target("4x2_blue"):
            return

        self.call(self.cli_h, Trigger.Request())

        if self.visual_insert(
            "2x2_red",
            layer_index=0.63,
            release_gripper=False,
            offset_studs_y= -0.18,#몸쪽으로 음수, 바깥쪽으로 양수
            pre_khj_scan=True,
            local_id=0,
        ):
            self.get_logger().info(
                "[망치] 첫 번째 2x2_red 결합 검증 완료. 남은 2x2_red 결합 시퀀스로 진행합니다."
            )

            self.call(self.cli_h, Trigger.Request())

            if self.visual_insert(
                "2x2_red",
                layer_index=1.8,
                pre_khj_scan=True,
                local_id=1,
            ):
                self.assembly_completed = True
                self.get_logger().info("[완료] 망치")
            return

        self.get_logger().warn("첫 번째 2x2_red 결합 검증 실패. 망치 조립 취소.")

    def build_big_carrot(self):
        self.get_logger().info(
            "[큰 당근] 초록2x2(Pick) -> 노랑4x2 -> 노랑2x2 -> 노랑2x2(Base)"
        )

        if not self.pick_target("2x2_green"):
            return

        self.call(self.cli_h, Trigger.Request())

        if not self.visual_insert(
            "4x2_yellow",
            layer_index=0.7,
            release_gripper=False,
        ):
            return

        self.call(self.cli_h, Trigger.Request())

        if not self.visual_insert(
            "2x2_yellow",
            layer_index=1.5,
            release_gripper=False,
            pre_khj_scan=True,
            local_id=0,
        ):
            return

        self.call(self.cli_h, Trigger.Request())

        if self.visual_insert(
            "2x2_yellow",
            layer_index=2.5,
            release_gripper=False,
            pre_khj_scan=True,
            local_id=1,
        ):
            self.assembly_completed = True
            self.get_logger().info("[완료] 큰 당근")

    def build_burger(self):
        self.get_logger().info(
            "[버거] 4x2_yellow(Pick) -> KHJ 스캔 -> 4x2_red 결합 -> "
            "2x2_red 결합 -> 4x2_yellow 최종 결합"
        )

        self.get_logger().info("[Phase 1] 4x2_yellow 파지")
        if not self.pick_target("4x2_yellow"):
            self.get_logger().warn("4x2_yellow 파지 실패. 조립 취소.")
            return
        # pick_target은 HOME + verify로 끝남

        # Phase 간 HOME 복귀 + 바닥 블록 개수 확인 (다음 phase 기준값 갱신)
        self.call(self.cli_h, Trigger.Request())
        time.sleep(0.3)
        self.current_floor_count_at_home = None  # visual_insert 시작 시 HOME에서 재취득

        self.get_logger().info(
            "[Phase 2] KHJ 스캔 위치 이동 후 4x2_red 비전 → 결합 (그리퍼 유지)"
        )
        if not self.visual_insert(
            "4x2_red",
            layer_index=0.7,
            offset_studs_y=-1.0,
            release_gripper=False,
            pre_khj_scan=True,
        ):
            self.get_logger().warn("4x2_red 인식/결합 실패. 조립 취소.")
            return

        # Phase 간 HOME 복귀 + 바닥 블록 개수 확인
        self.call(self.cli_h, Trigger.Request())
        time.sleep(0.3)
        self.current_floor_count_at_home = None

        self.get_logger().info(
            "[Phase 3] KHJ 스캔 위치 이동 후 2x2_red 비전 → 결합 (그리퍼 유지)"
        )
        if not self.visual_insert(
            "2x2_red",
            layer_index=0.7,
            offset_studs_y=2.0,
            release_gripper=False,
            pre_khj_scan=True,
        ):
            self.get_logger().warn("2x2_red 인식/결합 실패. 조립 취소.")
            return

        # Phase 간 HOME 복귀 + 바닥 블록 개수 확인
        self.call(self.cli_h, Trigger.Request())
        time.sleep(0.3)
        self.current_floor_count_at_home = None

        self.get_logger().info(
            "[Phase 4] KHJ 스캔 위치 이동 후 4x2_yellow 비전 → 최종 결합"
        )
        if self.visual_insert(
            "4x2_yellow",
            layer_index=1.4,
            pre_khj_scan=True,
        ):
            self.assembly_completed = True
            self.get_logger().info("[완료] 버거")
            return

        self.get_logger().warn("버거 최종 결합 실패.")

    # def build_ice_cream(self):
    #     self.get_logger().info("[아이스크림] 모듈형 조립 전략")

    #     self.get_logger().info("[Phase 1] 하단 조립: 노랑4x2(Pick) -> 노랑2x2(Base)")
    #     if not self.pick_target("4x2_yellow"):
    #         return

    #     self.call(self.cli_h, Trigger.Request())

    #     if not self.visual_insert("2x2_yellow", layer_index=0.7):
    #         self.get_logger().warn("하단 모듈 조립 실패")
    #         return

    #     self.get_logger().info("[Phase 2-1] 파랑2x2(Pick) -> 빨강2x2 옆에 배치")
    #     self.call(self.cli_h, Trigger.Request())

    #     if not self.pick_target("2x2_blue"):
    #         return

    #     self.call(self.cli_h, Trigger.Request())

    #     if not self.visual_insert(
    #         "2x2_red",
    #         layer_index=0,
    #         offset_studs_y=2.0,
    #     ):
    #         self.get_logger().warn("파란색 블록 배치 실패")
    #         return

    #     self.get_logger().info("[Phase 2-2] 초록2x2(Pick) -> 파랑2x2(Base) 결합")
    #     self.call(self.cli_h, Trigger.Request())

    #     if not self.pick_target("2x2_green"):
    #         return

    #     self.call(self.cli_h, Trigger.Request())

    #     if not self.visual_insert(
    #         "2x2_blue",
    #         layer_index=0.7,
    #         offset_studs_y=-1.0,
    #         release_gripper=False,
    #     ):
    #         self.get_logger().warn("상단 모듈 결합 실패")
    #         return

    #     self.get_logger().info("[Phase 3] 상단 모듈 -> 하단 모듈 4x2_yellow 위에 최종 결합")
    #     self.call(self.cli_h, Trigger.Request())

    #     if self.visual_insert("4x2_yellow", layer_index=1.7):
    #         self.get_logger().info("[완료] 아이스크림")

    # def build_ice_cream(self):
    #     self.get_logger().info(
    #         "[아이스크림] 새 시퀀스: "
    #         "2x2_red -> 4x2_yellow(y+1.0), "
    #         "2x2_blue -> 4x2_yellow(y-1.0), "
    #         "2x2_green -> 2x2_blue(y+1.0) 후 그리퍼 유지, "
    #         "2x2_yellow 최종 결합"
    #     )

    #     saved_bottom_yellow_pose = None
    #     self.get_logger().info("[Phase 0] 최종 base용 2x2_yellow 위치 저장")
    #     saved_bottom_yellow_pose = self.find_target_with_retry("2x2_yellow")
    #     if not saved_bottom_yellow_pose:
    #         self.get_logger().warn(
    #             "2x2_yellow 위치 저장 실패. 최종 단계에서 메모리 fallback 없이 진행합니다."
    #         )

    #     self.call(self.cli_h, Trigger.Request())
    #     time.sleep(1.0)

    #     self.get_logger().info("[Phase 1] 2x2_red 파지")
    #     if not self.pick_target("2x2_red"):
    #         self.get_logger().warn("2x2_red 파지 실패. 아이스크림 조립 취소.")
    #         return

    #     self.call(self.cli_h, Trigger.Request())
    #     time.sleep(1.0)

    #     self.get_logger().info(
    #         "[Phase 2] 들고 있는 2x2_red -> 4x2_yellow 결합 "
    #         "(offset_studs_y=+1.0)"
    #     )
    #     if not self.visual_insert(
    #         "4x2_yellow",
    #         layer_index=0.7,
    #         offset_studs_y=1.0,
    #     ):
    #         self.get_logger().warn("4x2_yellow 인식/결합 실패. 아이스크림 조립 취소.")
    #         return

    #     self.call(self.cli_h, Trigger.Request())
    #     time.sleep(1.0)

    #     self.get_logger().info("[Phase 3] 2x2_blue 파지")
    #     if not self.pick_target("2x2_blue"):
    #         self.get_logger().warn("2x2_blue 파지 실패. 아이스크림 조립 취소.")
    #         return

    #     self.call(self.cli_h, Trigger.Request())
    #     time.sleep(1.0)

    #     self.get_logger().info(
    #         "[Phase 4] 들고 있는 2x2_blue -> 4x2_yellow 결합 "
    #         "(offset_studs_y=-1.0)"
    #     )
    #     if not self.visual_insert(
    #         "4x2_yellow",
    #         layer_index=0.7,
    #         offset_studs_y=-1.0,
    #     ):
    #         self.get_logger().warn("4x2_yellow 인식/결합 실패. 아이스크림 조립 취소.")
    #         return

    #     self.call(self.cli_h, Trigger.Request())
    #     time.sleep(1.0)

    #     self.get_logger().info("[Phase 5] 2x2_green 파지")
    #     if not self.pick_target("2x2_green"):
    #         self.get_logger().warn("2x2_green 파지 실패. 아이스크림 조립 취소.")
    #         return

    #     self.call(self.cli_h, Trigger.Request())
    #     time.sleep(1.0)

    #     self.get_logger().info(
    #         "[Phase 6] 들고 있는 2x2_green -> 2x2_blue 결합 "
    #         "(offset_studs_y=+1.0, 그리퍼 유지)"
    #     )
    #     if not self.visual_insert(
    #         "2x2_blue",
    #         layer_index=1.7,
    #         offset_studs_y=1.0,
    #         release_gripper=False,
    #     ):
    #         self.get_logger().warn("2x2_blue 인식/결합 실패. 아이스크림 조립 취소.")
    #         return

    #     self.call(self.cli_h, Trigger.Request())
    #     time.sleep(1.0)

    #     self.get_logger().info("[Phase 7] 최종 결합: 현재 조립체 -> 2x2_yellow")

    #     if self.visual_insert(
    #         "2x2_yellow",
    #         layer_index=2.7,
    #         release_gripper=True,
    #     ):
    #         self.get_logger().info("[완료] 아이스크림")
    #         return

    #     self.get_logger().warn(
    #         "[Phase 7] 현재 시야에서 2x2_yellow 인식 실패. "
    #         "저장한 위치로 fallback합니다."
    #     )

    #     if saved_bottom_yellow_pose and self.blind_insert(
    #         saved_bottom_yellow_pose,
    #         layer_index=2.7,
    #         release_gripper=True,
    #     ):
    #         self.get_logger().info("[완료] 아이스크림 (저장 위치 fallback)")
    #         return

    #     self.get_logger().warn("저장된 2x2_yellow 위치도 없어 아이스크림 최종 결합 실패.")


    def build_ice_cream(self):
        self.get_logger().info(
            "[아이스크림] 새 시퀀스: "
            "2x2_red -> 4x2_yellow(y+1.0), "
            "2x2_blue -> 4x2_yellow(y-1.0), "
            "2x2_green -> 2x2_blue(y+1.0) 후 그리퍼 유지, "
            "2x2_yellow 최종 결합"
        )

        self.call(self.cli_h, Trigger.Request())
        time.sleep(0.3)

        self.get_logger().info("[Phase 1] 2x2_red 파지")
        if not self.pick_target("2x2_red"):
            self.get_logger().warn("2x2_red 파지 실패. 아이스크림 조립 취소.")
            return

        self.call(self.cli_h, Trigger.Request())
        time.sleep(0.3)

        self.get_logger().info(
            "[Phase 2] 들고 있는 2x2_red -> 4x2_yellow 결합 "
            "(offset_studs_y=+1.0)"
        )
        if not self.visual_insert(
            "4x2_yellow",
            layer_index=0.7,
            offset_studs_y=1.0,
        ):
            self.get_logger().warn("4x2_yellow 인식/결합 실패. 아이스크림 조립 취소.")
            return

        self.call(self.cli_h, Trigger.Request())
        time.sleep(0.3)

        self.get_logger().info("[Phase 3] 2x2_blue 파지")
        if not self.pick_target("2x2_blue"):
            self.get_logger().warn("2x2_blue 파지 실패. 아이스크림 조립 취소.")
            return

        self.call(self.cli_h, Trigger.Request())
        time.sleep(0.3)

        self.get_logger().info(
            "[Phase 4] 들고 있는 2x2_blue -> 2x2_red 옆에 결합 "
            "(offset_studs_y=-1.0)"
        )
        if not self.visual_insert(
            "2x2_red",
            layer_index=0.5,
            offset_studs_y=-2.0,
        ):
            self.get_logger().warn("2x2_red 인식/결합 실패. 아이스크림 조립 취소.")
            return

        self.call(self.cli_h, Trigger.Request())
        time.sleep(0.3)

        self.get_logger().info("[Phase 5] 2x2_green 파지")
        if not self.pick_target("2x2_green"):
            self.get_logger().warn("2x2_green 파지 실패. 아이스크림 조립 취소.")
            return

        self.call(self.cli_h, Trigger.Request())
        time.sleep(0.3)

        self.get_logger().info(
            "[Phase 6] 들고 있는 2x2_green -> 2x2_blue 결합 "
            "(offset_studs_y=+1.0, 그리퍼 유지)"
        )
        if not self.visual_insert(
            "2x2_blue",
            layer_index=1.4,
            offset_studs_y=1.0,
            release_gripper=False,
        ):
            self.get_logger().warn("2x2_blue 인식/결합 실패. 아이스크림 조립 취소.")
            return

        self.call(self.cli_h, Trigger.Request())
        time.sleep(0.3)

        self.get_logger().info("[Phase 7] 최종 결합: 현재 조립체 -> 2x2_yellow")

        if self.visual_insert(
            "2x2_yellow",
            layer_index=2.5,
            release_gripper=True,
        ):
            self.assembly_completed = True
            self.get_logger().info("[완료] 아이스크림")
            return

        self.get_logger().warn("[Phase 7] 2x2_yellow 인식 실패. 아이스크림 최종 결합 실패.")



    # def build_big_tree(self):
    #     self.get_logger().info("[큰 나무] 6x2 베이스 조립 후 2x2_yellow 중앙에 최종 결합")

    #     self.get_logger().info("[초기화] 맨 처음 2x2_yellow 위치 스캔 및 기억")
    #     saved_yellow_pose = self.find_target_with_retry("2x2_yellow")
    #     if not saved_yellow_pose:
    #         self.get_logger().warn("바닥에 2x2_yellow가 없습니다. 조립 취소.")
    #         return

    #     self.call(self.cli_h, Trigger.Request())

    #     self.get_logger().info("[Phase 1] 4x2_green 파지 -> 4x2_green 결합, 그리퍼 유지")
    #     if not self.pick_target("4x2_green", offset_studs_y=0):
    #         return

    #     self.call(self.cli_h, Trigger.Request())

    #     if not self.visual_insert(
    #         "4x2_green",
    #         layer_index=0.7,
    #         offset_studs_y=-1.0,
    #         release_gripper=False,
    #     ):
    #         return

    #     self.call(self.cli_h, Trigger.Request())
    #     time.sleep(1.0)

    #     self.get_logger().info("[Phase 2] 바닥의 다른 2x2_green 스캔 및 6x2 조립")
    #     saved_6x2_pose = self.find_target_with_retry("2x2_green")
    #     if not saved_6x2_pose:
    #         self.get_logger().warn("바닥에 다른 2x2_green가 없습니다.")
    #         return

    #     if not self.blind_insert(
    #         saved_6x2_pose,
    #         layer_index=0.7,
    #         offset_studs_y=2.0,
    #         release_gripper=True,
    #     ):
    #         return

    #     self.call(self.cli_h, Trigger.Request())

    #     self.get_logger().info("[Phase 3] 2x2_green 파지 -> 6x2 중심에 결합, 그리퍼 유지")
    #     if not self.pick_target("2x2_green", offset_studs_y=-0.2):
    #         return

    #     self.call(self.cli_h, Trigger.Request())

    #     if not self.blind_insert(
    #         saved_6x2_pose,
    #         layer_index=-1.7,
    #         offset_studs_y=0.0,
    #         release_gripper=False,
    #     ):
    #         return

    #     self.call(self.cli_h, Trigger.Request())

    #     self.get_logger().info("[Phase 4] 덩어리를 2x2_yellow 중앙에 최종 결합")
    #     if self.blind_insert(
    #         saved_yellow_pose,
    #         layer_index=2.7,
    #         offset_studs_y=0.0,
    #     ):
    #         self.get_logger().info("[완료] 큰 나무")

    def build_big_tree(self):
        self.get_logger().info(
            "[큰 나무] 버거형 시퀀스: "
            "2x2_green(Pick) -> 4x2_green -> 4x2_green(offset) "
            "-> 2x2_green(offset) -> 2x2_yellow 최종 결합"
        )

        # ------------------------------------------------------------- #
        # 튜닝용 offset 값
        #
        # 기존 big_tree에서 사용하던 주요 offset 흐름을 최대한 유지했다.
        # 실제 조립 방향이 반대면 부호만 바꾸면 된다.
        #
        # Phase 3:
        #   현재 조립체를 다른 4x2_green 위에 꽂을 때 사용할 y offset
        #
        # Phase 4:
        #   현재 조립체를 2x2_green 위에 꽂을 때 사용할 y offset
        #
        # Phase 5:
        #   최종 2x2_yellow 위에 꽂을 때 사용할 y offset
        # ------------------------------------------------------------- #
        big_tree_4x2_green_offset_y = -1.0
        big_tree_2x2_green_offset_y = 2.0
        big_tree_final_yellow_offset_y = 0.0

        self.call(self.cli_h, Trigger.Request())
        time.sleep(0.3)

        # ------------------------------------------------------------- #
        # Phase 1. 가장 윗층이 될 2x2_green 파지
        #
        # 기존 방식은 4x2_green을 먼저 잡고 베이스를 만든 뒤
        # 나중에 2x2_green을 다시 집었지만,
        # 이번 방식은 가장 위의 2x2_green부터 집고 시작한다.
        # ------------------------------------------------------------- #
        self.get_logger().info("[Phase 1] 가장 윗층 2x2_green 파지")
        if not self.pick_target("2x2_green"):
            self.get_logger().warn("2x2_green 파지 실패. 큰 나무 조립 취소.")
            return

        self.call(self.cli_h, Trigger.Request())
        time.sleep(0.3)

        # ------------------------------------------------------------- #
        # Phase 2. 들고 있는 2x2_green을 4x2_green 위에 결합
        #
        # release_gripper=False:
        #   결합 후에도 그리퍼를 열지 않고 계속 잡고 간다.
        #
        # layer_index=0.7:
        #   바닥에 있는 4x2_green 위에 한 층 올리는 기존 결합 높이.
        # ------------------------------------------------------------- #
        self.get_logger().info(
            "[Phase 2] 들고 있는 2x2_green -> 4x2_green 결합 "
            "(offset 없음, 그리퍼 유지)"
        )

        if not self.visual_insert(
            "4x2_green",
            layer_index=0.7,
            offset_studs_y=0.0,
            release_gripper=False,
            pre_khj_scan=True,
            local_id=0,
        ):
            self.get_logger().warn("첫 번째 4x2_green 인식/결합 실패. 큰 나무 조립 취소.")
            return

        self.call(self.cli_h, Trigger.Request())
        time.sleep(0.3)

        # ------------------------------------------------------------- #
        # Phase 3. 현재 조립체를 다른 4x2_green 위에 offset 적용해서 결합
        #
        # 기존 big_tree에서 4x2_green끼리 결합할 때 쓰던
        # offset_studs_y=-1.0 흐름을 유지했다.
        #
        # 만약 방향이 반대면 big_tree_4x2_green_offset_y를 +1.0으로 바꾸면 된다.
        # ------------------------------------------------------------- #
        self.get_logger().info(
            "[Phase 3] 현재 조립체 -> 다른 4x2_green 결합 "
            f"(offset_studs_y={big_tree_4x2_green_offset_y}, 그리퍼 유지)"
        )

        if not self.visual_insert(
            "4x2_green",
            layer_index=1.7,
            offset_studs_y=-1.0,
            release_gripper=False,
            pre_khj_scan=True,
            local_id=1,
        ):
            self.get_logger().warn("두 번째 4x2_green 인식/결합 실패. 큰 나무 조립 취소.")
            return

        self.call(self.cli_h, Trigger.Request())
        time.sleep(0.3)

        # ------------------------------------------------------------- #
        # Phase 4. 현재 조립체를 2x2_green 위에 offset 적용해서 결합
        #
        # 기존 big_tree에서 6x2 베이스를 만들 때 사용하던
        # offset_studs_y=2.0 흐름을 가져왔다.
        #
        # 실제 조립 위치가 너무 멀거나 반대면
        # big_tree_2x2_green_offset_y 값을 조정하면 된다.
        # ------------------------------------------------------------- #
        self.get_logger().info(
            "[Phase 4] 현재 조립체 -> 2x2_green 결합 "
            f"(offset_studs_y={big_tree_2x2_green_offset_y}, 그리퍼 유지)"
        )

        if not self.visual_insert(
            "2x2_green",
            layer_index=1.7,
            offset_studs_y=2.0,
            release_gripper=False,
            pre_khj_scan=True,
            local_id=1,
        ):
            self.get_logger().warn("2x2_green 인식/결합 실패. 큰 나무 조립 취소.")
            return

        self.call(self.cli_h, Trigger.Request())
        time.sleep(0.3)

        # ------------------------------------------------------------- #
        # Phase 5. 최종 결합: 현재 들고 있는 전체 조립체를 2x2_yellow 위에 결합
        #
        # 먼저 visual_insert를 시도한다.
        # 실패하면 Phase 0에서 저장해 둔 saved_yellow_pose로 blind_insert fallback한다.
        #
        # layer_index는 버거 최종 결합과 유사하게 1.4를 우선 사용했다.
        # 너무 깊게 들어가거나 충돌하면 1.2~1.3으로 낮춰보고,
        # 덜 들어가면 1.5 정도로 올려서 테스트하면 된다.
        # ------------------------------------------------------------- #
        self.get_logger().info(
            "[Phase 5] 최종 결합: 현재 조립체 -> 2x2_yellow "
            f"(offset_studs_y={big_tree_final_yellow_offset_y})"
        )

        if self.visual_insert(
            "2x2_yellow",
            layer_index=2.4,
            offset_studs_y=0.0,
            release_gripper=True,
        ):
            self.assembly_completed = True
            self.get_logger().info("[완료] 큰 나무")
            return

        self.get_logger().warn("[Phase 5] 2x2_yellow 인식 실패. 큰 나무 최종 결합 실패.")

    # ------------------------------------------------------------------ #
    # 메인 루프
    # ------------------------------------------------------------------ #

    def run(self):
        self.get_logger().info("STARTING ASSEMBLY SEQUENCE (Keyboard Select Mode)")

        self.get_logger().info("[INIT] HOME / robot2 assembly_joint / gripper open 동시 시작")
        for cli in (self.cli_h, self.cli_r2, self.cli_g):
            while not cli.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f"Waiting for {cli.srv_name}...")

        future_h  = self.cli_h.call_async(Trigger.Request())
        future_r2 = self.cli_r2.call_async(GetTargetPose.Request(target_size="ASSEMBLY_JOINT"))
        future_g  = self.cli_g.call_async(SetBool.Request(data=False))

        rclpy.spin_until_future_complete(self, future_h)
        rclpy.spin_until_future_complete(self, future_r2)
        rclpy.spin_until_future_complete(self, future_g)

        home_res     = future_h.result()
        assembly_res = future_r2.result()

        if home_res is None or not home_res.success:
            self.get_logger().error("시작 HOME 이동 실패. 조립을 시작하지 않습니다.")
            return
        if assembly_res is None or not assembly_res.success:
            self.get_logger().error("robot2 assembly_joint 이동 실패. 조립을 시작하지 않습니다.")
            return
        self.get_logger().info("[INIT] 초기화 완료")

        actions = {
            "1":          self.build_battery,
            "battery":   self.build_battery,
            "배터리":     self.build_battery,

            "2":          self.build_magnet,
            "magnet":    self.build_magnet,
            "자석":       self.build_magnet,

            "3":          self.build_e_stop,
            "estop":     self.build_e_stop,
            "비상정지":   self.build_e_stop,

            "4":          self.build_carrot,
            "carrot":    self.build_carrot,
            "당근":       self.build_carrot,

            "5":          self.build_traffic_light,
            "traffic":   self.build_traffic_light,
            "신호등":     self.build_traffic_light,

            "6":          self.build_small_tree,
            "tree":      self.build_small_tree,
            "작은나무":   self.build_small_tree,

            "7":          self.build_hammer,
            "hammer":    self.build_hammer,
            "망치":       self.build_hammer,

            "8":          self.build_big_carrot,
            "bigcarrot": self.build_big_carrot,
            "큰당근":     self.build_big_carrot,

            "9":          self.build_burger,
            "burger":    self.build_burger,
            "버거":       self.build_burger,

            "10":         self.build_ice_cream,
            "icecream":  self.build_ice_cream,
            "아이스크림": self.build_ice_cream,

            "11":         self.build_big_tree,
            "big_tree":  self.build_big_tree,
            "큰나무":     self.build_big_tree,
        }

        print("\n=== Master Node Assembly Keyboard Select ===")
        print("1: 배터리 / 2: 자석 / 3: 비상정지 / 4: 당근 / 5: 신호등 / 6: 작은나무")
        print("7: 망치 / 8: 큰당근 / 9: 버거 / 10: 아이스크림 / 11: 큰나무")
        print("q: 종료")

        while rclpy.ok():
            user_input = (
                input("\n조립할 항목을 선택하세요 [1~11/q]: ")
                .strip()
                .replace(" ", "")
                .lower()
            )

            if user_input in ("q", "quit", "exit", "종료"):
                self.get_logger().info("조립 시퀀스를 종료합니다.")
                break

            action = actions.get(user_input)
            if action is None:
                print("잘못된 입력입니다. 1~11 또는 q 중에서 선택하세요.")
                continue

            self.get_logger().info(f"작업 시작: {user_input}")
            self.post_action_home_done = False

            action()

            if self.post_action_home_done:
                self.get_logger().info(
                    "[POST] 검증 성공 후 이미 HOME 이동 완료. 추가 HOME 이동은 생략합니다."
                )
            else:
                self.get_logger().info(
                    "[POST] 검증 성공 HOME 이동이 없었으므로 안전하게 HOME 이동"
                )
                self.call(self.cli_h, Trigger.Request())
                time.sleep(1.0)

            self.get_logger().info("개별 조립 완료")

        self.get_logger().info("[END] HOME 이동")
        self.call(self.cli_h, Trigger.Request())
        time.sleep(1.0)
        self.get_logger().info("[END] gripper open 후 END 이동")
        self.call(self.cli_g, SetBool.Request(data=False))
        time.sleep(self.WAIT_TIME)
        self.move_robot_end()
        self.move_robot2_end()

        self.get_logger().info("ALL SEQUENCE DONE")


def main():
    rclpy.init()
    node = MasterNode()

    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()



# import rclpy
# from rclpy.node import Node

# from srvs_pkg.srv import GetTargetPose
# from std_srvs.srv import SetBool, Trigger

# import time
# import math


# # --------------------------------------------------------------------- #
# # 모듈 레벨 기본값
# # --------------------------------------------------------------------- #

# Z_OFF_DEFAULT = -100.0 # 카메라 Z → 로봇 툴 Z 변환 오프셋 (mm)
# Z_MARGIN_DEFAULT = 22.0 # APPROACH 후 최종 하강 여유 거리 (mm)
# BLOCK_H_DEFAULT = 19.5 # 듀플로/레고 블록 한 층 높이 (mm)
# WAIT_TIME_DEFAULT = 1.5 # 모션 완료 후 대기 시간 (s)
# PRE_XY_LOWER_DEFAULT = 100.0 # 정밀 재촬영을 위한 중간 Z 높이 (mm)
# PRE_XY_LOWER_Z2_DEFAULT = -150.0 # 2번째 이후 블록 재촬영 Z 높이 (mm)
# PRECISION_SCAN_DELTA_X_OFFSET_DEFAULT = 0.0 # 2번째 이후 재촬영 delta X 보정 (m)
# PRECISION_SCAN_DELTA_Y_OFFSET_DEFAULT = 0.1 # 2번째 이후 재촬영 delta Y 보정 (m)
# WRIST_OFFSET_DEFAULT = 0.0 # 손목 추가 회전 각도 (deg)
# HOME_X_SEARCH_ENABLE_DEFAULT = True
# HOME_X_SEARCH_STEP_M_DEFAULT = 0.300
# VERIFY_SCAN_VIA_HOME_DEFAULT = False

# # 정밀 재촬영 시, 대상 블록 중심에서 global y축 방향으로 이동할 거리.
# PRECISION_SCAN_GLOBAL_Y_OFFSET_DEFAULT = 0.100

# # 처음 인식한 pose.x가 음수인 블록만 정밀 재촬영 위치를 추가 보정할지 여부
# NEGATIVE_X_SCAN_EXTRA_ENABLE_DEFAULT = True

# # pose.x < 0일 때 global x축 방향으로 추가 이동할 거리
# NEGATIVE_X_EXTRA_GLOBAL_X_OFFSET_DEFAULT = 0.030

# STUD_PITCH = 0.016 # 스터드 간격 (m)


# # 기존 CLASS_TO_TARGET_ID 매핑 유지
# CLASS_TO_TARGET_ID = {
#     "2x2_red": "1",
#     "2x2_green": "2",
#     "2x2_blue": "3",
#     "2x2_yellow": "4",

#     "4x2_red": "5",
#     "4x2_green": "6",
#     "4x2_blue": "7",
#     "4x2_yellow": "8",

#     "2x4_red": "5",
#     "2x4_green": "6",
#     "2x4_blue": "7",
#     "2x4_yellow": "8",

#     "assembly": "999",
# }


# class MasterNode(Node):
#     def __init__(self):
#         super().__init__("master_node")

#         self.cli_v = self.create_client(GetTargetPose, "/get_target_pose")
#         self.cli_r = self.create_client(GetTargetPose, "/robot1/robot_move_step")
#         self.cli_g = self.create_client(SetBool, "/control_gripper")
#         self.cli_h = self.create_client(Trigger, "/robot1/robot_home")

#         self.Z_OFF = float(self.declare_parameter("robot1_z_off", Z_OFF_DEFAULT).value)
#         self.Z_MARGIN = float(self.declare_parameter("robot1_z_margin", Z_MARGIN_DEFAULT).value)
#         self.BLOCK_H = float(self.declare_parameter("block_h", BLOCK_H_DEFAULT).value)
#         self.WAIT_TIME = float(self.declare_parameter("wait_time", WAIT_TIME_DEFAULT).value)
#         self.PRE_XY_LOWER = float(self.declare_parameter("pre_xy_lower_mm", PRE_XY_LOWER_DEFAULT).value)
#         self.PRE_XY_LOWER_Z2 = float(self.declare_parameter("pre_xy_lower_z2_mm", PRE_XY_LOWER_Z2_DEFAULT).value)
#         self.PRECISION_SCAN_DELTA_X_OFFSET_M = float(
#             self.declare_parameter("precision_scan_delta_x_offset_m", PRECISION_SCAN_DELTA_X_OFFSET_DEFAULT).value
#         )
#         self.PRECISION_SCAN_DELTA_Y_OFFSET_M = float(
#             self.declare_parameter("precision_scan_delta_y_offset_m", PRECISION_SCAN_DELTA_Y_OFFSET_DEFAULT).value
#         )
#         self.WRIST_OFFSET = float(self.declare_parameter("wrist_offset_deg", WRIST_OFFSET_DEFAULT).value)
#         self.HOME_X_SEARCH_ENABLE = bool(
#             self.declare_parameter("home_x_search_enable", HOME_X_SEARCH_ENABLE_DEFAULT).value
#         )
#         self.HOME_X_SEARCH_STEP_M = float(
#             self.declare_parameter("home_x_search_step_m", HOME_X_SEARCH_STEP_M_DEFAULT).value
#         )
#         self.VERIFY_SCAN_VIA_HOME = bool(
#             self.declare_parameter("verify_scan_via_home", VERIFY_SCAN_VIA_HOME_DEFAULT).value
#         )

#         self.PRECISION_SCAN_GLOBAL_Y_OFFSET_M = float(
#             self.declare_parameter("precision_scan_global_y_offset_m", PRECISION_SCAN_GLOBAL_Y_OFFSET_DEFAULT).value
#         )

#         self.NEGATIVE_X_SCAN_EXTRA_ENABLE = bool(
#             self.declare_parameter("negative_x_scan_extra_enable", NEGATIVE_X_SCAN_EXTRA_ENABLE_DEFAULT).value
#         )
#         self.NEGATIVE_X_EXTRA_GLOBAL_X_OFFSET_M = float(
#             self.declare_parameter("negative_x_extra_global_x_offset_m", NEGATIVE_X_EXTRA_GLOBAL_X_OFFSET_DEFAULT).value
#         )

#         self.last_perfect_pose = None
#         self.last_home_pose_xy = None
#         self.home_x_search_skip_z_classes = set()
#         self.precision_scan_requests = {}
#         self.last_precision_scan_request = None
#         self.last_verify_visible_poses = {}
#         self.held_class = None
#         self.post_action_home_done = False

#         self.get_logger().info(
#             "[PARAM] robot1_z_off=%.1fmm, robot1_z_margin=%.1fmm, block_h=%.1fmm, "
#             "wait_time=%.2fs, pre_xy_lower_mm=%.1fmm, pre_xy_lower_z2_mm=%.1fmm, wrist_offset_deg=%.1fdeg, "
#             "precision_scan_global_y_offset_m=%.4fm, precision_scan_delta_offset=(%.4f, %.4f)m, "
#             "negative_x_scan_extra_enable=%s, negative_x_extra_global_x_offset_m=%.4fm, "
#             "verify_scan_via_home=%s, home_x_search_enable=%s, home_x_search_step_m=%.4fm"
#             % (
#                 self.Z_OFF, self.Z_MARGIN, self.BLOCK_H, self.WAIT_TIME, self.PRE_XY_LOWER, self.PRE_XY_LOWER_Z2, self.WRIST_OFFSET,
#                 self.PRECISION_SCAN_GLOBAL_Y_OFFSET_M,
#                 self.PRECISION_SCAN_DELTA_X_OFFSET_M,
#                 self.PRECISION_SCAN_DELTA_Y_OFFSET_M,
#                 self.NEGATIVE_X_SCAN_EXTRA_ENABLE,
#                 self.NEGATIVE_X_EXTRA_GLOBAL_X_OFFSET_M,
#                 self.VERIFY_SCAN_VIA_HOME,
#                 self.HOME_X_SEARCH_ENABLE,
#                 self.HOME_X_SEARCH_STEP_M,
#             )
#         )

#     def call(self, cli, req):
#         while not cli.wait_for_service(timeout_sec=1.0):
#             self.get_logger().info(f"Waiting for {cli.srv_name}...")
#         future = cli.call_async(req)
#         rclpy.spin_until_future_complete(self, future)
#         return future.result()

#     def go_home(self, wait_sec=None):
#         """
#         외부 노드(wb_command_node 등)에서 호출하기 위한 HOME 이동 함수
#         """
#         res = self.call(self.cli_h, Trigger.Request())
#         if res is not None and getattr(res, "success", False):
#             self.last_home_pose_xy = None
#             self.post_action_home_done = True
#         if wait_sec is None:
#             wait_sec = self.WAIT_TIME
#         if wait_sec and wait_sec > 0.0:
#             time.sleep(wait_sec)
#         return res

#     def to_vision_target_id(self, target):
#         target = str(target).strip()
#         for prefix in ("count_", "far_"):
#             if target.startswith(prefix):
#                 target = target[len(prefix):]
#         if target.isdigit():
#             return target
#         target_id = CLASS_TO_TARGET_ID.get(target)
#         if target_id is None:
#             self.get_logger().error(f"vision_node.py ID 매핑 없음: {target}")
#             return target
#         return target_id

#     def request_target_pose(self, target):
#         req = GetTargetPose.Request(target_color=self.to_vision_target_id(target))
#         return self.call(self.cli_v, req)

#     def find_target_with_retry(self, color, enable_home_x_search=True):
#         p = self.request_target_pose(color)
#         if p is not None and p.success:
#             self.get_logger().info(f"[VISION] {color} 인식 성공: x={p.x:.4f}m, y={p.y:.4f}m, z={p.z:.4f}m, yaw={p.yaw:.1f}deg")
#             return p

#         if not (enable_home_x_search and self.HOME_X_SEARCH_ENABLE):
#             self.get_logger().error(f"[{color}] 타겟 인식 실패")
#             return None

#         self.get_logger().warn(
#             f"[{color}] 현재 시야에서 인식 실패. "
#             f"HOME 기준 x축 +/-{self.HOME_X_SEARCH_STEP_M * 1000.0:.0f}mm 탐색을 시작합니다."
#         )

#         for x_offset in (-self.HOME_X_SEARCH_STEP_M, self.HOME_X_SEARCH_STEP_M):
#             self.go_home(wait_sec=self.WAIT_TIME)

#             self.call(
#                 self.cli_r,
#                 GetTargetPose.Request(z=self.PRE_XY_LOWER, target_size="Z"),
#             )
#             time.sleep(self.WAIT_TIME)

#             self.call(
#                 self.cli_r,
#                 GetTargetPose.Request(
#                     x=x_offset,
#                     y=0.0,
#                     z=0.0,
#                     yaw=0.0,
#                     target_size="HOME_X_SEARCH",
#                 ),
#             )
#             time.sleep(self.WAIT_TIME)

#             p = self.request_target_pose(color)
#             if p is not None and p.success:
#                 self.home_x_search_skip_z_classes.add(str(color).strip())
#                 self.get_logger().info(
#                     f"[HOME X SEARCH] {color} 인식 성공: "
#                     f"x_offset={x_offset:.4f}m, x={p.x:.4f}m, y={p.y:.4f}m, "
#                     f"z={p.z:.4f}m, yaw={p.yaw:.1f}deg"
#                 )
#                 return p

#         self.get_logger().warn(f"[HOME X SEARCH] {color} 좌우 탐색 실패. HOME 복귀")
#         self.go_home(wait_sec=self.WAIT_TIME)
#         self.get_logger().error(f"[{color}] 타겟 인식 실패")
#         return None

#     def normalize_yaw(self, yaw):
#         while yaw > 90.0: yaw -= 180.0
#         while yaw < -90.0: yaw += 180.0
#         return yaw

#     def is_2x2_pose(self, pose):
#         return str(getattr(pose, "class_name", "")).startswith("2x2_")

#     def fold_2x2_yaw(self, yaw):
#         yaw = self.normalize_yaw(yaw)
#         if yaw > 45.0: yaw -= 90.0
#         elif yaw < -45.0: yaw += 90.0
#         return yaw

#     def get_wrist_yaw(self, pose, yaw_offset=0.0):
#         raw_yaw = pose.yaw + yaw_offset
#         target_yaw = self.fold_2x2_yaw(raw_yaw) if self.is_2x2_pose(pose) else self.normalize_yaw(raw_yaw)
#         return target_yaw

#     def pose_yaw_for_xy_offset(self, pose):
#         if self.is_2x2_pose(pose): return self.fold_2x2_yaw(pose.yaw)
#         return pose.yaw

#     def calc_target_xy(self, pose, offset_studs_x=0.0, offset_studs_y=0.0):
#         dx = offset_studs_x * STUD_PITCH
#         dy = offset_studs_y * STUD_PITCH
#         yaw_rad = math.radians(self.pose_yaw_for_xy_offset(pose))
#         real_offset_x = dx * math.cos(yaw_rad) - dy * math.sin(yaw_rad)
#         real_offset_y = dx * math.sin(yaw_rad) + dy * math.cos(yaw_rad)
#         target_x = pose.x + real_offset_x
#         target_y = pose.y + real_offset_y
#         return target_x, target_y

#     def calc_precision_scan_xy(self, pose, apply_y_offset=True):
#         scan_x = pose.x
#         y_offset = self.PRECISION_SCAN_GLOBAL_Y_OFFSET_M if apply_y_offset else 0.0
#         scan_y = pose.y + y_offset
#         negative_x_extra_applied = False

#         if self.NEGATIVE_X_SCAN_EXTRA_ENABLE and pose.x > 0.0:
#             scan_x += self.NEGATIVE_X_EXTRA_GLOBAL_X_OFFSET_M
#             negative_x_extra_applied = True

#         self.get_logger().info(
#             f"[PRECISION] 재촬영 위치: global_y_offset={y_offset:.4f}m, "
#             f"negative_x_extra_applied={negative_x_extra_applied} -> scan_x={scan_x:.4f}m, scan_y={scan_y:.4f}m"
#         )
#         return scan_x, scan_y

#     def calc_approach_from_pose(self, pose, layer_index=0, yaw_offset=0.0, offset_studs_x=0.0, offset_studs_y=0.0):
#         target_x, target_y = self.calc_target_xy(pose, offset_studs_x=offset_studs_x, offset_studs_y=offset_studs_y)
#         target_yaw = self.get_wrist_yaw(pose, yaw_offset=yaw_offset)
#         z_move = (pose.z * 1000.0 + self.Z_OFF) - (self.BLOCK_H * layer_index)
#         z_approach = z_move - self.Z_MARGIN
#         return target_x, target_y, z_approach, target_yaw

#     def move_fast_from_pose(
#         self,
#         pose,
#         layer_index=0,
#         yaw_offset=0.0,
#         offset_studs_x=0.0,
#         offset_studs_y=0.0,
#         enable_precision_scan=True,
#         scan_z_mm=None,
#     ):
#         if pose is None: return False
#         original_class_name = str(getattr(pose, "class_name", "")).strip()
#         skip_precision_z = original_class_name in self.home_x_search_skip_z_classes
#         if skip_precision_z:
#             self.home_x_search_skip_z_classes.discard(original_class_name)
#             self.get_logger().info(
#                 "[PRECISION] HOME X SEARCH에서 이미 재촬영 높이로 인식한 pose라 "
#                 "정밀 재촬영 이동 시 Z 이동만 생략합니다."
#             )

#         target_x, target_y, z_approach, target_yaw = self.calc_approach_from_pose(
#             pose, layer_index=layer_index, yaw_offset=yaw_offset, offset_studs_x=offset_studs_x, offset_studs_y=offset_studs_y
#         )

#         used_refined_pose = False

#         if enable_precision_scan and original_class_name:
#             apply_y_offset = scan_z_mm is None
#             scan_x, scan_y = self.calc_precision_scan_xy(pose, apply_y_offset=apply_y_offset)
#             scan_z = 0.0 if skip_precision_z else (self.PRE_XY_LOWER if scan_z_mm is None else float(scan_z_mm))
#             use_delta_scan = scan_z_mm is not None and self.last_home_pose_xy is not None

#             if skip_precision_z:
#                 req_x = scan_x
#                 req_y = scan_y
#                 target_size = "XY"
#                 self.get_logger().info(
#                     f"[PRECISION] HOME X SEARCH pose라 Z 이동 없이 XY 재촬영 이동: "
#                     f"x={req_x:.4f}m, y={req_y:.4f}m"
#                 )
#             elif use_delta_scan:
#                 ref_x, ref_y = self.last_home_pose_xy
#                 raw_delta_x = pose.x - ref_x
#                 raw_delta_y = pose.y - ref_y
#                 req_x = raw_delta_x + self.PRECISION_SCAN_DELTA_X_OFFSET_M
#                 req_y = raw_delta_y + self.PRECISION_SCAN_DELTA_Y_OFFSET_M
#                 target_size = "APPROACH_DELTA"
#                 self.get_logger().info(
#                     f"[PRECISION][DELTA] 재촬영 위치 이동: "
#                     f"ref_home=({ref_x:.4f}, {ref_y:.4f})m, "
#                     f"target_home=({pose.x:.4f}, {pose.y:.4f})m, "
#                     f"raw_delta=({raw_delta_x:.4f}, {raw_delta_y:.4f})m, "
#                     f"offset=({self.PRECISION_SCAN_DELTA_X_OFFSET_M:.4f}, {self.PRECISION_SCAN_DELTA_Y_OFFSET_M:.4f})m, "
#                     f"delta=({req_x:.4f}, {req_y:.4f})m, "
#                     f"z={scan_z:.1f}mm, target_size={target_size}"
#                 )
#             else:
#                 req_x = scan_x
#                 req_y = scan_y
#                 target_size = "APPROACH"
#                 self.get_logger().info(
#                     f"[PRECISION] 재촬영 위치 APPROACH 다이렉트 이동: "
#                     f"x={scan_x:.4f}m, y={scan_y:.4f}m, "
#                     f"z={scan_z:.1f}mm, target_size={target_size}"
#                 )

#             scan_request = {
#                 "x": req_x,
#                 "y": req_y,
#                 "z": scan_z,
#                 "target_size": target_size,
#                 "skip_z": skip_precision_z,
#             }
#             self.precision_scan_requests[original_class_name] = scan_request
#             self.last_precision_scan_request = scan_request

#             self.call(
#                 self.cli_r,
#                 GetTargetPose.Request(x=req_x, y=req_y, z=scan_z, yaw=0.0, target_size=target_size),
#             )
#             time.sleep(self.WAIT_TIME)
#             self.last_home_pose_xy = (pose.x, pose.y)

#             refined_pose = self.find_target_with_retry(original_class_name, enable_home_x_search=False)

#             if refined_pose:
#                 target_x, target_y, z_approach, target_yaw = self.calc_approach_from_pose(
#                     refined_pose, layer_index=layer_index, yaw_offset=yaw_offset, offset_studs_x=offset_studs_x, offset_studs_y=offset_studs_y
#                 )
#                 used_refined_pose = True

#         self.call(
#             self.cli_r,
#             GetTargetPose.Request(x=target_x, y=target_y, z=z_approach, yaw=target_yaw, target_size="APPROACH"),
#         )
#         time.sleep(self.WAIT_TIME)

#         self.call(self.cli_r, GetTargetPose.Request(z=self.Z_MARGIN, target_size="Z"))
#         time.sleep(self.WAIT_TIME)

#         return True

#     def verify_insert_from_saved_scan(self, target_color, home_after_success=False):
#         target_color = str(target_color).strip()
#         scan_request = self.precision_scan_requests.get(target_color)
#         if scan_request is None:
#             scan_request = self.last_precision_scan_request

#         if scan_request is None:
#             self.get_logger().warn(
#                 f"[VERIFY] {target_color} 저장된 재촬영 위치가 없어 검증을 생략합니다."
#             )
#             if home_after_success:
#                 self.go_home(wait_sec=self.WAIT_TIME)
#             return True

#         self.get_logger().info(
#             f"[VERIFY] 저장된 재촬영 위치에서 {target_color} 확인: "
#             f"x={scan_request['x']:.4f}m, y={scan_request['y']:.4f}m, "
#             f"z={scan_request['z']:.1f}mm, target_size={scan_request['target_size']}"
#         )

#         if self.VERIFY_SCAN_VIA_HOME:
#             self.get_logger().info(
#                 "[VERIFY] gripper 닫은 상태로 HOME 경유 후 저장 재촬영 위치로 이동합니다."
#             )
#             self.go_home(wait_sec=self.WAIT_TIME)
#             if scan_request.get("skip_z"):
#                 self.call(
#                     self.cli_r,
#                     GetTargetPose.Request(z=self.PRE_XY_LOWER, target_size="Z"),
#                 )
#                 time.sleep(self.WAIT_TIME)
#         else:
#             self.get_logger().info(
#                 "[VERIFY] HOME 경유 없이 현재 위치에서 저장된 재촬영 위치로 XY만 이동합니다."
#             )
#             scan_request = {
#                 "x": scan_request["x"],
#                 "y": scan_request["y"],
#                 "z": 0.0,
#                 "target_size": "XY",
#                 "skip_z": False,
#             }

#         self.call(
#             self.cli_r,
#             GetTargetPose.Request(
#                 x=scan_request["x"],
#                 y=scan_request["y"],
#                 z=scan_request["z"],
#                 yaw=0.0,
#                 target_size=scan_request["target_size"],
#             ),
#         )
#         time.sleep(self.WAIT_TIME)

#         p = self.request_target_pose(target_color)
#         if p is not None and p.success:
#             self.last_verify_visible_poses[target_color] = p
#             self.get_logger().warn(
#                 f"[VERIFY] {target_color}가 아직 보입니다. 조립 실패로 판단하고 재시도합니다."
#             )
#             return False

#         self.last_verify_visible_poses.pop(target_color, None)
#         self.get_logger().info(
#             f"[VERIFY] {target_color}가 보이지 않습니다. 조립 성공으로 판단합니다."
#         )
#         if home_after_success:
#             self.get_logger().info("[VERIFY] 최종 검증 성공 후 HOME 이동")
#             self.go_home(wait_sec=self.WAIT_TIME)
#         return True

#     def recover_after_failed_insert(self, held_before_insert, release_gripper):
#         if held_before_insert:
#             self.held_class = held_before_insert
#             self.get_logger().warn(
#                 f"[VERIFY] 조립 실패. 그리퍼를 열지 않았으므로 {held_before_insert} 유지 상태로 재시도합니다."
#             )
#         else:
#             self.get_logger().warn(
#                 "[VERIFY] 조립 실패. held_class 정보는 없지만 그리퍼 유지 상태로 재시도합니다."
#             )
#         self.go_home(wait_sec=self.WAIT_TIME)
#         return True

#     def pick_target(self, color, layer_index=0, offset_studs_x=0.0, offset_studs_y=0.0, pick_pose=None, scan_z_mm=None):
#         self.get_logger().info(f"--- PICK TARGET: [{color.upper()}] ---")

#         self.get_logger().info("[GRIPPER] ensure open before pick")
#         self.call(self.cli_g, SetBool.Request(data=False))
#         time.sleep(self.WAIT_TIME)
#         self.held_class = None

#         # pick_pose가 주어지면 스캔 생략하고 바로 해당 글로벌 좌표 사용
#         p = pick_pose if pick_pose else self.find_target_with_retry(color)
#         if not p: return False

#         if not self.move_fast_from_pose(
#             p, layer_index=layer_index, yaw_offset=self.WRIST_OFFSET, offset_studs_x=offset_studs_x, offset_studs_y=offset_studs_y, scan_z_mm=scan_z_mm
#         ):
#             return False

#         self.call(self.cli_g, SetBool.Request(data=True))
#         time.sleep(self.WAIT_TIME)
#         self.held_class = str(color).strip()

#         self.call(self.cli_r, GetTargetPose.Request(z=-50.0, target_size="Z"))
#         time.sleep(self.WAIT_TIME)

#         return True

#     def visual_insert(
#         self, target_color, layer_index, release_gripper=True, yaw_offset=0.0, offset_studs_x=0.0, offset_studs_y=0.0, base_pose=None, scan_z_mm=None
#     ):
#         target_color = str(target_color).strip()
#         self.get_logger().info(f"--- VISUAL STACK: [{target_color.upper()}] (Layer +{layer_index}) ---")
#         time.sleep(1.0)

#         if scan_z_mm is None:
#             scan_z_mm = self.PRE_XY_LOWER_Z2

#         attempt = 1
#         retry_pose = None
#         while rclpy.ok():
#             self.get_logger().info(
#                 f"[INSERT ATTEMPT] {target_color} visual_insert {attempt}회차"
#             )

#             use_verify_pose = retry_pose is not None
#             if use_verify_pose:
#                 p = retry_pose
#                 retry_pose = None
#                 self.get_logger().info(
#                     "[RETRY WITHOUT HOME] 검증 재촬영 pose로 바로 재조립합니다."
#                 )
#             else:
#                 p = base_pose if base_pose else self.find_target_with_retry(target_color)
#                 if not p: return False

#             self.last_perfect_pose = p
#             held_before_insert = self.held_class

#             if not self.move_fast_from_pose(
#                 p,
#                 layer_index=layer_index,
#                 yaw_offset=yaw_offset + self.WRIST_OFFSET,
#                 offset_studs_x=offset_studs_x,
#                 offset_studs_y=offset_studs_y,
#                 enable_precision_scan=not use_verify_pose,
#                 scan_z_mm=scan_z_mm,
#             ):
#                 return False

#             if release_gripper:
#                 self.get_logger().info(
#                     "[GRIPPER] release_gripper=True이지만 검증 전이므로 open하지 않습니다."
#                 )
#             else:
#                 self.get_logger().info("[GRIPPER] release_gripper=False, 그리퍼 유지")

#             self.get_logger().info(
#                 "[VERIFY] 조립 직후 마진 상승 없이 gripper 유지 상태로 검증 이동을 시작합니다."
#             )

#             if self.verify_insert_from_saved_scan(target_color, home_after_success=False):
#                 if release_gripper:
#                     self.held_class = None
#                 else:
#                     self.held_class = held_before_insert
#                 return True

#             retry_pose = self.last_verify_visible_poses.pop(target_color, None)
#             if retry_pose is not None:
#                 self.held_class = held_before_insert
#                 attempt += 1
#                 continue

#             if not self.recover_after_failed_insert(held_before_insert, release_gripper):
#                 return False
#             attempt += 1

#         return False

#     def blind_insert(
#         self, base_pose, layer_index, yaw_offset=0.0, release_gripper=True, offset_studs_x=0.0, offset_studs_y=0.0, scan_z_mm=None
#     ):
#         self.get_logger().info(f"--- BLIND STACK / MEMORY POSE: Layer {layer_index} ---")
#         time.sleep(1.0)
#         if scan_z_mm is None:
#             scan_z_mm = self.PRE_XY_LOWER_Z2

#         target_color = str(getattr(base_pose, "class_name", "")).strip()
#         attempt = 1
#         retry_pose = None

#         while rclpy.ok():
#             self.get_logger().info(
#                 f"[INSERT ATTEMPT] blind_insert {attempt}회차 "
#                 f"(target={target_color or 'unknown'})"
#             )

#             held_before_insert = self.held_class
#             use_verify_pose = retry_pose is not None
#             current_base_pose = retry_pose if use_verify_pose else base_pose
#             retry_pose = None

#             if not self.move_fast_from_pose(
#                 current_base_pose,
#                 layer_index=layer_index,
#                 yaw_offset=yaw_offset + self.WRIST_OFFSET,
#                 offset_studs_x=offset_studs_x,
#                 offset_studs_y=offset_studs_y,
#                 enable_precision_scan=not use_verify_pose,
#                 scan_z_mm=scan_z_mm,
#             ):
#                 return False

#             if release_gripper:
#                 self.get_logger().info(
#                     "[GRIPPER] release_gripper=True이지만 검증 전이므로 open하지 않습니다."
#                 )
#             else:
#                 self.get_logger().info("[GRIPPER] release_gripper=False, 그리퍼 유지")

#             if not target_color:
#                 self.get_logger().warn(
#                     "[VERIFY] base_pose.class_name이 없어 blind_insert 검증을 생략합니다."
#                 )
#                 if release_gripper:
#                     self.held_class = None
#                 else:
#                     self.held_class = held_before_insert
#                 return True

#             if self.verify_insert_from_saved_scan(target_color, home_after_success=False):
#                 if release_gripper:
#                     self.held_class = None
#                 else:
#                     self.held_class = held_before_insert
#                 return True

#             retry_pose = self.last_verify_visible_poses.pop(target_color, None)
#             if retry_pose is not None:
#                 self.held_class = held_before_insert
#                 attempt += 1
#                 continue

#             if not self.recover_after_failed_insert(held_before_insert, release_gripper):
#                 return False
#             attempt += 1

#         return False


#     # ------------------------------------------------------------------ #
#     # 조립 시퀀스
#     # ------------------------------------------------------------------ #

#     def build_battery(self):
#         self.get_logger().info("[배터리] 노란색(Pick) -> 파란색(Base)")
#         base_pose = self.find_target_with_retry("2x2_blue")
#         pick_pose = self.find_target_with_retry("2x2_yellow")
#         if not (base_pose and pick_pose): return

#         if not self.pick_target("2x2_yellow", pick_pose=pick_pose): return
#         if self.visual_insert("2x2_blue", layer_index=0.7, base_pose=base_pose):
#             self.get_logger().info("[완료] 배터리")

#     def build_magnet(self):
#         self.get_logger().info("[자석] 파란색(Pick) -> 빨간색(Base)")
#         base_pose = self.find_target_with_retry("2x2_red")
#         pick_pose = self.find_target_with_retry("2x2_blue")
#         if not (base_pose and pick_pose): return

#         if not self.pick_target("2x2_blue", pick_pose=pick_pose): return
#         if self.visual_insert("2x2_red", layer_index=0.7, base_pose=base_pose):
#             self.get_logger().info("[완료] 자석")

#     def build_e_stop(self):
#         self.get_logger().info("[비상정지] 빨간색(Pick) -> 노란색4x2(Base)")
#         base_pose = self.find_target_with_retry("4x2_yellow")
#         pick_pose = self.find_target_with_retry("2x2_red")
#         if not (base_pose and pick_pose): return

#         if not self.pick_target("2x2_red", pick_pose=pick_pose): return
#         if self.visual_insert("4x2_yellow", layer_index=0.7, yaw_offset=0.0, base_pose=base_pose):
#             self.get_logger().info("[완료] 비상정지")

#     def build_carrot(self):
#         self.get_logger().info("[당근] 초록(Pick) -> 노랑(그리퍼 유지) -> 노랑(Base)")
#         base_pose_1 = self.find_target_with_retry("2x2_yellow")
#         pick_pose = self.find_target_with_retry("2x2_green")
#         if not (base_pose_1 and pick_pose): return

#         if not self.pick_target("2x2_green", pick_pose=pick_pose): return
#         if not self.visual_insert("2x2_yellow", layer_index=0.7, release_gripper=False, base_pose=base_pose_1): return
#         if self.visual_insert("2x2_yellow", layer_index=1.7, base_pose=base_pose_1):
#             self.get_logger().info("[완료] 당근")

#     def build_traffic_light(self):
#         self.get_logger().info("[신호등] 빨강(Pick) -> 노랑(그리퍼 유지) -> 초록(Base)")
#         base_yellow = self.find_target_with_retry("2x2_yellow")
#         base_green = self.find_target_with_retry("2x2_green")
#         pick_red = self.find_target_with_retry("2x2_red")
#         if not (base_yellow and base_green and pick_red): return

#         if not self.pick_target("2x2_red", pick_pose=pick_red): return
#         if not self.visual_insert("2x2_yellow", layer_index=0.7, release_gripper=False, base_pose=base_yellow): return
#         if self.visual_insert("2x2_green", layer_index=1.7, base_pose=base_green):
#             self.get_logger().info("[완료] 신호등")

#     def build_small_tree(self):
#         self.get_logger().info("[작은 나무] 초록2x2(Pick) -> 초록4x2(그리퍼 유지) -> 노랑2x2(Base)")
#         base_green4x2 = self.find_target_with_retry("4x2_green")
#         base_yellow2x2 = self.find_target_with_retry("2x2_yellow")
#         pick_green2x2 = self.find_target_with_retry("2x2_green")
#         if not (base_green4x2 and base_yellow2x2 and pick_green2x2): return

#         if not self.pick_target("2x2_green", pick_pose=pick_green2x2): return
#         if not self.visual_insert("4x2_green", layer_index=0.7, release_gripper=False, base_pose=base_green4x2): return
#         if self.visual_insert("2x2_yellow", layer_index=1.5, base_pose=base_yellow2x2):
#             self.get_logger().info("[완료] 작은 나무")

#     def build_hammer(self):
#         self.get_logger().info("[망치] 파랑4x2(Pick) -> 빨강2x2(그리퍼 유지) -> 빨강2x2(Base)")
#         base_red2x2 = self.find_target_with_retry("2x2_red")
#         pick_blue4x2 = self.find_target_with_retry("4x2_blue")
#         if not (base_red2x2 and pick_blue4x2): return

#         if not self.pick_target("4x2_blue", pick_pose=pick_blue4x2): return
#         if not self.visual_insert("2x2_red", layer_index=0.7, release_gripper=False, base_pose=base_red2x2): return
#         if self.visual_insert("2x2_red", layer_index=1.5, base_pose=base_red2x2):
#             self.get_logger().info("[완료] 망치")

#     def build_big_carrot(self):
#         self.get_logger().info("[큰 당근] 초록2x2(Pick) -> 노랑4x2 -> 노랑2x2 -> 노랑2x2(Base)")
#         base_yellow4x2 = self.find_target_with_retry("4x2_yellow")
#         base_yellow2x2 = self.find_target_with_retry("2x2_yellow")
#         pick_green2x2 = self.find_target_with_retry("2x2_green")
#         if not (base_yellow4x2 and base_yellow2x2 and pick_green2x2): return

#         if not self.pick_target("2x2_green", pick_pose=pick_green2x2): return
#         if not self.visual_insert("4x2_yellow", layer_index=0.7, release_gripper=False, base_pose=base_yellow4x2): return
#         if not self.visual_insert("2x2_yellow", layer_index=1.5, release_gripper=False, base_pose=base_yellow2x2): return
#         if self.visual_insert("2x2_yellow", layer_index=2.5, release_gripper=False, base_pose=base_yellow2x2):
#             self.get_logger().info("[완료] 큰 당근")

#     def build_burger(self):
#         self.get_logger().info("[버거] 4x2_yellow(Pick) -> 4x2_red 결합 -> 2x2_red로 6x2 조립 -> 4x2_yellow 최종 결합")

#         pick_yellow_pose = self.find_target_with_retry("4x2_yellow")
#         base_4x2_red = self.find_target_with_retry("4x2_red")
#         base_2x2_red = self.find_target_with_retry("2x2_red")
#         if not (pick_yellow_pose and base_4x2_red and base_2x2_red): return

#         if not self.pick_target("4x2_yellow", pick_pose=pick_yellow_pose): return

#         self.get_logger().info("[Phase 1-2] HOME 복귀 없이 남은 4x2_yellow 위치 저장")
#         saved_bottom_yellow_pose = self.find_target_with_retry("4x2_yellow")
#         if not saved_bottom_yellow_pose:
#             self.get_logger().warn(
#                 "남은 4x2_yellow 위치 저장 실패. Phase 4 메모리 fallback 없이 진행합니다."
#             )

#         if not self.visual_insert("4x2_red", layer_index=0.7, offset_studs_y=-1.0, release_gripper=False, base_pose=base_4x2_red): return
#         if not self.visual_insert("2x2_red", layer_index=0.7, offset_studs_y=2.05, release_gripper=False, base_pose=base_2x2_red): return

#         if self.visual_insert("4x2_yellow", layer_index=1.4, base_pose=saved_bottom_yellow_pose):
#             self.get_logger().info("[완료] 버거")
#             return

#         if saved_bottom_yellow_pose and self.blind_insert(saved_bottom_yellow_pose, layer_index=1.5):
#             self.get_logger().info("[완료] 버거 (저장 위치 fallback)")

#     def build_ice_cream(self):
#         self.get_logger().info("[아이스크림] 2x2_red -> 4x2_yellow(y+1.0), 2x2_blue -> 4x2_yellow(y-1.0), 2x2_green -> 2x2_blue(y+1.0) 후 2x2_yellow 최종 결합")

#         saved_bottom_yellow_pose = self.find_target_with_retry("2x2_yellow")
#         base_4x2_yellow = self.find_target_with_retry("4x2_yellow")
#         pick_red = self.find_target_with_retry("2x2_red")
#         pick_blue = self.find_target_with_retry("2x2_blue")
#         pick_green = self.find_target_with_retry("2x2_green")
#         if not (saved_bottom_yellow_pose and base_4x2_yellow and pick_red and pick_blue and pick_green): return

#         if not self.pick_target("2x2_red", pick_pose=pick_red): return
#         if not self.visual_insert("4x2_yellow", layer_index=0.7, offset_studs_y=1.0, base_pose=base_4x2_yellow): return

#         if not self.pick_target("2x2_blue", pick_pose=pick_blue, scan_z_mm=self.PRE_XY_LOWER_Z2): return
#         if not self.visual_insert("2x2_red", layer_index=0.5, offset_studs_y=-2.0): return

#         self.get_logger().info("[Phase 4-2] HOME 복귀 없이 방금 조립한 2x2_blue 위치 재촬영")
#         base_2x2_blue = self.find_target_with_retry("2x2_blue")
#         if not base_2x2_blue: return

#         if not self.pick_target("2x2_green", pick_pose=pick_green, scan_z_mm=self.PRE_XY_LOWER_Z2): return
#         if not self.visual_insert("2x2_blue", layer_index=1.7, offset_studs_y=1.0, release_gripper=False, base_pose=base_2x2_blue): return

#         if self.visual_insert("2x2_yellow", layer_index=2.5, release_gripper=True, base_pose=saved_bottom_yellow_pose):
#             self.get_logger().info("[완료] 아이스크림")
#             return

#         if saved_bottom_yellow_pose and self.blind_insert(saved_bottom_yellow_pose, layer_index=2.5, release_gripper=True):
#             self.get_logger().info("[완료] 아이스크림 (저장 위치 fallback)")

#     def build_big_tree(self):
#         self.get_logger().info("[큰 나무] 버거형 시퀀스")

#         saved_yellow_pose = self.find_target_with_retry("2x2_yellow")
#         base_4x2_green_1 = self.find_target_with_retry("4x2_green")
#         pick_2x2_green = self.find_target_with_retry("2x2_green")
#         if not (saved_yellow_pose and base_4x2_green_1 and pick_2x2_green): return

#         if not self.pick_target("2x2_green", pick_pose=pick_2x2_green): return

#         if not self.visual_insert("4x2_green", layer_index=0.7, offset_studs_y=0.0, release_gripper=False, base_pose=base_4x2_green_1): return

#         self.get_logger().info("[Phase 2-2] HOME 복귀 없이 다음 4x2_green / 2x2_green 위치 재촬영")
#         base_4x2_green_2 = self.find_target_with_retry("4x2_green")
#         base_2x2_green_2 = self.find_target_with_retry("2x2_green")
#         if not (base_4x2_green_2 and base_2x2_green_2): return

#         if not self.visual_insert("4x2_green", layer_index=1.7, offset_studs_y=-1.0, release_gripper=False, base_pose=base_4x2_green_2): return
#         if not self.visual_insert("2x2_green", layer_index=1.7, offset_studs_y=2.0, release_gripper=False, base_pose=base_2x2_green_2): return

#         if self.visual_insert("2x2_yellow", layer_index=2.4, offset_studs_y=0.0, release_gripper=True, base_pose=saved_yellow_pose):
#             self.get_logger().info("[완료] 큰 나무")
#             return

#         if saved_yellow_pose and self.blind_insert(saved_yellow_pose, layer_index=2.4, offset_studs_y=0.0, release_gripper=True):
#             self.get_logger().info("[완료] 큰 나무 (저장 위치 fallback)")

#     # ------------------------------------------------------------------ #
#     # 메인 루프
#     # ------------------------------------------------------------------ #

#     def run(self):
#         self.get_logger().info("STARTING ASSEMBLY SEQUENCE (Keyboard Select Mode)")

#         home_res = self.go_home(wait_sec=0.0)
#         if home_res is None or not home_res.success:
#             self.get_logger().error("시작 HOME 이동 실패. 조립을 시작하지 않습니다.")
#             return

#         self.get_logger().info("[INIT] gripper open")
#         self.call(self.cli_g, SetBool.Request(data=False))
#         time.sleep(1.0)

#         actions = {
#             "1": self.build_battery, "battery": self.build_battery, "배터리": self.build_battery,
#             "2": self.build_magnet, "magnet": self.build_magnet, "자석": self.build_magnet,
#             "3": self.build_e_stop, "estop": self.build_e_stop, "비상정지": self.build_e_stop,
#             "4": self.build_carrot, "carrot": self.build_carrot, "당근": self.build_carrot,
#             "5": self.build_traffic_light, "traffic": self.build_traffic_light, "신호등": self.build_traffic_light,
#             "6": self.build_small_tree, "tree": self.build_small_tree, "작은나무": self.build_small_tree,
#             "7": self.build_hammer, "hammer": self.build_hammer, "망치": self.build_hammer,
#             "8": self.build_big_carrot, "bigcarrot": self.build_big_carrot, "큰당근": self.build_big_carrot,
#             "9": self.build_burger, "burger": self.build_burger, "버거": self.build_burger,
#             "10": self.build_ice_cream, "icecream": self.build_ice_cream, "아이스크림": self.build_ice_cream,
#             "11": self.build_big_tree, "big_tree": self.build_big_tree, "큰나무": self.build_big_tree,
#         }

#         print("\n=== Master Node Assembly Keyboard Select ===")
#         print("1: 배터리 / 2: 자석 / 3: 비상정지 / 4: 당근 / 5: 신호등 / 6: 작은나무")
#         print("7: 망치 / 8: 큰당근 / 9: 버거 / 10: 아이스크림 / 11: 큰나무")
#         print("q: 종료")

#         while rclpy.ok():
#             user_input = input("\n조립할 항목을 선택하세요 [1~11/q]: ").strip().replace(" ", "").lower()

#             if user_input in ("q", "quit", "exit", "종료"):
#                 self.get_logger().info("조립 시퀀스를 종료합니다.")
#                 break

#             action = actions.get(user_input)
#             if action is None:
#                 print("잘못된 입력입니다. 1~11 또는 q 중에서 선택하세요.")
#                 continue

#             self.get_logger().info(f"작업 시작: {user_input}")
#             action()

#             self.get_logger().info("[POST] 개별 작업 후 HOME 이동")
#             self.go_home(wait_sec=1.0)
#             self.get_logger().info("개별 조립 완료")

#         self.get_logger().info("[END] HOME 이동")
#         self.go_home(wait_sec=1.0)
#         self.get_logger().info("ALL SEQUENCE DONE")


# def main():
#     rclpy.init()
#     node = MasterNode()
#     try:
#         node.run()
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == "__main__":
#     main()
