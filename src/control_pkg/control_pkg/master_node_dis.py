import time
import rclpy
from rclpy.node import Node
from srvs_pkg.srv import GetTargetPose
from std_srvs.srv import SetBool, Trigger

# ============================================================
# 조정 가능 파라미터 (ROS 파라미터로도 오버라이드 가능)
# ============================================================
Z_OFF_DEFAULT         = -95.0   # 비전 Z → 엔드이펙터 Z 변환 오프셋 (mm)
Z_MARGIN_DEFAULT      =  20.0   # 블록 접근 전 안전 여유 거리 (mm)
BLOCK_H_DEFAULT       =  16.0   # 레고 블록 한 층 높이 실측값 (mm)
LAYER_IDX_OFFSET      =   1.5   # 층 인덱스 보정 오프셋 — assembly 실측값 기준 (1층→0.6, 2층→1.6, 3층→2.6)
WAIT_TIME_DEFAULT     =   0.7   # 동작 간 일반 대기 시간 (s)
GRIP_WAIT_DEFAULT     =   1.3   # 그리퍼 동작 후 안정화 대기 (s)
INITIAL_LIFT_DEFAULT  = -20.0   # 그립 직후 초기 상승 거리 (mm, 음수=위로)
PULL_UP_DEFAULT       = -30.0   # 강제 분리 추가 상승 거리 (mm, 음수=위로)
WRIST_OFFSET_DEFAULT  =  0.0   # robot1 픽업 시 손목 추가 회전 각도 (deg)# 비전쪽에서 장축으로 넘겨줌
BURGER_Y_MIN_DEFAULT  =   0.0   # 버거 4x2 빨강 Y필터 하한 (m) — assembly Y > DROP Y, 실측 후 조정
# ============================================================


class BatteryDualDisassembly(Node):
    def __init__(self, node_name="master_node_dis"):
        super().__init__(node_name)

        # 서비스 클라이언트
        self.cli_v1 = self.create_client(GetTargetPose, "/get_target_pose")
        self.cli_r1 = self.create_client(GetTargetPose, "/robot1/robot_move_step")
        self.cli_r2 = self.create_client(GetTargetPose, "/robot2/robot_move_step")
        self.cli_h1 = self.create_client(Trigger, "/robot1/robot_home")
        self.cli_h2 = self.create_client(Trigger, "/robot2/robot_home")

        r1_grip_srv = self.declare_parameter("robot1_gripper_service", "/control_gripper").value
        r2_grip_srv = self.declare_parameter("robot2_gripper_service", "/robot2/control_gripper").value
        self.cli_g1 = self.create_client(SetBool, r1_grip_srv)
        self.cli_g2 = self.create_client(SetBool, r2_grip_srv)

        # 파라미터 로드 (기본값은 위 상수 참조)
        self.Z_OFF          = float(self.declare_parameter("robot1_z_off",          Z_OFF_DEFAULT).value)
        self.Z_MARGIN       = float(self.declare_parameter("robot1_z_margin",       Z_MARGIN_DEFAULT).value)
        self.BLOCK_H        = float(self.declare_parameter("block_h",               BLOCK_H_DEFAULT).value)
        self.LAYER_IDX_OFF  = float(self.declare_parameter("layer_idx_offset",      LAYER_IDX_OFFSET).value)
        self.WAIT_TIME      = float(self.declare_parameter("wait_time",              WAIT_TIME_DEFAULT).value)
        self.GRIP_WAIT      = float(self.declare_parameter("grip_wait_time",         GRIP_WAIT_DEFAULT).value)
        self.INITIAL_LIFT   = float(self.declare_parameter("robot1_initial_lift_mm", INITIAL_LIFT_DEFAULT).value)
        self.PULL_UP        = float(self.declare_parameter("robot1_pull_up_mm",      PULL_UP_DEFAULT).value)
        self.WRIST_OFFSET   = float(self.declare_parameter("wrist_offset_deg",       WRIST_OFFSET_DEFAULT).value)
        self.BURGER_Y_MIN   = float(self.declare_parameter("burger_y_min_m",         BURGER_Y_MIN_DEFAULT).value)

        # 비전 클래스명 → 타겟 ID 매핑
        self.class_to_target_id = {
            "2x2_red": "1",  "2x2_green": "2",  "2x2_blue": "3",  "2x2_yellow": "4",
            "4x2_red": "5",  "4x2_green": "6",  "4x2_blue": "7",  "4x2_yellow": "8",
            "assembly": "999", "Magnet": "13", "Battery": "34", "estop": "81",
            "traffic light": "241", "carrot": "442", "small tree": "462", "hammer": "711",
            "big carrot": "4482", "burger": "8518", "bigtree": "46262", "icecream": "48132",
        }

        self.get_logger().info(f"Disassembly node ready. g1={r1_grip_srv}, g2={r2_grip_srv}")

    # ── 저수준 헬퍼 ──────────────────────────────────────────

    def call(self, cli, req):
        while not cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f"Waiting for {cli.srv_name}...")
        future = cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def sleep(self):
        time.sleep(self.WAIT_TIME)

    def set_gripper(self, cli, closed: bool) -> bool:
        res = self.call(cli, SetBool.Request(data=closed))
        time.sleep(self.GRIP_WAIT)
        return res.success

    def move_z(self, cli, dz_mm: float) -> bool:
        req = GetTargetPose.Request()
        req.target_size = "Z"
        req.z = dz_mm
        return self.call(cli, req).success

    def send_pose(self, cli, pose_name: str) -> bool:
        """이름으로 사전 정의된 조인트 자세로 이동"""
        req = GetTargetPose.Request()
        req.target_size = pose_name
        return self.call(cli, req).success

    # ── 비전 ─────────────────────────────────────────────────

    def _normalize_target(self, target: str) -> str:
        aliases = {
            "2x4_red": "4x2_red", "2x4_green": "4x2_green",
            "2x4_blue": "4x2_blue", "2x4_yellow": "4x2_yellow",
        }
        return aliases.get(str(target), str(target))

    def request_vision_pose(self, target: str):
        name = self._normalize_target(target)
        tid  = self.class_to_target_id.get(name, name)
        req  = GetTargetPose.Request()
        req.target_color = tid

        self.get_logger().info(f"비전 요청: {name} (id={tid})")
        p = self.call(self.cli_v1, req)

        if p is None:
            self.get_logger().error(f"비전 응답 없음: {name}")
            return None
        if not p.success:
            self.get_logger().warn(f"비전 탐색 실패: {name}")
            return None

        self.get_logger().info(
            f"비전 응답: {name}, x={p.x*1000:.1f} y={p.y*1000:.1f} "
            f"z={p.z*1000:.1f}mm yaw={p.yaw:.1f}deg"
        )
        return p

    def _target_fallbacks(self, target: str) -> list:
        """대체 타겟 목록 반환 (2x2↔4x2 상호 대체)"""
        if target.startswith("2x2_"):
            return [target, target.replace("2x2_", "4x2_", 1)]
        if target.startswith("4x2_"):
            return [target, target.replace("4x2_", "2x2_", 1)]
        if target.startswith("2x4_"):
            return [target, target.replace("2x4_", "2x2_", 1)]
        return [target]

    def find_target(self, target: str, retries: int = 3,
                    y_min_m: float = None, prefer_max_y: bool = False):
        """타겟(+대체 타겟) 탐색. 성공 시 (pose, 실제_타겟명) 반환.
        prefer_max_y=True 이면 모든 후보를 탐색해 Y값이 가장 큰 것을 반환."""
        candidates = self._target_fallbacks(target)
        for _ in range(retries):
            if prefer_max_y:
                best_p, best_candidate = None, None
                for candidate in candidates:
                    p = self.request_vision_pose(candidate)
                    if p and (best_p is None or p.y > best_p.y):
                        best_p, best_candidate = p, candidate
                if best_p is not None:
                    if best_candidate != target:
                        self.get_logger().warn(
                            f"{target} → 대체 {best_candidate}로 진행 (Y최대 선택: {best_p.y*1000:.1f}mm)"
                        )
                    return best_p, best_candidate
            else:
                for candidate in candidates:
                    p = self.request_vision_pose(candidate)
                    if p:
                        if y_min_m is not None and p.y < y_min_m:
                            self.get_logger().warn(
                                f"[Y필터] {candidate} y={p.y*1000:.1f}mm < {y_min_m*1000:.1f}mm → 스킵"
                            )
                            time.sleep(0.3)
                            continue
                        if candidate != target:
                            self.get_logger().warn(f"{target} → 대체 {candidate}로 진행")
                        return p, candidate
                    time.sleep(0.3)
        return None, None

    # ── Yaw 정규화 ───────────────────────────────────────────

    def _pick_wrist_yaw(self, yaw: float) -> float:
        while yaw > 0.0:    yaw -= 180.0
        while yaw < -180.0: yaw += 180.0
        return yaw

    # ── 홈 / 종료 자세 ───────────────────────────────────────

    def move_both_home_pose(self):
        self.get_logger().info("양쪽 로봇 HOME")
        self.call(self.cli_h1, Trigger.Request())
        self.call(self.cli_h2, Trigger.Request())
        self.sleep()
        return True

    def move_both_end_pose(self):
        self.get_logger().info("양쪽 로봇 END")
        ok1 = self.send_pose(self.cli_r1, "END")
        ok2 = self.send_pose(self.cli_r2, "END")
        self.sleep()
        return ok1 and ok2

    # ── 분해 원자 동작 ───────────────────────────────────────

    def robot1_top_pick(self, target: str, top_label: str,
                        expected_layer: int = None, yaw_offset: float = 0.0,
                        z_extra_mm: float = 0.0, y_min_m: float = None,
                        prefer_max_y: bool = False) -> bool:
        """
        비전 1회 스캔 → 원샷 APPROACH → 그립 → 초기 상승 → SEPARATION 자세.
        z_extra_mm: 자동 Z 계산이 맞지 않는 특수 케이스용 보정 (양수=더 깊이).
        y_min_m: Y 하한 필터 — 이 값보다 작은 Y의 블록은 스킵.
        prefer_max_y: True 이면 후보 중 Y가 가장 큰 블록 선택.
        """
        self.get_logger().info(
            f"[PICK] {top_label} (layer={expected_layer}, yaw_off={yaw_offset}, z_extra={z_extra_mm})"
        )
        p, _ = self.find_target(target, y_min_m=y_min_m, prefer_max_y=prefer_max_y)
        if not p:
            self.get_logger().error(f"[PICK] 비전 실패: {top_label}")
            return False

        target_yaw  = self._pick_wrist_yaw(p.yaw + yaw_offset + self.WRIST_OFFSET)
        layer_index = max(0.0, (expected_layer or 2) - self.LAYER_IDX_OFF)  # 1→0.6, 2→0.6, 3→1.6
        z_move      = (p.z * 1000.0 + self.Z_OFF) - (self.BLOCK_H * layer_index) + z_extra_mm
        z_approach  = z_move - self.Z_MARGIN

        req = GetTargetPose.Request()
        req.target_size = "APPROACH"
        req.x   = p.x
        req.y   = p.y
        req.z   = z_approach
        req.yaw = target_yaw
        self.call(self.cli_r1, req)
        self.sleep()

        self.move_z(self.cli_r1, self.Z_MARGIN)     # 최종 수직 하강
        self.sleep()
        self.set_gripper(self.cli_g1, True)
        self.move_z(self.cli_r1, self.INITIAL_LIFT)  # 초기 상승
        self.sleep()
        self.send_pose(self.cli_r1, "SEPARATION")
        self.sleep()
        return True

    def robot2_side_hold(self, bottom_label: str) -> bool:
        """robot2: SEPARATION 자세로 이동 후 하단 블록 그립"""
        self.get_logger().info(f"[HOLD] robot2 하단 고정: {bottom_label}")
        if not self.send_pose(self.cli_r2, "SEPARATION"):
            self.get_logger().error("robot2: SEPARATION 실패")
            return False
        self.sleep()
        self.set_gripper(self.cli_g2, True)
        return True

    def robot1_pull_up(self, top_label: str) -> bool:
        """robot1: 추가 상승으로 상단 블록 강제 분리"""
        self.get_logger().info(f"[PULL] 강제 분리: {top_label}")
        self.move_z(self.cli_r1, self.PULL_UP)
        self.sleep()
        return True

    def robot2_return_home_holding(self, bottom_label: str) -> bool:
        """robot2: 블록 잡은 상태로 홈 복귀"""
        self.get_logger().info(f"[HOME] robot2 블록 잡고 홈: {bottom_label}")
        self.call(self.cli_h2, Trigger.Request())
        self.sleep()
        return True

    def robot2_release_and_home(self, bottom_label: str) -> bool:
        """robot2: 제자리에서 그리퍼 열고 홈 복귀"""
        self.get_logger().info(f"[RELEASE] robot2: {bottom_label}")
        self.set_gripper(self.cli_g2, False)
        self.call(self.cli_h2, Trigger.Request())
        self.sleep()
        return True

    def robot1_place_top_at_center_and_home(self, top_label: str) -> bool:
        """robot1: CENTER 임시 배치 후 홈 복귀"""
        self.get_logger().info(f"[CENTER] robot1 임시 배치: {top_label}")
        if not self.send_pose(self.cli_r1, "CENTER"):
            self.get_logger().error("robot1: CENTER 실패")
            return False
        self.sleep()
        self.set_gripper(self.cli_g1, False)
        self.call(self.cli_h1, Trigger.Request())
        self.sleep()
        return True

    def robot1_drop_top_and_home(self, top_label: str, drop_slot: str = "DROP") -> bool:
        """robot1: 지정 슬롯에 블록 내려놓고 홈 복귀"""
        self.get_logger().info(f"[DROP] robot1 → {drop_slot}: {top_label}")
        if not self.send_pose(self.cli_r1, drop_slot):
            self.get_logger().error(f"robot1: {drop_slot} 실패")
            return False
        self.sleep()
        self.set_gripper(self.cli_g1, False)
        self.call(self.cli_h1, Trigger.Request())
        self.sleep()
        return True

    def robot2_drop_bottom_and_home(self, bottom_label: str, drop_slot: str = "DROP") -> bool:
        """robot2: 지정 슬롯에 블록 내려놓고 홈 복귀"""
        self.get_logger().info(f"[DROP] robot2 → {drop_slot}: {bottom_label}")
        if not self.send_pose(self.cli_r2, drop_slot):
            self.get_logger().error(f"robot2: {drop_slot} 실패")
            return False
        self.sleep()
        self.set_gripper(self.cli_g2, False)
        self.call(self.cli_h2, Trigger.Request())
        self.sleep()
        return True

    # ── 복합 시퀀스 ──────────────────────────────────────────

    def robot1_pick_layer_and_drop(
        self, target, label, expected_layer, drop_slot,
        bottom_label=None, yaw_offset=0.0, robot2_drop_slot=None, z_extra_mm=0.0,
        y_min_m=None, prefer_max_y=False,
    ) -> bool:
        """
        표준 단층 분해.
        bottom_label=None 이면 robot2 없이 robot1만 DROP.
        """
        if not self.robot1_top_pick(target, label, expected_layer, yaw_offset, z_extra_mm, y_min_m, prefer_max_y):
            self.get_logger().error(f"{expected_layer}층 {label} 분해 실패")
            return False

        if bottom_label is None:
            return self.robot1_drop_top_and_home(label, drop_slot)

        r2_slot = robot2_drop_slot or ("DROP2" if expected_layer in (2, 3) else "DROP")

        if not self.robot2_side_hold(bottom_label):
            return False
        if not self.robot1_pull_up(label):
            self.robot2_drop_bottom_and_home(bottom_label, r2_slot)
            return False
        if not self.robot2_return_home_holding(bottom_label):
            return False
        if not self.robot1_drop_top_and_home(label, drop_slot):
            self.robot2_drop_bottom_and_home(bottom_label, r2_slot)
            return False
        return self.robot2_drop_bottom_and_home(bottom_label, r2_slot)

    def robot1_pick_layer_place_center_robot2_drop(
        self, target, label, expected_layer, bottom_label, robot2_drop_slot,
        yaw_offset=0.0, z_extra_mm=0.0,
    ) -> bool:
        """robot1: CENTER 임시 배치, robot2: 하단 블록 DROP. (큰 나무 특수 스텝)"""
        if not self.robot1_top_pick(target, label, expected_layer, yaw_offset, z_extra_mm):
            return False
        if not self.robot2_side_hold(bottom_label):
            return False
        if not self.robot1_pull_up(label):
            self.robot2_drop_bottom_and_home(bottom_label, robot2_drop_slot)
            return False
        if not self.robot2_return_home_holding(bottom_label):
            return False
        if not self.robot1_place_top_at_center_and_home(label):
            self.robot2_drop_bottom_and_home(bottom_label, robot2_drop_slot)
            return False
        return self.robot2_drop_bottom_and_home(bottom_label, robot2_drop_slot)

    def robot1_pick_layer_drop_top_then_robot2_drop(
        self, target, label, expected_layer, drop_slot,
        bottom_label, robot2_drop_slot, yaw_offset=0.0, z_extra_mm=0.0,
    ) -> bool:
        """robot1 DROP 후 robot2도 지정 슬롯으로 DROP. (큰 나무 특수 스텝)"""
        if not self.robot1_top_pick(target, label, expected_layer, yaw_offset, z_extra_mm):
            return False
        if not self.robot2_side_hold(bottom_label):
            return False
        if not self.robot1_pull_up(label):
            self.robot2_drop_bottom_and_home(bottom_label, robot2_drop_slot)
            return False
        if not self.robot2_return_home_holding(bottom_label):
            return False
        if not self.robot1_drop_top_and_home(label, drop_slot):
            self.robot2_drop_bottom_and_home(bottom_label, robot2_drop_slot)
            return False
        return self.robot2_drop_bottom_and_home(bottom_label, robot2_drop_slot)

    # ── N층 공용 러너 ────────────────────────────────────────

    def _run_two_layer(self, name: str, layers: list) -> bool:
        """
        2층 전용 분해. robot2 drop_slot = "DROP" (기본).
        layers = [(2, target, label [, yaw_offset]), (1, target, label)]
        """
        top_layer, bot_layer = layers[0], layers[1]
        top_target = top_layer[1]
        top_label  = top_layer[2]
        yaw_offset = top_layer[3] if len(top_layer) > 3 else 0.0
        bot_label  = bot_layer[2]

        self.get_logger().info(f"{name} 2층 분해 시작: {top_label} / {bot_label}")
        self.move_both_home_pose()
        self.set_gripper(self.cli_g1, False)
        self.set_gripper(self.cli_g2, False)

        if not self.robot1_top_pick(top_target, top_label, expected_layer=2, yaw_offset=yaw_offset):
            return False
        if not self.robot2_side_hold(bot_label):
            return False
        if not self.robot1_pull_up(top_label):
            return False
        if not self.robot2_return_home_holding(bot_label):
            return False
        if not self.robot1_drop_top_and_home(top_label):
            return False
        if not self.robot2_drop_bottom_and_home(bot_label):
            return False

        self.get_logger().info(f"[완료] {name}")
        self.move_both_end_pose()
        return True

    def _run_layers(self, name: str, layers: list) -> bool:
        """
        3층 이상 범용 분해 러너. 위층부터 순서대로 표준 분해.
        layers = [(층번호, target, label [, yaw_offset [, r1_drop_slot [, z_extra_mm]]]), ...]
        r1_drop_slot 미지정 시 순서 기반 자동 할당 (첫번째=DROP, 두번째=DROP2, ...).
        """
        self.get_logger().info(f"{name} {len(layers)}층 분해 시작")
        self.move_both_home_pose()
        self.set_gripper(self.cli_g1, False)
        self.set_gripper(self.cli_g2, False)

        auto_slots = ["DROP" if i == 0 else f"DROP{i + 1}" for i in range(len(layers))]

        for idx, info in enumerate(layers):
            layer        = info[0]
            target       = info[1]
            label        = info[2]
            yaw_offset   = info[3] if len(info) > 3 else 0.0
            drop_slot    = info[4] if len(info) > 4 else auto_slots[idx]
            z_extra_mm   = info[5] if len(info) > 5 else 0.0
            bottom_label = layers[idx + 1][2] if idx + 1 < len(layers) else None

            if not self.robot1_pick_layer_and_drop(
                target, label, layer, drop_slot,
                bottom_label=bottom_label, yaw_offset=yaw_offset, z_extra_mm=z_extra_mm,
            ):
                return False

        self.get_logger().info(f"[완료] {name}")
        self.move_both_end_pose()
        return True

    # ── 아이템별 분해 시퀀스 ────────────────────────────────

    def run_battery_once(self):
        return self._run_two_layer("배터리", [
            (2, "2x2_yellow", "2x2 노랑"),
            (1, "2x2_blue",   "2x2 파랑"),
        ])

    def run_magnet_once(self):
        return self._run_two_layer("자석", [
            (2, "2x2_blue", "2x2 파랑"),
            (1, "2x2_red",  "2x2 빨강"),
        ])

    def run_estop_once(self):
        return self._run_two_layer("E-stop", [
            (2, "2x2_red",    "2x2 빨강", 90.0),
            (1, "2x4_yellow", "2x4 노랑"),
        ])

    def _run_new_seq(self, name: str, top_target: str, top_label: str,
                     mid_label: str, bot_label: str) -> bool:
        """
        신호등 시퀀스와 동일한 3층 분해 패턴.
          robot1: 3층 픽업 → DROP
          robot2: 2층 그립 → 대기 → DROP
          robot1: 1층 그립(DROP_AFTER_GRIP) → DROP_AFTER_DROP
        """
        self.get_logger().info(f"{name} 분해 시작")
        self.move_both_home_pose()
        self.set_gripper(self.cli_g1, False)
        self.set_gripper(self.cli_g2, False)

        if not self.robot1_top_pick(top_target, top_label, expected_layer=3):
            return False

        if not self.robot2_side_hold(mid_label):
            return False

        if not self.robot1_pull_up(top_label):
            return False

        if not self.robot2_return_home_holding(mid_label):
            return False

        self.get_logger().info(f"[DROP] robot1: {top_label} → DROP → DROP_AFTER_HOME")
        if not self.send_pose(self.cli_r1, "DROP"):
            return False
        self.sleep()
        self.set_gripper(self.cli_g1, False)
        if not self.send_pose(self.cli_r1, "DROP_AFTER_HOME"):
            return False
        self.sleep()

        self.get_logger().info(f"[SEP] robot2: separation_joint 이동 ({mid_label} 그립 유지)")
        if not self.send_pose(self.cli_r2, "SEPARATION"):
            return False
        self.sleep()

        self.get_logger().info(f"[GRIP] robot1: drop_after_grip_joint → {bot_label} 그립")
        if not self.send_pose(self.cli_r1, "DROP_AFTER_GRIP"):
            return False
        self.sleep()
        self.set_gripper(self.cli_g1, True)

        self.get_logger().info("[LIFT] robot2: Z 초기 상승 (Base)")
        req_z = GetTargetPose.Request()
        req_z.target_size = "Z_BASE"
        req_z.z = -self.INITIAL_LIFT
        self.call(self.cli_r2, req_z)
        self.sleep()

        self.get_logger().info("[MOVE] robot1: DROP_AFTER_HOME 이동")
        if not self.send_pose(self.cli_r1, "DROP_AFTER_HOME"):
            return False
        self.sleep()

        self.get_logger().info(f"[DROP] robot2: {mid_label} → DROP → HOME")
        if not self.send_pose(self.cli_r2, "DROP"):
            return False
        self.sleep()
        self.set_gripper(self.cli_g2, False)
        self.call(self.cli_h2, Trigger.Request())
        self.sleep()

        self.get_logger().info(f"[DROP] robot1: {bot_label} → DROP_AFTER_DROP → HOME")
        if not self.send_pose(self.cli_r1, "DROP_AFTER_DROP"):
            return False
        self.sleep()
        self.set_gripper(self.cli_g1, False)
        self.call(self.cli_h1, Trigger.Request())
        self.sleep()

        self.get_logger().info(f"[완료] {name}")
        self.move_both_end_pose()
        return True

    def run_carrot_once(self):
        return self._run_new_seq("당근", "2x2_green", "3층 초록", "2층 노랑", "1층 노랑")

    def run_small_tree_once(self):
        return self._run_new_seq("작은 나무", "2x2_green", "3층 초록", "2층 4x2 초록", "1층 노랑")

    def run_traffic_light_once(self):
        return self._run_new_seq("신호등", "2x2_red", "3층 빨강", "2층 노랑", "1층 초록")

    def run_hammer_once(self):
        return self._run_new_seq("망치", "4x2_blue", "3층 파랑", "2층 빨강", "1층 빨강")

    def run_big_carrot_once(self):
        return self._run_layers("큰 당근", [
            (4, "2x2_green",  "2x2 초록"),
            (3, "4x2_yellow", "4x2 노랑"),
            (2, "2x2_yellow", "2x2 노랑"),
            (1, "2x2_yellow", "2x2 노랑"),
        ])

    def run_burger_once(self):
        """버거: 3층=4x2노랑 / 2층=2x2빨강→4x2빨강(Y필터) / 1층=4x2노랑"""
        self.get_logger().info("버거 분해 시작")
        self.move_both_home_pose()
        self.set_gripper(self.cli_g1, False)
        self.set_gripper(self.cli_g2, False)

        steps = [
            lambda: self.robot1_pick_layer_and_drop(
                "4x2_yellow", "3층 4x2 노랑", 3, "DROP",
                bottom_label="2x2 빨강"),
            lambda: self.robot1_pick_layer_and_drop(
                "2x2_red", "2층 2x2 빨강", 2, "DROP2",
                bottom_label="4x2 빨강"),
            lambda: self.robot1_pick_layer_and_drop(
                "4x2_red", "2층 4x2 빨강", 2, "DROP3",
                bottom_label="4x2 노랑", prefer_max_y=True),
            lambda: self.robot1_pick_layer_and_drop(
                "4x2_yellow", "1층 4x2 노랑", 1, "DROP4"),
        ]

        for step in steps:
            if not step():
                return False

        self.get_logger().info("[완료] 버거")
        self.move_both_end_pose()
        return True

    def run_ice_cream_once(self):
        self.get_logger().info("아이스크림 5층 분해 시작")
        self.move_both_home_pose()
        self.set_gripper(self.cli_g1, False)
        self.set_gripper(self.cli_g2, False)

        steps = [
            lambda: self.robot1_pick_layer_and_drop(
                "2x2_green", "4층 2x2 초록", 4, "DROP",
                bottom_label="2x2 파랑", robot2_drop_slot="DROP3"),
            lambda: self.robot1_pick_layer_and_drop(
                "2x2_blue", "3층 2x2 파랑", 3, "DROP2",
                bottom_label="2x2 빨강", robot2_drop_slot="DROP2"),
            lambda: self.robot1_pick_layer_and_drop(
                "2x2_red", "3층 2x2 빨강", 3, "DROP3",
                bottom_label="4x2 노랑", robot2_drop_slot="DROP2"),
            lambda: self.robot1_pick_layer_and_drop(
                "4x2_yellow", "2층 4x2 노랑", 2, "DROP4",
                bottom_label="2x2 노랑", robot2_drop_slot="DROP2"),
            lambda: self.robot1_pick_layer_and_drop(
                "2x2_yellow", "1층 2x2 노랑", 1, "DROP5"),
        ]

        for step in steps:
            if not step():
                return False

        self.get_logger().info("[완료] 아이스크림")
        self.move_both_end_pose()
        return True

    def run_big_tree_once(self):
        """큰 나무: 비표준 순서가 필요한 특수 시퀀스"""
        self.get_logger().info("큰 나무 특수 분해 시작")
        self.move_both_home_pose()
        self.set_gripper(self.cli_g1, False)
        self.set_gripper(self.cli_g2, False)

        steps = [
            lambda: self.robot1_pick_layer_and_drop(
                "2x2_green", "4층 2x2 초록", 4, "DROP",
                bottom_label="4x2 초록", robot2_drop_slot="DROP3"),
            lambda: self.robot1_pick_layer_place_center_robot2_drop(
                "4x2_green", "2층 4x2 초록 (1층 분리용)", 2,
                bottom_label="2x2 노랑", robot2_drop_slot="DROP2", z_extra_mm=13.0),
            lambda: self.robot1_pick_layer_and_drop(
                "2x2_yellow", "1층 2x2 노랑", 1, "DROP2"),
            lambda: self.robot1_pick_layer_drop_top_then_robot2_drop(
                "4x2_green", "3층 4x2 초록", 3, "DROP3",
                bottom_label="2층 4x2 초록", robot2_drop_slot="DROP2"),
            lambda: self.robot1_pick_layer_and_drop(
                "4x2_green", "남은 2층 4x2 초록", 2, "DROP4"),
            lambda: self.robot1_pick_layer_and_drop(
                "2x2_green", "남은 2층 2x2 초록", 2, "DROP5"),
        ]

        for step in steps:
            if not step():
                return False

        self.get_logger().info("[완료] 큰 나무")
        self.move_both_end_pose()
        return True

    # ── 메인 루프 ────────────────────────────────────────────

    def run(self):
        self.move_both_home_pose()

        MENU = {
            "1":  ("배터리",     self.run_battery_once),
            "2":  ("자석",       self.run_magnet_once),
            "3":  ("E-stop",     self.run_estop_once),
            "4":  ("당근",       self.run_carrot_once),
            "5":  ("작은 나무",   self.run_small_tree_once),
            "6":  ("신호등",     self.run_traffic_light_once),
            "7":  ("망치",       self.run_hammer_once),
            "8":  ("큰 당근",    self.run_big_carrot_once),
            "9":  ("버거",       self.run_burger_once),
            "10": ("아이스크림",  self.run_ice_cream_once),
            "11": ("큰 나무",    self.run_big_tree_once),
        }
        aliases = {
            "battery": "1", "배터리": "1",
            "magnet": "2", "자석": "2",
            "estop": "3", "e-stop": "3", "비상정지": "3",
            "carrot": "4", "당근": "4",
            "tree": "5", "small_tree": "5", "작은나무": "5",
            "traffic": "6", "traffic_light": "6", "신호등": "6",
            "hammer": "7", "망치": "7",
            "bigcarrot": "8", "큰당근": "8",
            "burger": "9", "버거": "9",
            "icecream": "10", "아이스크림": "10",
            "bigtree": "11", "큰나무": "11",
        }

        print("\n=== 분해 노드 ===")
        for key, (name, _) in MENU.items():
            print(f"  {key:>2}: {name}")
        print("   q: 종료\n")

        while rclpy.ok():
            raw = input("선택 [1~11/q]: ").strip().lower().replace(" ", "")
            if raw in ("q", "quit", "exit", "종료"):
                self.get_logger().info("종료")
                break
            key = aliases.get(raw, raw)
            if key not in MENU:
                print("1~11 또는 q 를 입력하세요.")
                continue
            MENU[key][1]()


def main():
    rclpy.init()
    node = BatteryDualDisassembly()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()


# 1. 홈 포즈 한 번 수정해서 4층 블록 얕게 잡는 문제 해결하기
# 2. 1층짜리 블록 로봇 2가 내려놓을 때 안정적으로 놓게 새로 조인트 잡기 