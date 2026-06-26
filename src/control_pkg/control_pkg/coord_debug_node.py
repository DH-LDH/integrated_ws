#!/usr/bin/env python3
"""
coord_debug_node.py
====================
버드아이뷰(KHJ) ↔ 로봇1 좌표 변환 디버깅 노드.

배경
----
로봇1(vision camera)이 남쪽, 로봇2(birdseye camera)가 서쪽에 설치되어 있어
두 카메라의 좌표축이 90° 회전 관계에 있다.
birdseye_assembly.py의 offset_cm 계산은 "robot1이 북쪽"을 가정했기 때문에
실제 환경에서는 축 매핑이 틀릴 수 있다.

이 노드는 /khj_point와 /get_target_pose를 동시에 읽어
"KHJ offset_cm → robot 좌표" 변환의 오차를 측정하고
올바른 2×2 선형 변환 행렬을 찾는다.

사용법 (3단계)
--------------
  1) 블록을 여러 위치(최소 2곳, 권장 4곳+)에 놓고 각 위치마다:
       ros2 service call /coord_debug/snapshot std_srvs/srv/Trigger

  2) 수집 완료 후 변환 행렬 피팅:
       ros2 service call /coord_debug/fit std_srvs/srv/Trigger

  3) 권장 계수를 master_node.py 파라미터에 반영

서비스
------
  /coord_debug/snapshot      Trigger  현재 KHJ + Vision 스냅샷 → 캘리브 데이터 추가
  /coord_debug/snapshot_raw  Trigger  birdseye 원시 데이터 + Vision (khj_point 불필요)
  /coord_debug/fit           Trigger  수집된 점으로 2×2 변환 행렬 피팅
  /coord_debug/clear         Trigger  수집 데이터 초기화
  /coord_debug/diagnose      Trigger  단일 점에서 축 방향 진단 (수집 없이)

snapshot_raw 사용법 (target_id_map이 비어있을 때)
-------------------------------------------------
  블록 1개만 놓고:
    ros2 service call /coord_debug/snapshot_raw std_srvs/srv/Trigger \\
      --ros-args -p raw_class:=2x2_red

수동 데이터 추가 (vision이 없을 때)
------------------------------------
  ros2 topic pub --once /coord_debug/add_point std_msgs/msg/String \\
    'data: "{\"class_name\":\"2x2_red\",\"robot_x\":0.1234,\"robot_y\":-0.0567}"'

토픽 구독
---------
  /khj_point                          String   KHJ 버드아이뷰 데이터
  /birdseye_assembly/object_positions String   원시 버드아이뷰 픽셀 데이터
  /coord_debug/add_point              String   수동 캘리브 점 추가

토픽 발행
---------
  /coord_debug/status  String  현재 변환 비교 결과 (1Hz)
  /coord_debug/matrix  String  fit 결과 JSON

파라미터
--------
  ~vision_classes      (str)   쉼표 구분 클래스 목록 (default: 2x2_red,2x2_blue,...)
  ~khj_y_to_robot_x    (float) 현재 offset_cm.y → robot_x 계수 (default: 0.008678)
  ~khj_x_to_robot_y    (float) 현재 offset_cm.x → robot_y 계수 (default: -0.000311)
  ~vision_timeout_sec  (float) /get_target_pose 타임아웃 (default: 3.0)
"""

import json
import math
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

try:
    from srvs_pkg.srv import GetTargetPose
    _VISION_SRV_AVAILABLE = True
except ImportError:
    _VISION_SRV_AVAILABLE = False

DEFAULT_CLASSES = (
    "2x2_red,2x2_blue,2x2_green,2x2_yellow,"
    "4x2_red,4x2_blue,4x2_green,4x2_yellow"
)
KHJ_Y_TO_ROBOT_X_DEFAULT = 0.008678
KHJ_X_TO_ROBOT_Y_DEFAULT = -0.000311

# master_node.py의 CLASS_TO_TARGET_ID와 동일하게 유지
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


def class_to_vision_id(class_name: str) -> str:
    """클래스명 → vision 서비스 target_color ID 변환 (master_node와 동일 로직)."""
    name = class_name.strip()
    if name.isdigit():
        return name
    return CLASS_TO_TARGET_ID.get(name, name)

SEPARATOR = "=" * 80
THIN_LINE = "-" * 80


# ──────────────────────────────────────────────────────────────────────────── #
# 수학 유틸 (numpy 없이 2×2 최소제곱법)
# ──────────────────────────────────────────────────────────────────────────── #

def lstsq_2x2(points):
    """
    points: [(ox, oy, rx, ry), ...]

    모델:
        rx = a*ox + b*oy
        ry = c*ox + d*oy

    반환: (a, b, c, d, rmse_x_mm, rmse_y_mm)
    """
    n = len(points)
    if n < 2:
        raise ValueError(f"최소 2점 필요 (현재 {n}점)")

    sxx = sxy = syy = 0.0
    sx_rx = sy_rx = 0.0
    sx_ry = sy_ry = 0.0

    for ox, oy, rx, ry in points:
        sxx += ox * ox
        sxy += ox * oy
        syy += oy * oy
        sx_rx += ox * rx
        sy_rx += oy * rx
        sx_ry += ox * ry
        sy_ry += oy * ry

    det = sxx * syy - sxy * sxy
    if abs(det) < 1e-15:
        raise ValueError("행렬 역산 불가 (점들이 한 직선 위에 있음)")

    inv00 = syy / det
    inv01 = -sxy / det
    inv11 = sxx / det

    a = inv00 * sx_rx + inv01 * sy_rx
    b = inv01 * sx_rx + inv11 * sy_rx
    c = inv00 * sx_ry + inv01 * sy_ry
    d = inv01 * sx_ry + inv11 * sy_ry

    errs_x = [(a * ox + b * oy - rx) * 1000.0 for ox, oy, rx, ry in points]
    errs_y = [(c * ox + d * oy - ry) * 1000.0 for ox, oy, rx, ry in points]
    rmse_x = math.sqrt(sum(e**2 for e in errs_x) / n)
    rmse_y = math.sqrt(sum(e**2 for e in errs_y) / n)

    return a, b, c, d, rmse_x, rmse_y


# ──────────────────────────────────────────────────────────────────────────── #
# ROS 노드
# ──────────────────────────────────────────────────────────────────────────── #

class CoordDebugNode(Node):
    def __init__(self):
        super().__init__("coord_debug_node")

        # ── 파라미터 ── #
        self.vision_classes = [
            c.strip()
            for c in self.declare_parameter(
                "vision_classes", DEFAULT_CLASSES
            ).value.split(",")
            if c.strip()
        ]
        self.KHJ_Y_TO_ROBOT_X = float(
            self.declare_parameter(
                "khj_y_to_robot_x", KHJ_Y_TO_ROBOT_X_DEFAULT
            ).value
        )
        self.KHJ_X_TO_ROBOT_Y = float(
            self.declare_parameter(
                "khj_x_to_robot_y", KHJ_X_TO_ROBOT_Y_DEFAULT
            ).value
        )
        self.vision_timeout = float(
            self.declare_parameter("vision_timeout_sec", 3.0).value
        )
        # snapshot_raw 용 기본 클래스명 (파라미터로 오버라이드 가능)
        self.raw_class = str(
            self.declare_parameter("raw_class", "").value
        ).strip()

        # ── 상태 ── #
        self._khj_data: dict = {}
        self._bird_raw: dict = {}
        # (ox_cm, oy_cm, robot_x, robot_y, class_name)
        self._calib_points: list = []

        # ── 콜백 그룹 (서비스 콜백 안에서 다른 서비스 호출 허용) ── #
        self._cbg = ReentrantCallbackGroup()

        # ── 구독 ── #
        self.create_subscription(String, "/khj_point", self._khj_cb, 10)
        self.create_subscription(
            String, "/birdseye_assembly/object_positions", self._bird_cb, 10
        )
        self.create_subscription(
            String, "/coord_debug/add_point", self._add_point_cb, 10
        )

        # ── 발행 ── #
        self._status_pub = self.create_publisher(String, "/coord_debug/status", 10)
        self._matrix_pub = self.create_publisher(String, "/coord_debug/matrix", 10)

        # ── 서비스 (ReentrantCallbackGroup 적용) ── #
        self.create_service(Trigger, "/coord_debug/snapshot",     self._snapshot_cb,     callback_group=self._cbg)
        self.create_service(Trigger, "/coord_debug/snapshot_raw", self._snapshot_raw_cb, callback_group=self._cbg)
        self.create_service(Trigger, "/coord_debug/fit",          self._fit_cb,          callback_group=self._cbg)
        self.create_service(Trigger, "/coord_debug/clear",        self._clear_cb,        callback_group=self._cbg)
        self.create_service(Trigger, "/coord_debug/diagnose",     self._diagnose_cb,     callback_group=self._cbg)

        # ── Vision 클라이언트 (같은 콜백 그룹) ── #
        self._vision_cli = None
        if _VISION_SRV_AVAILABLE:
            self._vision_cli = self.create_client(
                GetTargetPose, "/get_target_pose", callback_group=self._cbg
            )

        # ── 타이머 ── #
        self.create_timer(1.0, self._status_timer)

        self.get_logger().info(
            "\n" + SEPARATOR + "\n"
            "  coord_debug_node 시작\n"
            "\n"
            f"  현재 변환 계수:\n"
            f"    KHJ_Y_TO_ROBOT_X = {self.KHJ_Y_TO_ROBOT_X:.6f}  "
            f"(offset_cm.y → robot_x)\n"
            f"    KHJ_X_TO_ROBOT_Y = {self.KHJ_X_TO_ROBOT_Y:.6f}  "
            f"(offset_cm.x → robot_y)\n"
            "\n"
            "  서비스:\n"
            "    /coord_debug/snapshot  → 블록 스냅샷 + 캘리브 데이터 수집\n"
            "    /coord_debug/fit       → 변환 행렬 피팅 (2점+)\n"
            "    /coord_debug/diagnose  → 현재 점 축 방향 진단\n"
            "    /coord_debug/clear     → 데이터 초기화\n"
            "\n"
            "  수동 데이터 추가:\n"
            '    ros2 topic pub --once /coord_debug/add_point std_msgs/msg/String \\\n'
            '    \'data: "{\\\"class_name\\\":\\\"2x2_red\\\","'
            '\\\"robot_x\\\":0.12,\\\"robot_y\\\":-0.05}"\'\n'
            + SEPARATOR
        )

    # ────────────────────────────────────────────────────────────────────── #
    # 콜백
    # ────────────────────────────────────────────────────────────────────── #

    def _khj_cb(self, msg: String):
        try:
            self._khj_data = json.loads(msg.data)
        except Exception as e:
            self.get_logger().warn(f"[KHJ] 파싱 실패: {e}")

    def _bird_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
            self._bird_raw = {
                str(obj["id"]): obj for obj in data.get("objects", [])
            }
        except Exception as e:
            self.get_logger().warn(f"[BIRDSEYE] 파싱 실패: {e}")

    def _add_point_cb(self, msg: String):
        """
        수동으로 캘리브레이션 점을 추가한다.
        형식: {"class_name": "2x2_red", "robot_x": 0.12, "robot_y": -0.05}
        반드시 /khj_point에 같은 class_name의 블록이 있어야 한다.
        """
        try:
            data = json.loads(msg.data)
            class_name = str(data.get("class_name", "")).strip()
            rx = float(data["robot_x"])
            ry = float(data["robot_y"])
        except Exception as e:
            self.get_logger().error(f"[ADD_POINT] 파싱 실패: {e}")
            return

        khj_xy = self._find_khj_offset(class_name)
        if khj_xy is None:
            self.get_logger().error(
                f"[ADD_POINT] /khj_point에 '{class_name}' 없음"
            )
            return

        ox, oy = khj_xy
        self._calib_points.append((ox, oy, rx, ry, class_name))
        self.get_logger().info(
            f"[ADD_POINT] 수동 추가: {class_name} "
            f"offset_cm=({ox:.2f},{oy:.2f}) robot=({rx:.4f},{ry:.4f}m) "
            f"[총 {len(self._calib_points)}점]"
        )

    # ────────────────────────────────────────────────────────────────────── #
    # 타이머 (1Hz 상태 출력)
    # ────────────────────────────────────────────────────────────────────── #

    def _status_timer(self):
        if not self._khj_data:
            return

        lines = [
            "── KHJ 현황 (1Hz) ──",
            f"  {'#':3} {'class':15} │ {'offset_cm.x':>12} {'offset_cm.y':>12} "
            f"│ {'calc_rx(m)':>12} {'calc_ry(m)':>12}",
        ]
        for id_str, blk in sorted(self._khj_data.items()):
            cn = blk.get("class_name", "?")
            ox = float(blk.get("offset_cm", {}).get("x", 0.0))
            oy = float(blk.get("offset_cm", {}).get("y", 0.0))
            crx = oy * self.KHJ_Y_TO_ROBOT_X
            cry = ox * self.KHJ_X_TO_ROBOT_Y
            lines.append(
                f"  #{id_str:2} {cn:15} │ {ox:>+12.2f}cm {oy:>+12.2f}cm "
                f"│ {crx:>+12.4f}m {cry:>+12.4f}m"
            )

        if self._calib_points:
            lines.append(f"  [캘리브 데이터 {len(self._calib_points)}점 수집됨]")

        status = "\n".join(lines)
        self._status_pub.publish(String(data=status))
        # 변환 현황은 터미널에 너무 자주 출력하면 노이즈가 되므로 debug 레벨만 사용
        self.get_logger().debug(status)

    # ────────────────────────────────────────────────────────────────────── #
    # 내부 유틸
    # ────────────────────────────────────────────────────────────────────── #

    def _find_khj_offset(self, class_name: str, local_id: int = 0):
        """class_name에 해당하는 offset_cm (ox, oy) 반환. 없으면 None."""
        if not self._khj_data:
            return None
        matches = sorted(
            [
                (k, v)
                for k, v in self._khj_data.items()
                if str(v.get("class_name", "")).strip() == class_name
            ],
            key=lambda kv: int(kv[0]) if kv[0].isdigit() else 0,
        )
        if not matches or local_id >= len(matches):
            return None
        _, blk = matches[local_id]
        ox = float(blk.get("offset_cm", {}).get("x", 0.0))
        oy = float(blk.get("offset_cm", {}).get("y", 0.0))
        return ox, oy

    def _call_vision(self, class_name: str):
        """/get_target_pose 호출. 클래스명 → 숫자 ID 변환. MultiThreadedExecutor 환경에서 안전."""
        if self._vision_cli is None:
            return None
        if not self._vision_cli.wait_for_service(timeout_sec=1.0):
            return None
        target_id = class_to_vision_id(class_name)
        self.get_logger().info(
            f"[VISION] '{class_name}' → target_color='{target_id}' 호출"
        )
        req = GetTargetPose.Request(target_color=target_id)
        future = self._vision_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.vision_timeout)
        if not future.done():
            self.get_logger().warn(f"[VISION] '{class_name}' 타임아웃({self.vision_timeout}s)")
            return None
        return future.result()

    def _format_ratio(self, numerator, denominator, label):
        if abs(denominator) < 1e-4:
            return f"  {label:30s} : N/A (분모 ≈ 0)"
        r = numerator / denominator
        return f"  {label:30s} : {r:+.6f} m/cm"

    # ────────────────────────────────────────────────────────────────────── #
    # 서비스: snapshot
    # ────────────────────────────────────────────────────────────────────── #

    def _snapshot_cb(self, request, response):
        """
        현재 /khj_point 에 있는 블록들을 /get_target_pose 로 순서대로 조회하고
        두 좌표 시스템의 차이를 출력한다.
        성공한 쌍은 캘리브레이션 데이터로 저장된다.
        """
        if not self._khj_data:
            msg = "[SNAPSHOT] /khj_point 데이터 없음. khj_point_node 실행 여부를 확인하세요."
            self.get_logger().warn(msg)
            response.success = False
            response.message = msg
            return response

        vision_ok = (
            self._vision_cli is not None
            and self._vision_cli.wait_for_service(timeout_sec=1.0)
        )
        if not vision_ok:
            self.get_logger().warn(
                "[SNAPSHOT] /get_target_pose 서비스 없음. "
                "KHJ 데이터만 출력합니다."
            )

        lines = [
            SEPARATOR,
            "  COORDINATE SNAPSHOT",
            THIN_LINE,
        ]

        added = 0
        for id_str, blk in sorted(self._khj_data.items()):
            cn  = blk.get("class_name", "?")
            ox  = float(blk.get("offset_cm", {}).get("x", 0.0))
            oy  = float(blk.get("offset_cm", {}).get("y", 0.0))
            dist = float(blk.get("dist_cm", 0.0))

            calc_rx = oy * self.KHJ_Y_TO_ROBOT_X
            calc_ry = ox * self.KHJ_X_TO_ROBOT_Y

            lines.append(
                f"  ── 블록 #{id_str}  class={cn}  dist={dist:.1f}cm"
            )
            lines.append(
                f"     KHJ offset_cm  : x={ox:+8.2f}cm  y={oy:+8.2f}cm"
            )
            lines.append(
                f"     현재 변환 결과 : robot_x={calc_rx:+.4f}m  robot_y={calc_ry:+.4f}m"
            )

            if vision_ok and cn in self.vision_classes:
                vr = self._call_vision(cn)
                if vr and vr.success:
                    vx, vy = vr.x, vr.y
                    err_x_mm = (vx - calc_rx) * 1000.0
                    err_y_mm = (vy - calc_ry) * 1000.0
                    err_d_mm = math.hypot(err_x_mm, err_y_mm)

                    lines.append(
                        f"     Vision (실제)   : robot_x={vx:+.4f}m  robot_y={vy:+.4f}m"
                    )
                    lines.append(
                        f"     오차            : Δx={err_x_mm:+.1f}mm  Δy={err_y_mm:+.1f}mm  "
                        f"|Δ|={err_d_mm:.1f}mm"
                    )

                    # 축별 비율 (어느 KHJ 축이 어느 로봇 축에 매핑되는지 판단)
                    lines.append("     개별 축 비율 (올바른 계수 추정용):")
                    lines.append(self._format_ratio(vx, oy, "vision_x / offset_cm.y  [현재 사용]"))
                    lines.append(self._format_ratio(vx, ox, "vision_x / offset_cm.x  [교차항?]"))
                    lines.append(self._format_ratio(vy, ox, "vision_y / offset_cm.x  [현재 사용]"))
                    lines.append(self._format_ratio(vy, oy, "vision_y / offset_cm.y  [교차항?]"))

                    self._calib_points.append((ox, oy, vx, vy, cn))
                    added += 1
                else:
                    lines.append(f"     Vision           : 인식 실패 (성공=False 또는 타임아웃)")
            else:
                lines.append(
                    "     Vision           : 미사용 (서비스 없거나 클래스 목록 외)"
                )
            lines.append("")

        lines.append(THIN_LINE)
        lines.append(
            f"  캘리브 점 {added}개 추가. 누적 총 {len(self._calib_points)}개"
        )
        if len(self._calib_points) >= 2:
            lines.append(
                "  ✔ 충분한 데이터. "
                "'ros2 service call /coord_debug/fit std_srvs/srv/Trigger' 로 피팅하세요."
            )
        else:
            need = 2 - len(self._calib_points)
            lines.append(
                f"  ✗ 피팅까지 {need}개 더 필요 (블록을 다른 위치로 옮기고 다시 snapshot)."
            )
        lines.append(SEPARATOR)

        output = "\n".join(lines)
        self.get_logger().info("\n" + output)
        response.success = True
        response.message = (
            f"snapshot 완료: {added}개 추가 (누적 {len(self._calib_points)}개)"
        )
        return response

    # ────────────────────────────────────────────────────────────────────── #
    # 서비스: snapshot_raw
    # ────────────────────────────────────────────────────────────────────── #

    def _snapshot_raw_cb(self, request, response):
        """
        /khj_point (target_id_map) 없이 /birdseye_assembly/object_positions 를
        직접 읽어 Vision 좌표와 비교한다.

        블록이 1개만 있을 때 가장 유용하다.
        매칭은 birdseye 객체 순서대로 vision 결과를 대응시킨다.

        raw_class 파라미터(또는 노드 실행 시 --ros-args -p raw_class:=2x2_red)로
        조회할 클래스명을 지정한다.
        """
        if not self._bird_raw:
            msg = (
                "[SNAPSHOT_RAW] /birdseye_assembly/object_positions 데이터 없음. "
                "birdseye_assembly 노드 실행 여부를 확인하세요."
            )
            self.get_logger().warn(msg)
            response.success = False
            response.message = msg
            return response

        # raw_class 파라미터 실시간 조회 (노드 실행 중에도 변경 가능)
        try:
            raw_class = str(
                self.get_parameter("raw_class").value
            ).strip()
        except Exception:
            raw_class = self.raw_class

        vision_ok = (
            self._vision_cli is not None
            and self._vision_cli.wait_for_service(timeout_sec=1.0)
        )

        lines = [
            SEPARATOR,
            "  SNAPSHOT_RAW  (birdseye 직접 사용, target_id_map 불필요)",
            f"  raw_class 파라미터: '{raw_class}' "
            + ("" if raw_class else "← 비어있음. 아래 안내 참고."),
            THIN_LINE,
        ]

        # birdseye 객체 목록 출력
        lines.append(
            f"  [birdseye 객체] 총 {len(self._bird_raw)}개"
        )
        lines.append(
            f"  {'id':>4} {'label':>12} {'offset_cm.x':>13} {'offset_cm.y':>13} "
            f"{'dist_cm':>10} {'calc_rx(m)':>12} {'calc_ry(m)':>12}"
        )

        bird_list = sorted(self._bird_raw.values(), key=lambda o: o.get("id", 0))
        for obj in bird_list:
            oid   = obj.get("id", "?")
            label = obj.get("label", "?")
            ox    = float(obj.get("offset_cm", {}).get("x", 0.0))
            oy    = float(obj.get("offset_cm", {}).get("y", 0.0))
            dist  = float(obj.get("dist_cm", 0.0))
            crx   = oy * self.KHJ_Y_TO_ROBOT_X
            cry   = ox * self.KHJ_X_TO_ROBOT_Y
            lines.append(
                f"  {oid:>4} {label:>12} {ox:>+13.2f}cm {oy:>+13.2f}cm "
                f"{dist:>10.1f}cm {crx:>+12.4f}m {cry:>+12.4f}m"
            )

        lines.append("")

        # Vision 조회
        if not raw_class:
            lines.append("  !! raw_class 파라미터를 설정하세요 !!")
            lines.append("  노드 재실행 시:")
            lines.append(
                "    ros2 run control_pkg coord_debug_node "
                "--ros-args -p raw_class:=2x2_red"
            )
            lines.append("  또는 실행 중 변경:")
            lines.append(
                "    ros2 param set /coord_debug_node raw_class 2x2_red"
            )
            lines.append(SEPARATOR)
            output = "\n".join(lines)
            self.get_logger().info("\n" + output)
            response.success = False
            response.message = "raw_class 파라미터 필요"
            return response

        lines.append(f"  [Vision 조회] class='{raw_class}'")
        if not vision_ok:
            lines.append("  ✗ /get_target_pose 서비스 없음.")
            lines.append(SEPARATOR)
            output = "\n".join(lines)
            self.get_logger().info("\n" + output)
            response.success = False
            response.message = "vision 서비스 없음"
            return response

        vr = self._call_vision(raw_class)
        if not (vr and vr.success):
            lines.append(f"  ✗ '{raw_class}' 인식 실패.")
            lines.append(SEPARATOR)
            output = "\n".join(lines)
            self.get_logger().info("\n" + output)
            response.success = False
            response.message = f"'{raw_class}' vision 인식 실패"
            return response

        vx, vy = vr.x, vr.y
        lines.append(
            f"  Vision robot_x={vx:+.4f}m  robot_y={vy:+.4f}m  "
            f"z={vr.z:.4f}m  yaw={vr.yaw:.1f}deg"
        )
        lines.append("")

        # 가장 가까운 birdseye 객체를 현재 변환으로 비교
        if len(bird_list) == 1:
            obj = bird_list[0]
            ox  = float(obj.get("offset_cm", {}).get("x", 0.0))
            oy  = float(obj.get("offset_cm", {}).get("y", 0.0))
        else:
            # 현재 변환 결과가 vision과 가장 가까운 객체 선택
            def dist_to_vision(o):
                cx = float(o.get("offset_cm", {}).get("y", 0.0)) * self.KHJ_Y_TO_ROBOT_X
                cy = float(o.get("offset_cm", {}).get("x", 0.0)) * self.KHJ_X_TO_ROBOT_Y
                return math.hypot(cx - vx, cy - vy)
            obj = min(bird_list, key=dist_to_vision)
            lines.append(
                f"  (birdseye 객체 {len(bird_list)}개 중 변환값이 vision과 가장 가까운 "
                f"#{obj.get('id')} 선택)"
            )
            ox  = float(obj.get("offset_cm", {}).get("x", 0.0))
            oy  = float(obj.get("offset_cm", {}).get("y", 0.0))

        calc_rx = oy * self.KHJ_Y_TO_ROBOT_X
        calc_ry = ox * self.KHJ_X_TO_ROBOT_Y
        err_x   = (vx - calc_rx) * 1000.0
        err_y   = (vy - calc_ry) * 1000.0

        lines += [
            f"  매칭된 birdseye 객체 #{obj.get('id')} ({obj.get('label','?')})",
            f"    offset_cm.x = {ox:+.2f}cm  offset_cm.y = {oy:+.2f}cm",
            f"    현재 변환   → robot_x={calc_rx:+.4f}m  robot_y={calc_ry:+.4f}m",
            f"    Vision 실제 → robot_x={vx:+.4f}m  robot_y={vy:+.4f}m",
            f"    오차        → Δx={err_x:+.1f}mm  Δy={err_y:+.1f}mm  "
            f"|Δ|={math.hypot(err_x,err_y):.1f}mm",
            "",
            "    개별 축 비율 (올바른 계수 추정용):",
            self._format_ratio(vx, oy, "vision_x / offset_cm.y  [현재 b]"),
            self._format_ratio(vx, ox, "vision_x / offset_cm.x  [교차 a]"),
            self._format_ratio(vy, ox, "vision_y / offset_cm.x  [현재 c]"),
            self._format_ratio(vy, oy, "vision_y / offset_cm.y  [교차 d]"),
        ]

        self._calib_points.append((ox, oy, vx, vy, raw_class))
        lines += [
            "",
            THIN_LINE,
            f"  캘리브 점 1개 추가. 누적 총 {len(self._calib_points)}개",
            "  블록을 다른 위치로 옮기고 다시 snapshot_raw 호출하세요.",
            SEPARATOR,
        ]

        output = "\n".join(lines)
        self.get_logger().info("\n" + output)
        response.success = True
        response.message = (
            f"snapshot_raw 완료: {raw_class} "
            f"birdseye({ox:+.1f},{oy:+.1f})cm vision({vx:+.4f},{vy:+.4f})m "
            f"(누적 {len(self._calib_points)}점)"
        )
        return response

    # ────────────────────────────────────────────────────────────────────── #
    # 서비스: diagnose
    # ────────────────────────────────────────────────────────────────────── #

    def _diagnose_cb(self, request, response):
        """
        캘리브레이션 데이터를 쓰지 않고, 현재 /khj_point 데이터와 vision 결과를
        비교해 좌표축 방향을 진단한다.
        """
        if not self._khj_data:
            response.success = False
            response.message = "[DIAGNOSE] /khj_point 데이터 없음"
            return response

        vision_ok = (
            self._vision_cli is not None
            and self._vision_cli.wait_for_service(timeout_sec=1.0)
        )

        lines = [
            SEPARATOR,
            "  AXIS ORIENTATION DIAGNOSIS",
            "  (로봇1=남쪽, 버드아이=서쪽 기준 좌표 방향 분석)",
            THIN_LINE,
            "  [좌표계 설명]",
            "  birdseye_assembly.py 계산식:",
            "    offset_cm.x = (by - py_probe) * CM_PER_PX_Y   ← 버드아이 Y픽셀(깊이)",
            "    offset_cm.y = -(bx - px_probe) * CM_PER_PX_X  ← 버드아이 X픽셀(좌우, 반전)",
            "  master_node.py 변환식:",
            f"    robot_x = offset_cm.y * {self.KHJ_Y_TO_ROBOT_X:.6f}",
            f"    robot_y = offset_cm.x * {self.KHJ_X_TO_ROBOT_Y:.6f}",
            THIN_LINE,
        ]

        pairs = []
        for id_str, blk in sorted(self._khj_data.items()):
            cn = blk.get("class_name", "?")
            ox = float(blk.get("offset_cm", {}).get("x", 0.0))
            oy = float(blk.get("offset_cm", {}).get("y", 0.0))

            if not (vision_ok and cn in self.vision_classes):
                continue

            vr = self._call_vision(cn)
            if not (vr and vr.success):
                continue

            vx, vy = vr.x, vr.y
            pairs.append((cn, ox, oy, vx, vy))

            lines.append(f"  블록: {cn} (#{id_str})")
            lines.append(f"    offset_cm    x={ox:+8.2f}cm  y={oy:+8.2f}cm")
            lines.append(f"    vision robot x={vx:+8.4f}m   y={vy:+8.4f}m")
            lines.append("")

        if not pairs:
            lines.append("  ✗ Vision 매칭 결과 없음.")
            lines.append(SEPARATOR)
            output = "\n".join(lines)
            self.get_logger().info("\n" + output)
            response.success = False
            response.message = "Vision 매칭 실패"
            return response

        # ── 방향성 진단 ── #
        lines.append("  [축 방향 진단]")
        lines.append(
            "  각 블록에서 offset_cm.x, offset_cm.y의 부호와 크기를 "
            "vision robot_x, robot_y와 비교:"
        )
        lines.append("")
        lines.append(
            f"  {'class':15} {'ox→rx 비율':>15} {'oy→rx 비율':>15} "
            f"{'ox→ry 비율':>15} {'oy→ry 비율':>15}"
        )
        lines.append("  " + "-" * 65)

        ratios = {"ox_rx": [], "oy_rx": [], "ox_ry": [], "oy_ry": []}
        for cn, ox, oy, vx, vy in pairs:
            r_ox_rx = vx / ox if abs(ox) > 0.5 else float("nan")
            r_oy_rx = vx / oy if abs(oy) > 0.5 else float("nan")
            r_ox_ry = vy / ox if abs(ox) > 0.5 else float("nan")
            r_oy_ry = vy / oy if abs(oy) > 0.5 else float("nan")

            def fmt(v):
                return f"{v:+.5f}" if not math.isnan(v) else "  N/A   "

            lines.append(
                f"  {cn:15} {fmt(r_ox_rx):>15} {fmt(r_oy_rx):>15} "
                f"{fmt(r_ox_ry):>15} {fmt(r_oy_ry):>15}"
            )

            if not math.isnan(r_ox_rx): ratios["ox_rx"].append(r_ox_rx)
            if not math.isnan(r_oy_rx): ratios["oy_rx"].append(r_oy_rx)
            if not math.isnan(r_ox_ry): ratios["ox_ry"].append(r_ox_ry)
            if not math.isnan(r_oy_ry): ratios["oy_ry"].append(r_oy_ry)

        def mean_std(lst):
            if not lst:
                return float("nan"), float("nan")
            m = sum(lst) / len(lst)
            if len(lst) < 2:
                return m, float("nan")
            s = math.sqrt(sum((v - m) ** 2 for v in lst) / (len(lst) - 1))
            return m, s

        lines.append(THIN_LINE)
        lines.append("  [평균 비율 요약] (분산이 작을수록 올바른 매핑)")
        for key, lbl in [
            ("ox_rx", "offset_cm.x → robot_x"),
            ("oy_rx", "offset_cm.y → robot_x  ← 현재 사용"),
            ("ox_ry", "offset_cm.x → robot_y  ← 현재 사용"),
            ("oy_ry", "offset_cm.y → robot_y"),
        ]:
            m, s = mean_std(ratios[key])
            m_str = f"{m:+.5f}" if not math.isnan(m) else "  N/A   "
            s_str = f"±{s:.5f}" if not math.isnan(s) else ""
            lines.append(f"  {lbl:40s}: {m_str} {s_str}")

        lines.append("")
        lines.append("  [결론 힌트]")
        lines.append(
            "  • 분산이 가장 작은 비율 쌍이 올바른 축 매핑입니다."
        )
        lines.append(
            "  • 로봇1(남쪽) + 버드아이(서쪽) 경우, 예상 올바른 매핑:"
        )
        lines.append(
            "    - offset_cm.x  →  robot_y  (버드아이 Y픽셀 = 깊이 = 로봇 Y방향)"
        )
        lines.append(
            "    - offset_cm.y  →  robot_x  (버드아이 X픽셀 반전 = 가로 = 로봇 X방향)"
        )
        lines.append(
            "  • 만약 부호가 반대이면 계수의 부호를 반전시키세요."
        )
        lines.append(SEPARATOR)

        output = "\n".join(lines)
        self.get_logger().info("\n" + output)
        response.success = True
        response.message = f"diagnose 완료 ({len(pairs)}블록)"
        return response

    # ────────────────────────────────────────────────────────────────────── #
    # 서비스: fit
    # ────────────────────────────────────────────────────────────────────── #

    def _fit_cb(self, request, response):
        """
        수집된 (offset_cm.x, offset_cm.y) → (robot_x, robot_y) 쌍으로
        2×2 선형 변환 행렬을 최소제곱법으로 피팅한다.

        모델:
          robot_x = a*offset_cm.x + b*offset_cm.y
          robot_y = c*offset_cm.x + d*offset_cm.y

        권장:
          b → KHJ_Y_TO_ROBOT_X
          c → KHJ_X_TO_ROBOT_Y
        """
        n = len(self._calib_points)
        if n < 2:
            msg = f"[FIT] 데이터가 {n}개뿐입니다. 최소 2개 필요 (권장 4개+)."
            self.get_logger().warn(msg)
            response.success = False
            response.message = msg
            return response

        points_4d = [(ox, oy, rx, ry) for ox, oy, rx, ry, _ in self._calib_points]
        try:
            a, b, c, d, rmse_x, rmse_y = lstsq_2x2(points_4d)
        except ValueError as e:
            msg = f"[FIT] 피팅 실패: {e}"
            self.get_logger().error(msg)
            response.success = False
            response.message = msg
            return response

        lines = [
            SEPARATOR,
            f"  변환 행렬 피팅 결과  (N={n}점)",
            THIN_LINE,
            "  모델:  robot = M × offset_cm",
            "",
            f"     robot_x = {a:+.6f} × offset_cm.x + {b:+.6f} × offset_cm.y",
            f"     robot_y = {c:+.6f} × offset_cm.x + {d:+.6f} × offset_cm.y",
            "",
            f"  RMSE: robot_x={rmse_x:.2f}mm  robot_y={rmse_y:.2f}mm",
            THIN_LINE,
            "  현재 설정 (단순 대각 모델):",
            f"    KHJ_Y_TO_ROBOT_X = {self.KHJ_Y_TO_ROBOT_X:.6f}  "
            f"(b 역할)  ← {b:+.6f} 으로 변경 권장",
            f"    KHJ_X_TO_ROBOT_Y = {self.KHJ_X_TO_ROBOT_Y:.6f}  "
            f"(c 역할)  ← {c:+.6f} 으로 변경 권장",
            "",
            "  !! 교차항 크기 확인 (0에 가까울수록 단순 대각 모델이 충분) !!",
            f"    a (offset_cm.x → robot_x) = {a:+.6f}  "
            + ("≈ 0 ✔" if abs(a) < 0.002 else "≠ 0 ← 교차항 고려 필요"),
            f"    d (offset_cm.y → robot_y) = {d:+.6f}  "
            + ("≈ 0 ✔" if abs(d) < 0.002 else "≠ 0 ← 교차항 고려 필요"),
            THIN_LINE,
            "  각 측정점 잔차:",
            f"  {'#':3} {'class':15} {'ox':>8} {'oy':>8} "
            f"{'vision_x':>10} {'vision_y':>10} "
            f"{'pred_x':>10} {'pred_y':>10} "
            f"{'err_x':>8} {'err_y':>8}",
        ]

        for i, (ox, oy, rx, ry, cn) in enumerate(self._calib_points):
            px = a * ox + b * oy
            py = c * ox + d * oy
            lines.append(
                f"  {i+1:3} {cn:15} {ox:>+8.2f} {oy:>+8.2f} "
                f"{rx:>+10.4f} {ry:>+10.4f} "
                f"{px:>+10.4f} {py:>+10.4f} "
                f"{(px-rx)*1000:>+8.1f} {(py-ry)*1000:>+8.1f}"
            )

        lines += [
            THIN_LINE,
            "  master_node.py 파라미터 업데이트 명령:",
            "  (ros2 param set 또는 launch 파일에 반영)",
            f"    ros2 param set /master_node khj_y_to_robot_x {b:.6f}",
            f"    ros2 param set /master_node khj_x_to_robot_y {c:.6f}",
            SEPARATOR,
        ]

        output = "\n".join(lines)
        self.get_logger().info("\n" + output)

        matrix_json = json.dumps(
            {
                "n": n,
                "matrix": {"a": a, "b": b, "c": c, "d": d},
                "rmse_mm": {"x": rmse_x, "y": rmse_y},
                "current": {
                    "KHJ_Y_TO_ROBOT_X": self.KHJ_Y_TO_ROBOT_X,
                    "KHJ_X_TO_ROBOT_Y": self.KHJ_X_TO_ROBOT_Y,
                },
                "recommended": {
                    "KHJ_Y_TO_ROBOT_X": b,
                    "KHJ_X_TO_ROBOT_Y": c,
                    "cross_a": a,
                    "cross_d": d,
                },
                "points": [
                    {
                        "class": cn,
                        "ox": ox,
                        "oy": oy,
                        "rx": rx,
                        "ry": ry,
                    }
                    for ox, oy, rx, ry, cn in self._calib_points
                ],
            },
            indent=2,
        )
        self._matrix_pub.publish(String(data=matrix_json))

        response.success = True
        response.message = (
            f"피팅 완료 N={n}, RMSE x={rmse_x:.1f}mm y={rmse_y:.1f}mm, "
            f"권장: KHJ_Y_TO_ROBOT_X={b:.6f} KHJ_X_TO_ROBOT_Y={c:.6f}"
        )
        return response

    # ────────────────────────────────────────────────────────────────────── #
    # 서비스: clear
    # ────────────────────────────────────────────────────────────────────── #

    def _clear_cb(self, request, response):
        count = len(self._calib_points)
        self._calib_points.clear()
        self.get_logger().info(f"[CLEAR] 캘리브 데이터 {count}개 초기화")
        response.success = True
        response.message = f"cleared {count} points"
        return response


# ──────────────────────────────────────────────────────────────────────────── #
# main
# ──────────────────────────────────────────────────────────────────────────── #

def main(args=None):
    rclpy.init(args=args)
    node = CoordDebugNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
