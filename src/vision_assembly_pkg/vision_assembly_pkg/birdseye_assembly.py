import json
import math

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger


IMAGE_TOPIC     = "/camera/camera/color/image_raw"
SUMMARY_TOPIC   = "/decision_assembly/summary"
POSITIONS_TOPIC = "/birdseye_assembly/object_positions"

# decision_assembly.py의 ROI polygon 그대로 사용 (좌상→우상→우하→좌하)
ROI_POLYGON = [(234, 0), (478, 0), (640, 480), (90, 480)]

# 버드뷰 출력 해상도
BIRD_W = 500
BIRD_H = 600

# image_raw 상의 중앙점 (probe) 픽셀
PROBE_PIXEL = (553, 233)

# ── 실제 거리 스케일 ─────────────────────────────────────────────
# 좌상→우상: 38 cm  /  좌상→좌하: 74 cm
ROI_REAL_W_CM = 38.0
ROI_REAL_H_CM = 74.0
CM_PER_PX_X   = ROI_REAL_W_CM / BIRD_W   # 0.076  cm/px
CM_PER_PX_Y   = ROI_REAL_H_CM / BIRD_H   # 0.1233 cm/px

WINDOW_MAIN = "birdseye_assembly"

BOX_COLORS = [
    (0, 255, 255),
    (0, 180, 255),
    (80, 220, 80),
    (255, 160, 80),
    (220, 120, 255),
    (255, 220, 80),
]


# ── 유틸 ─────────────────────────────────────────────────────────

def make_sensor_qos():
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )


def compute_homography(roi_polygon, bird_w, bird_h):
    src = np.float32(roi_polygon)
    dst = np.float32([
        [0,      0],
        [bird_w, 0],
        [bird_w, bird_h],
        [0,      bird_h],
    ])
    return cv2.getPerspectiveTransform(src, dst)


def project_point(pt, H):
    arr = np.float32([[list(pt)]])
    result = cv2.perspectiveTransform(arr, H)
    x, y = result[0][0]
    return int(round(x)), int(round(y))


def real_dist_cm(p1, p2):
    """버드뷰 픽셀 두 점 사이의 실제 거리 (cm)."""
    dx = (p2[0] - p1[0]) * CM_PER_PX_X
    dy = (p2[1] - p1[1]) * CM_PER_PX_Y
    return math.sqrt(dx * dx + dy * dy)


def raw_centers_from_det(det):
    """estimated_count 기준으로 bbox를 분할하여 중심점 목록 반환.

    count=1: 단일 중심점 (기존 동작)
    count=N: 긴 축 방향으로 N등분 → N개 중심점
    """
    count = max(1, int(det.get("estimated_count", 1)))
    x1, y1, x2, y2 = [float(v) for v in det["bbox_xyxy"]]

    if count == 1:
        if "center_xy" in det:
            return [(int(det["center_xy"][0]), int(det["center_xy"][1]))]
        return [(int((x1 + x2) / 2), int((y1 + y2) / 2))]

    w, h = x2 - x1, y2 - y1
    centers = []
    if w >= h:
        # 가로 방향으로 등분
        step = w / count
        cy = int((y1 + y2) / 2)
        for i in range(count):
            centers.append((int(x1 + step * (i + 0.5)), cy))
    else:
        # 세로 방향으로 등분
        step = h / count
        cx = int((x1 + x2) / 2)
        for i in range(count):
            centers.append((cx, int(y1 + step * (i + 0.5))))
    return centers


# ── 시각화 ────────────────────────────────────────────────────────

def draw_projected(bird_bgr, proj_list, probe_bird, total_blocks):
    """proj_list: list of {id, center_bird, dist_cm, label, offset_cm}"""
    out = bird_bgr.copy()

    for pd in proj_list:
        bx, by = pd["center_bird"]
        if 0 <= bx < BIRD_W and 0 <= by < BIRD_H:
            cv2.line(out, probe_bird, (bx, by), (160, 160, 160), 1, cv2.LINE_AA)

    for pd in proj_list:
        bx, by = pd["center_bird"]
        dist   = pd["dist_cm"]
        label  = pd["label"]
        obj_id = pd["id"]
        ox_cm  = pd["offset_cm"]["x"]
        oy_cm  = pd["offset_cm"]["y"]
        color  = BOX_COLORS[(obj_id - 1) % len(BOX_COLORS)]

        in_view = 0 <= bx < BIRD_W and 0 <= by < BIRD_H
        draw_x  = max(4, min(BIRD_W - 4, bx))
        draw_y  = max(4, min(BIRD_H - 4, by))

        cv2.circle(out, (draw_x, draw_y), 7, color, -1)
        cv2.circle(out, (draw_x, draw_y), 9, (255, 255, 255), 1)
        # ID 뱃지
        cv2.putText(out, str(obj_id), (draw_x - 4, draw_y + 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 0, 0), 2, cv2.LINE_AA)

        if not in_view:
            cv2.putText(out, "out", (draw_x + 10, draw_y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1, cv2.LINE_AA)
            continue

        cv2.putText(out, f"#{obj_id} {label}",
                    (bx + 12, max(16, by - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.48, color, 2, cv2.LINE_AA)
        cv2.putText(out, f"dx{ox_cm:+.1f} dy{oy_cm:+.1f}cm",
                    (bx + 12, max(30, by + 12)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, color, 1, cv2.LINE_AA)
        mx = (bx + probe_bird[0]) // 2
        my = (by + probe_bird[1]) // 2
        cv2.putText(out, f"{dist:.1f}cm",
                    (mx + 4, my),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.52, (160, 220, 255), 2, cv2.LINE_AA)

    cv2.putText(out, f"objs={len(proj_list)}  total_blocks={total_blocks}",
                (10, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2, cv2.LINE_AA)
    cv2.putText(out,
                f"scale {CM_PER_PX_X:.3f}cm/px(x)  {CM_PER_PX_Y:.3f}cm/px(y)",
                (6, BIRD_H - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.38, (100, 100, 100), 1, cv2.LINE_AA)
    return out


# ── ROS 노드 ──────────────────────────────────────────────────────

class BirdseyeAssemblyNode(Node):
    def __init__(self):
        super().__init__("birdseye_assembly")

        self.bridge = CvBridge()
        self.H = compute_homography(ROI_POLYGON, BIRD_W, BIRD_H)
        self.probe_bird = project_point(PROBE_PIXEL, self.H)

        # decision_assembly 에서 수신한 최신 detections
        self.latest_detections = []
        self.total_blocks = 0

        # 조립 시작 전 데이터 동결 플래그
        self._locked = False

        cv2.namedWindow(WINDOW_MAIN, cv2.WINDOW_NORMAL)

        self.image_sub = self.create_subscription(
            Image, IMAGE_TOPIC, self.image_cb, make_sensor_qos()
        )
        self.summary_sub = self.create_subscription(
            String, SUMMARY_TOPIC, self.summary_cb, 10
        )
        self.positions_pub = self.create_publisher(String, POSITIONS_TOPIC, 10)
        self.lock_srv = self.create_service(
            Trigger, '/lock_positions', self._lock_cb
        )

        self.get_logger().info(f"[BIRDSEYE] image    : {IMAGE_TOPIC}")
        self.get_logger().info(f"[BIRDSEYE] summary  : {SUMMARY_TOPIC}")
        self.get_logger().info(f"[BIRDSEYE] positions: {POSITIONS_TOPIC}")
        self.get_logger().info(
            f"[BIRDSEYE] probe  image_raw={PROBE_PIXEL}  birdseye={self.probe_bird}"
        )
        self.get_logger().info(
            f"[BIRDSEYE] scale  x={CM_PER_PX_X:.4f} cm/px  y={CM_PER_PX_Y:.4f} cm/px"
        )
        dist_check = real_dist_cm((BIRD_W, 0), self.probe_bird)
        self.get_logger().info(
            f"[BIRDSEYE] 우상→중앙점 계산 거리: {dist_check:.1f} cm  (실측 50.0 cm)"
        )

    # ── 콜백 ──────────────────────────────────────────────────────

    def _lock_cb(self, request, response):
        """조립 시작 시 호출 → 이후 summary 업데이트 차단, 현재 상태 동결."""
        self._locked = True
        self.get_logger().info(
            f'[BIRDSEYE] 위치 데이터 동결 완료 '
            f'(objects={len(self.latest_detections)}, total_blocks={self.total_blocks})'
        )
        response.success = True
        response.message = f'locked with {len(self.latest_detections)} objects'
        return response

    def summary_cb(self, msg):
        if self._locked:
            return   # 동결 중 — 업데이트 차단
        try:
            data = json.loads(msg.data)
            self.latest_detections = data.get("detections", [])
            self.total_blocks      = int(data.get("total_blocks", 0))
        except Exception as e:
            self.get_logger().warn(f"[BIRDSEYE] summary 파싱 실패: {e}")

    def image_cb(self, msg):
        try:
            image_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"[BIRDSEYE] image 변환 실패: {e}")
            return

        # 버드아이뷰 변환
        bird = cv2.warpPerspective(image_bgr, self.H, (BIRD_W, BIRD_H))

        # decision_assembly detections → 버드뷰 좌표로 투영
        # estimated_count > 1 이면 bbox를 분할하여 개별 위치로 확장
        raw_proj = []
        px_probe, py_probe = self.probe_bird
        for det in self.latest_detections:
            name    = str(det.get("class_name", "blob"))
            centers = raw_centers_from_det(det)

            for cx_raw, cy_raw in centers:
                bx, by = project_point((cx_raw, cy_raw), self.H)
                dist   = real_dist_cm((bx, by), self.probe_bird)

                # robot1 좌표계 기준 offset.
                # robot1이 북쪽, birdseye/camera2 기준이 동쪽을 본다고 보고 축을 90도 맞춘다.
                # birdseye y가 커지는 방향 -> robot1 x 양수, birdseye x가 커지는 방향 -> robot1 y 음수.
                offset_x_cm = (by - py_probe) * CM_PER_PX_Y
                offset_y_cm = -(bx - px_probe) * CM_PER_PX_X

                raw_proj.append({
                    "center_bird": (bx, by),
                    "dist_cm":     dist,
                    "label":       name,
                    "offset_cm":   {"x": round(offset_x_cm, 2), "y": round(offset_y_cm, 2)},
                })

        # 픽셀 Y 내림차순 정렬 후 ID 부여 (y 큰 것 = 카메라에 가까운 것 = #1)
        raw_proj.sort(key=lambda p: p["center_bird"][1], reverse=True)
        proj_list = [{**p, "id": i + 1} for i, p in enumerate(raw_proj)]

        # 위치 토픽 발행
        pub_data = {
            "objects": [
                {
                    "id":          pd["id"],
                    "label":       pd["label"],
                    "center_bird": list(pd["center_bird"]),
                    "dist_cm":     round(pd["dist_cm"], 2),
                    "offset_cm":   pd["offset_cm"],
                }
                for pd in proj_list
            ]
        }
        self.positions_pub.publish(String(data=json.dumps(pub_data, ensure_ascii=False)))

        annotated = draw_projected(bird, proj_list, self.probe_bird, self.total_blocks)

        # ROI 경계
        cv2.rectangle(annotated, (0, 0), (BIRD_W - 1, BIRD_H - 1), (255, 255, 0), 2)

        # 중앙점 (probe)
        px, py = self.probe_bird
        if 0 <= px < BIRD_W and 0 <= py < BIRD_H:
            cv2.circle(annotated, (px, py), 8, (0, 0, 255), -1)
            cv2.circle(annotated, (px, py), 10, (255, 255, 255), 2)
            cv2.putText(
                annotated,
                f"center raw({PROBE_PIXEL[0]},{PROBE_PIXEL[1]})",
                (max(0, px + 13), max(18, py - 4)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 2, cv2.LINE_AA,
            )

        cv2.imshow(WINDOW_MAIN, annotated)
        cv2.waitKey(1)

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BirdseyeAssemblyNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
