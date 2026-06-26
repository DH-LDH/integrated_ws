# vision_node.py
import json
import time

import cv2
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
from srvs_pkg.srv import GetTargetPose
from vision_pkg import INUVisionCall as ivc
from ultralytics import YOLO as _YOLO

ID_MAP_TOPIC    = "/target_id_map"
PUBLISH_HZ      = 0.5   # 발행 주기 (초)
SCAN_CONF_THRESHOLD = 0.75  # 이 값 미만의 YOLO 감지는 노이즈로 간주하고 무시
SCAN_VISUALIZE = True
SCAN_WINDOW_NAME = "target_id_map_scan"


class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        self.srv = self.create_service(GetTargetPose, '/get_target_pose', self.get_pose_cb)
        self.get_logger().info('[VISION] 초기화 중... VisionManager 로드')

        self.vision = ivc.VisionManager()
        self.get_logger().info('[VISION] RealSense 카메라 지속 스트리밍 시작 중...')
        self.vision.start_camera(mode="mid_50", V_visualize=False)

        # filter_overlapping_masks 우회 — 직접 YOLO 모델 로드 (startup 시 1회)
        self._yolo = _YOLO(self.vision.yolo_dir_brick)
        self.get_logger().info('[VISION] YOLO 모델 로드 완료 (raw 추론용)')

        # [{x_mm, class_name}, ...] — scan 마다 교체
        self._blocks: list = []
        self._scanning: bool = False

        if SCAN_VISUALIZE:
            cv2.namedWindow(SCAN_WINDOW_NAME, cv2.WINDOW_NORMAL)

        self._pub     = self.create_publisher(String, ID_MAP_TOPIC, 10)
        self._pub_timer = self.create_timer(PUBLISH_HZ, self._publish_id_map)
        self.scan_srv   = self.create_service(Trigger, '/scan_all_blocks', self.scan_all_blocks_cb)

        self.get_logger().info(f'[VISION] pub  : {ID_MAP_TOPIC} @ {PUBLISH_HZ}s')
        self.get_logger().info('[VISION] scan : /scan_all_blocks 서비스 호출 시에만 스캔 (자동 스캔 없음)')

    def destroy_node(self):
        if hasattr(self, 'vision'):
            self.vision.close()
        if SCAN_VISUALIZE:
            cv2.destroyWindow(SCAN_WINDOW_NAME)
        super().destroy_node()

    # ── 공통 스캔 로직 ──────────────────────────────────────────────────

    def _do_scan(self):
        """capture → raw YOLO 직접 추론 → _blocks 갱신 (filter_overlapping_masks 우회)."""
        if self._scanning:
            self.get_logger().warn('[VISION][SCAN] 이전 스캔 진행 중, 건너뜀')
            return False
        self._scanning = True
        try:
            self.vision.capture_camera(mode='mid_50', V_visualize=False)
            color_rgb = self.vision.color_rgb
            if color_rgb is None:
                self.get_logger().error('[VISION][SCAN] 캡처 이미지 없음')
                return False

            # RGB → BGR 변환 후 YOLO 직접 추론
            color_bgr = cv2.cvtColor(color_rgb, cv2.COLOR_RGB2BGR)
            results = self._yolo(color_bgr, verbose=False)

            if not results or results[0].boxes is None or len(results[0].boxes) == 0:
                self.get_logger().warn('[VISION][SCAN] 검출된 객체 없음')
                self._blocks = []
                return True

            boxes = results[0].boxes
            names = results[0].names

            raw = []
            for i in range(len(boxes)):
                conf = float(boxes.conf[i].item())
                if conf < SCAN_CONF_THRESHOLD:
                    cls_id = int(boxes.cls[i].item())
                    self.get_logger().warn(
                        f'[VISION][SCAN] 낮은 confidence 제외: '
                        f'{names[cls_id]} conf={conf:.2f} (threshold={SCAN_CONF_THRESHOLD})'
                    )
                    continue
                cls_id = int(boxes.cls[i].item())
                x1, y1, x2, y2 = boxes.xyxy[i].cpu().numpy()
                cx = float((x1 + x2) / 2)  # bounding box X 중심 (픽셀)
                raw.append({
                    'x_px': cx,
                    'class_name': str(names[cls_id]),
                    'confidence': conf,
                    'bbox_xyxy': [float(x1), float(y1), float(x2), float(y2)],
                })

            # X 픽셀 오름차순 정렬 → ID 1 = 가장 왼쪽
            raw.sort(key=lambda b: b['x_px'])
            self._blocks = [
                {'x_mm': b['x_px'], 'class_name': b['class_name']}
                for b in raw
            ]
            if SCAN_VISUALIZE:
                self._show_scan_debug(color_bgr, raw)
            self.get_logger().info(
                f'[VISION][SCAN] {len(self._blocks)}개 감지: '
                + str([(b["class_name"], round(b["x_mm"], 1)) for b in self._blocks])
            )
            return True
        except Exception as e:
            self.get_logger().error(f'[VISION][SCAN] 오류: {e}')
            return False
        finally:
            self._scanning = False

    def _show_scan_debug(self, color_bgr, sorted_blocks):
        vis = color_bgr.copy()

        for idx, block in enumerate(sorted_blocks, start=1):
            x1, y1, x2, y2 = [int(round(v)) for v in block['bbox_xyxy']]
            class_name = block['class_name']
            conf = block['confidence']
            x_px = block['x_px']

            cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 255, 255), 2)
            cv2.circle(vis, (int(round(x_px)), int(round((y1 + y2) / 2))), 4, (0, 0, 255), -1)
            cv2.putText(
                vis,
                f"#{idx} {class_name} {conf:.2f} x={x_px:.0f}",
                (x1, max(18, y1 - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 255, 255),
                2,
                cv2.LINE_AA,
            )

        cv2.putText(
            vis,
            f"target_id_map count={len(sorted_blocks)}  sorted by pixel x ascending",
            (10, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )
        cv2.imshow(SCAN_WINDOW_NAME, vis)
        cv2.waitKey(1)

    # ── /scan_all_blocks 서비스 ─────────────────────────────────────────

    def scan_all_blocks_cb(self, request, response):
        ok = self._do_scan()
        response.success = ok
        response.message = f'{len(self._blocks)} blocks found' if ok else 'scan failed'
        if ok:
            self._publish_id_map()
        return response

    # ── /target_id_map 주기 발행 ────────────────────────────────────────

    def _publish_id_map(self):
        payload = {str(i + 1): b['class_name'] for i, b in enumerate(self._blocks)}
        self._pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))

    # ── /get_target_pose 서비스 ─────────────────────────────────────────

    def get_pose_cb(self, request, response):
        t_total = time.perf_counter()
        target_str = request.target_color.strip()
        self.get_logger().info(f'[VISION] 서비스 요청 수신 - target ID: {target_str}')

        try:
            if not target_str.isdigit():
                self.get_logger().error(
                    f'[VISION] 잘못된 입력입니다. 숫자 ID를 입력하세요: {target_str}'
                )
                response.success = False
                return response

            target_id = int(target_str)
            t_step = time.perf_counter()
            result = self.vision.run_pipeline_by_id(
                target_id=target_id,
                local_id=0,
                camera_mode="mid_50",
                brick_search_mode="fine",
                V_visualize_capture=False,
                V_visualize_search=False
            )
            self.get_logger().info(
                f'[TIME] node.run_pipeline_by_id: {time.perf_counter() - t_step:.3f}s'
            )

            if result["success"]:
                response.success    = True
                response.x          = float(result["x_mm"] / 1000.0)
                response.y          = float(result["y_mm"] / 1000.0)
                response.z          = float(result["z_mm"] / 1000.0)
                response.yaw        = float(result["yaw_deg"] -90.0)
                response.class_name = str(result["class_name"])
                self.get_logger().info(
                    f'[VISION] 타겟 발견! '
                    f'ID={result["target_id"]}, Class={result["class_name"]}, '
                    f'X={result["x_mm"]:.1f}mm, Y={result["y_mm"]:.1f}mm, '
                    f'Z={result["z_mm"]:.1f}mm, Yaw={result["yaw_deg"]:.2f}deg'
                )
            else:
                response.success = False
                self.get_logger().error(
                    f'[VISION] 타겟 탐색 실패: '
                    f'ID={result.get("target_id")}, Class={result.get("class_name")}, '
                    f'Reason={result.get("reason")}'
                )

        except Exception as e:
            self.get_logger().error(f'[VISION] 처리 중 심각한 오류 발생: {e}')
            response.success = False

        self.get_logger().info(
            f'[TIME] node.service_cb.total: {time.perf_counter() - t_total:.3f}s'
        )
        return response


def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
