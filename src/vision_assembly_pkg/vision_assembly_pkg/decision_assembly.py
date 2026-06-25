import argparse
import json
import os
import time
from collections import Counter

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Int32, String


PACKAGE_NAME = "vision_assembly_pkg"
DEFAULT_IMAGE_TOPIC = "/camera/camera/color/image_raw"
DEFAULT_ANNOTATED_TOPIC = "/decision_assembly/annotated_image"
DEFAULT_COUNT_TOPIC = "/decision_assembly/block_count"
DEFAULT_SUMMARY_TOPIC = "/decision_assembly/summary"
DEFAULT_ROI_POLYGON = [204, 26, 433, 27, 640, 480, 0, 480]
WINDOW_NAME = "decision_assembly"
BOX_COLORS = [
    (0, 255, 255),
    (0, 180, 255),
    (80, 220, 80),
    (255, 160, 80),
    (220, 120, 255),
    (255, 220, 80),
]


def get_model_path():
    try:
        share_dir = get_package_share_directory(PACKAGE_NAME)
        model_path = os.path.join(share_dir, "resource", "best.pt")
        if os.path.exists(model_path):
            return model_path
    except Exception:
        pass

    pkg_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    model_path = os.path.join(pkg_dir, "resource", "best.pt")
    if os.path.exists(model_path):
        return model_path

    raise FileNotFoundError("best.pt 모델 파일을 찾지 못했습니다.")


def make_sensor_qos():
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=1
    )


def parse_roi(values):
    if values is None:
        return None

    x1, y1, x2, y2 = values
    if x2 <= x1 or y2 <= y1:
        raise ValueError("--roi는 x1 y1 x2 y2 순서이며, x2>x1/y2>y1 이어야 합니다.")
    return tuple(int(round(v)) for v in values)


def parse_roi_polygon(values):
    if values is None:
        return None
    if len(values) < 6 or len(values) % 2 != 0:
        raise ValueError("--roi-polygon은 x y 좌표쌍 3개 이상이어야 합니다.")
    points = []
    for idx in range(0, len(values), 2):
        points.append((int(round(values[idx])), int(round(values[idx + 1]))))
    return points


def clamp_roi(roi, image_shape):
    if roi is None:
        return None

    height, width = image_shape[:2]
    x1, y1, x2, y2 = roi
    x1 = max(0, min(width - 1, x1))
    y1 = max(0, min(height - 1, y1))
    x2 = max(x1 + 1, min(width, x2))
    y2 = max(y1 + 1, min(height, y2))
    return x1, y1, x2, y2


def clamp_polygon(points, image_shape):
    if points is None:
        return None

    height, width = image_shape[:2]
    clamped = []
    for x, y in points:
        clamped.append((
            max(0, min(width - 1, int(x))),
            max(0, min(height - 1, int(y))),
        ))
    return clamped


def polygon_bounds(points):
    xs = [point[0] for point in points]
    ys = [point[1] for point in points]
    return min(xs), min(ys), max(xs) + 1, max(ys) + 1


def crop_for_inference(image_bgr, roi=None, roi_polygon=None):
    active_polygon = clamp_polygon(roi_polygon, image_bgr.shape)
    active_roi = clamp_roi(roi, image_bgr.shape) if active_polygon is None else polygon_bounds(active_polygon)
    if active_roi is None:
        return image_bgr, 0, 0, None, None

    x1, y1, x2, y2 = active_roi
    inference_bgr = image_bgr[y1:y2, x1:x2].copy()

    if active_polygon is not None:
        shifted = np.array(
            [[px - x1, py - y1] for px, py in active_polygon],
            dtype=np.int32
        )
        mask = np.zeros(inference_bgr.shape[:2], dtype=np.uint8)
        cv2.fillPoly(mask, [shifted], 255)
        inference_bgr = cv2.bitwise_and(inference_bgr, inference_bgr, mask=mask)

    return inference_bgr, x1, y1, active_roi, active_polygon


def make_crop_mask(crop_shape, offset_x, offset_y, active_polygon):
    mask = np.full(crop_shape[:2], 255, dtype=np.uint8)
    if active_polygon is None:
        return mask

    mask.fill(0)
    shifted = np.array(
        [[px - offset_x, py - offset_y] for px, py in active_polygon],
        dtype=np.int32
    )
    cv2.fillPoly(mask, [shifted], 255)
    return mask


def count_blocks_yolo(model, image_bgr, conf, iou, device, roi=None, roi_polygon=None):
    inference_bgr, offset_x, offset_y, active_roi, active_polygon = crop_for_inference(
        image_bgr=image_bgr,
        roi=roi,
        roi_polygon=roi_polygon
    )

    results = model(
        inference_bgr,
        conf=conf,
        iou=iou,
        device=device,
        verbose=False
    )

    if len(results) == 0 or results[0].boxes is None:
        return 0, Counter(), [], results, active_roi, active_polygon

    result = results[0]
    class_ids = result.boxes.cls.cpu().numpy().astype(int)
    confidences = result.boxes.conf.cpu().numpy()
    boxes_xyxy = result.boxes.xyxy.cpu().numpy()
    names = result.names
    class_counts = Counter(str(names[int(cls_id)]) for cls_id in class_ids)
    detections = []

    for box, cls_id, score in zip(boxes_xyxy, class_ids, confidences):
        x1, y1, x2, y2 = box
        detections.append({
            "class_id": int(cls_id),
            "class_name": str(names[int(cls_id)]),
            "confidence": float(score),
            "bbox_xyxy": [
                int(round(x1 + offset_x)),
                int(round(y1 + offset_y)),
                int(round(x2 + offset_x)),
                int(round(y2 + offset_y)),
            ],
        })

    return int(len(class_ids)), class_counts, detections, results, active_roi, active_polygon


def count_blocks_cv(
    image_bgr,
    roi=None,
    roi_polygon=None,
    min_area=900.0,
    max_area=0.0,
    sat_min=35,
    edge_low=50,
    edge_high=150,
    morph_kernel=7,
    close_iterations=2,
    open_iterations=1,
):
    crop_bgr, offset_x, offset_y, active_roi, active_polygon = crop_for_inference(
        image_bgr=image_bgr,
        roi=roi,
        roi_polygon=roi_polygon
    )
    roi_mask = make_crop_mask(crop_bgr.shape, offset_x, offset_y, active_polygon)

    hsv = cv2.cvtColor(crop_bgr, cv2.COLOR_BGR2HSV)
    saturation = hsv[:, :, 1]
    sat_mask = cv2.inRange(saturation, int(sat_min), 255)

    gray = cv2.cvtColor(crop_bgr, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (5, 5), 0)
    edge_mask = cv2.Canny(gray, int(edge_low), int(edge_high))

    foreground = cv2.bitwise_or(sat_mask, edge_mask)
    foreground = cv2.bitwise_and(foreground, roi_mask)

    kernel_size = max(1, int(morph_kernel))
    if kernel_size % 2 == 0:
        kernel_size += 1
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))

    if close_iterations > 0:
        foreground = cv2.morphologyEx(
            foreground,
            cv2.MORPH_CLOSE,
            kernel,
            iterations=int(close_iterations)
        )
    if open_iterations > 0:
        foreground = cv2.morphologyEx(
            foreground,
            cv2.MORPH_OPEN,
            kernel,
            iterations=int(open_iterations)
        )

    contours, _ = cv2.findContours(foreground, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    detections = []
    max_area = float(max_area)

    for contour in contours:
        area = float(cv2.contourArea(contour))
        if area < float(min_area):
            continue
        if max_area > 0.0 and area > max_area:
            continue

        x, y, w, h = cv2.boundingRect(contour)
        if w <= 2 or h <= 2:
            continue

        moments = cv2.moments(contour)
        if moments["m00"] != 0:
            center_x = int(round(moments["m10"] / moments["m00"])) + offset_x
            center_y = int(round(moments["m01"] / moments["m00"])) + offset_y
        else:
            center_x = int(round(x + w / 2)) + offset_x
            center_y = int(round(y + h / 2)) + offset_y

        detections.append({
            "class_id": 0,
            "class_name": "blob",
            "confidence": 1.0,
            "area": area,
            "center_xy": [center_x, center_y],
            "bbox_xyxy": [
                int(x + offset_x),
                int(y + offset_y),
                int(x + w + offset_x),
                int(y + h + offset_y),
            ],
        })

    detections.sort(key=lambda det: det["area"], reverse=True)
    class_counts = Counter({"blob": len(detections)}) if detections else Counter()
    return len(detections), class_counts, detections, foreground, active_roi, active_polygon


def draw_annotated(image_bgr, detections, total_count, roi=None, roi_polygon=None):
    annotated = image_bgr.copy()

    if roi_polygon is not None:
        points = np.array(roi_polygon, dtype=np.int32)
        cv2.polylines(annotated, [points], isClosed=True, color=(255, 255, 0), thickness=2)
        x1, y1 = roi_polygon[0]
        cv2.putText(
            annotated,
            "ROI",
            (x1 + 6, max(22, y1 + 22)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 0),
            2,
            cv2.LINE_AA
        )
    elif roi is not None:
        x1, y1, x2, y2 = roi
        cv2.rectangle(annotated, (x1, y1), (x2, y2), (255, 255, 0), 2)
        cv2.putText(
            annotated,
            "ROI",
            (x1 + 6, max(22, y1 + 22)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 0),
            2,
            cv2.LINE_AA
        )

    for det in detections:
        x1, y1, x2, y2 = det["bbox_xyxy"]
        color = BOX_COLORS[det["class_id"] % len(BOX_COLORS)]
        if "area" in det:
            label = f'{det["class_name"]} {det["area"]:.0f}'
        else:
            label = f'{det["class_name"]} {det["confidence"]:.2f}'
        cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 2)
        if "center_xy" in det:
            cv2.circle(annotated, tuple(det["center_xy"]), 4, color, -1)
        cv2.putText(
            annotated,
            label,
            (x1, max(18, y1 - 6)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            color,
            2,
            cv2.LINE_AA
        )

    cv2.putText(
        annotated,
        f"total_blocks={total_count}",
        (12, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (0, 255, 255),
        2,
        cv2.LINE_AA
    )
    return annotated


class DecisionAssemblyNode(Node):
    def __init__(self, args):
        super().__init__("decision_assembly")

        self.image_topic = args.image_topic
        self.annotated_topic = args.annotated_topic
        self.count_topic = args.count_topic
        self.summary_topic = args.summary_topic
        self.method = args.method
        self.conf = args.conf
        self.iou = args.iou
        self.device = args.device
        self.cv_min_area = args.cv_min_area
        self.cv_max_area = args.cv_max_area
        self.cv_sat_min = args.cv_sat_min
        self.cv_edge_low = args.cv_edge_low
        self.cv_edge_high = args.cv_edge_high
        self.cv_morph_kernel = args.cv_morph_kernel
        self.cv_close_iterations = args.cv_close_iterations
        self.cv_open_iterations = args.cv_open_iterations
        self.process_every_n = max(1, args.process_every_n)
        self.roi = parse_roi(args.roi)
        self.roi_polygon = parse_roi_polygon(args.roi_polygon)
        self.show = args.show
        self.save_dir = args.save_dir
        self.frame_idx = 0
        self.last_log_time = 0.0
        self.log_period_sec = max(0.0, args.log_period_sec)
        self.mouse_xy = None
        self.roi_clicks = []

        if self.save_dir:
            os.makedirs(self.save_dir, exist_ok=True)

        if self.show:
            cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
            cv2.setMouseCallback(WINDOW_NAME, self.mouse_cb)

        self.bridge = CvBridge()

        self.model = None
        if self.method == "yolo":
            from ultralytics import YOLO

            model_path = get_model_path()
            self.get_logger().info(f"[DECISION] model: {model_path}")
            self.model = YOLO(model_path)

        self.count_pub = self.create_publisher(Int32, self.count_topic, 10)
        self.summary_pub = self.create_publisher(String, self.summary_topic, 10)
        self.annotated_pub = self.create_publisher(Image, self.annotated_topic, 10)

        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_cb,
            make_sensor_qos()
        )

        self.get_logger().info(f"[DECISION] subscribe image: {self.image_topic}")
        self.get_logger().info(f"[DECISION] publish count: {self.count_topic}")
        self.get_logger().info(f"[DECISION] publish summary: {self.summary_topic}")
        self.get_logger().info(f"[DECISION] publish annotated: {self.annotated_topic}")
        self.get_logger().info(f"[DECISION] method: {self.method}")
        if self.method == "cv":
            self.get_logger().info(
                "[DECISION] cv params: "
                f"min_area={self.cv_min_area}, max_area={self.cv_max_area}, "
                f"sat_min={self.cv_sat_min}, edge=({self.cv_edge_low},{self.cv_edge_high}), "
                f"morph={self.cv_morph_kernel}, close={self.cv_close_iterations}, open={self.cv_open_iterations}"
            )
        if self.roi is None:
            self.get_logger().info("[DECISION] roi: disabled")
        else:
            self.get_logger().info(f"[DECISION] roi xyxy: {self.roi}")
        if self.roi_polygon is not None:
            self.get_logger().info(f"[DECISION] roi polygon: {self.roi_polygon}")
        if self.show:
            self.get_logger().info("[DECISION] mouse: move=좌표 표시, left click=좌표 로그, 두 번 클릭=ROI 후보 출력")

    def mouse_cb(self, event, x, y, flags, param):
        self.mouse_xy = (int(x), int(y))

        if event != cv2.EVENT_LBUTTONDOWN:
            return

        point = (int(x), int(y))
        self.roi_clicks.append(point)
        if len(self.roi_clicks) > 2:
            self.roi_clicks = [point]

        self.get_logger().info(f"[DECISION] mouse click: x={point[0]}, y={point[1]}")

        if len(self.roi_clicks) == 2:
            (x1, y1), (x2, y2) = self.roi_clicks
            left = min(x1, x2)
            top = min(y1, y2)
            right = max(x1, x2)
            bottom = max(y1, y2)
            self.get_logger().info(
                f"[DECISION] ROI candidate: --roi {left} {top} {right} {bottom}"
            )

    def image_cb(self, msg):
        self.frame_idx += 1
        if self.frame_idx % self.process_every_n != 0:
            return

        t0 = time.perf_counter()

        try:
            image_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"[DECISION] image 변환 실패: {e}")
            return

        if self.method == "yolo":
            total_count, class_counts, detections, _, active_roi, active_polygon = count_blocks_yolo(
                model=self.model,
                image_bgr=image_bgr,
                conf=self.conf,
                iou=self.iou,
                device=self.device,
                roi=self.roi,
                roi_polygon=self.roi_polygon
            )
        else:
            total_count, class_counts, detections, _, active_roi, active_polygon = count_blocks_cv(
                image_bgr=image_bgr,
                roi=self.roi,
                roi_polygon=self.roi_polygon,
                min_area=self.cv_min_area,
                max_area=self.cv_max_area,
                sat_min=self.cv_sat_min,
                edge_low=self.cv_edge_low,
                edge_high=self.cv_edge_high,
                morph_kernel=self.cv_morph_kernel,
                close_iterations=self.cv_close_iterations,
                open_iterations=self.cv_open_iterations,
            )

        annotated_bgr = draw_annotated(
            image_bgr=image_bgr,
            detections=detections,
            total_count=total_count,
            roi=active_roi,
            roi_polygon=active_polygon
        )
        if self.show:
            self.draw_mouse_overlay(annotated_bgr)

        self.publish_results(
            total_count=total_count,
            class_counts=class_counts,
            detections=detections,
            method=self.method,
            roi=active_roi,
            roi_polygon=active_polygon,
            annotated_bgr=annotated_bgr,
            stamp=msg.header.stamp,
            frame_id=msg.header.frame_id
        )

        if self.save_dir:
            path = os.path.join(self.save_dir, f"decision_{self.frame_idx:06d}.jpg")
            cv2.imwrite(path, annotated_bgr)

        if self.show:
            cv2.imshow(WINDOW_NAME, annotated_bgr)
            cv2.waitKey(1)

        elapsed = time.perf_counter() - t0
        now = time.monotonic()
        if now - self.last_log_time >= self.log_period_sec:
            self.last_log_time = now
            self.get_logger().info(
                f"[DECISION] total_blocks={total_count}, "
                f"classes={dict(class_counts)}, roi={active_roi}, polygon={active_polygon}, elapsed={elapsed:.3f}s"
            )

    def draw_mouse_overlay(self, image_bgr):
        if self.mouse_xy is not None:
            x, y = self.mouse_xy
            label = f"x={x}, y={y}"
            cv2.circle(image_bgr, (x, y), 4, (0, 0, 255), -1)
            cv2.putText(
                image_bgr,
                label,
                (max(0, x + 8), max(18, y - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 0, 255),
                2,
                cv2.LINE_AA
            )

        if len(self.roi_clicks) == 1:
            x, y = self.roi_clicks[0]
            cv2.circle(image_bgr, (x, y), 5, (0, 255, 255), -1)
        elif len(self.roi_clicks) == 2:
            (x1, y1), (x2, y2) = self.roi_clicks
            cv2.rectangle(
                image_bgr,
                (min(x1, x2), min(y1, y2)),
                (max(x1, x2), max(y1, y2)),
                (0, 255, 255),
                2
            )

    def publish_results(self, total_count, class_counts, detections, method, roi, roi_polygon, annotated_bgr, stamp, frame_id):
        count_msg = Int32()
        count_msg.data = int(total_count)
        self.count_pub.publish(count_msg)

        summary = {
            "total_blocks": int(total_count),
            "method": method,
            "classes": dict(class_counts),
            "detections": detections,
            "roi_xyxy": list(roi) if roi is not None else None,
            "roi_polygon_xy": [list(point) for point in roi_polygon] if roi_polygon is not None else None,
            "stamp": {
                "sec": int(stamp.sec),
                "nanosec": int(stamp.nanosec),
            },
            "frame_id": frame_id,
        }

        summary_msg = String()
        summary_msg.data = json.dumps(summary, ensure_ascii=False)
        self.summary_pub.publish(summary_msg)

        annotated_msg = self.bridge.cv2_to_imgmsg(annotated_bgr, encoding="bgr8")
        annotated_msg.header.stamp = stamp
        annotated_msg.header.frame_id = frame_id
        self.annotated_pub.publish(annotated_msg)

    def destroy_node(self):
        if self.show:
            cv2.destroyAllWindows()
        super().destroy_node()


def build_arg_parser():
    parser = argparse.ArgumentParser(
        description="realsense-ros image 토픽을 구독해 바닥 블럭 개수를 판단합니다."
    )
    parser.add_argument("--image-topic", default=DEFAULT_IMAGE_TOPIC)
    parser.add_argument("--annotated-topic", default=DEFAULT_ANNOTATED_TOPIC)
    parser.add_argument("--count-topic", default=DEFAULT_COUNT_TOPIC)
    parser.add_argument("--summary-topic", default=DEFAULT_SUMMARY_TOPIC)
    parser.add_argument(
        "--method",
        choices=("cv", "yolo"),
        default="cv",
        help="판단 방식. 기본값 cv는 pt 파일 없이 ROI 내부 덩어리를 카운트합니다."
    )
    parser.add_argument("--conf", type=float, default=0.5)
    parser.add_argument("--iou", type=float, default=0.45)
    parser.add_argument("--device", default="0", help="Ultralytics device 값. 예: 0, cpu")
    parser.add_argument("--cv-min-area", type=float, default=900.0, help="CV blob 최소 contour 면적")
    parser.add_argument("--cv-max-area", type=float, default=0.0, help="CV blob 최대 contour 면적. 0이면 제한 없음")
    parser.add_argument("--cv-sat-min", type=int, default=35, help="CV 색상 채도 foreground 기준")
    parser.add_argument("--cv-edge-low", type=int, default=50, help="CV Canny edge low threshold")
    parser.add_argument("--cv-edge-high", type=int, default=150, help="CV Canny edge high threshold")
    parser.add_argument("--cv-morph-kernel", type=int, default=7, help="CV morphology kernel 크기")
    parser.add_argument("--cv-close-iterations", type=int, default=2, help="CV close 반복 횟수")
    parser.add_argument("--cv-open-iterations", type=int, default=1, help="CV open 반복 횟수")
    parser.add_argument("--process-every-n", type=int, default=1)
    parser.add_argument(
        "--roi",
        nargs=4,
        type=float,
        metavar=("X1", "Y1", "X2", "Y2"),
        default=None,
        help="픽셀 기준 사각형 관심영역. 지정하면 이 영역만 판단합니다."
    )
    parser.add_argument(
        "--roi-polygon",
        nargs="+",
        type=float,
        metavar="XY",
        default=DEFAULT_ROI_POLYGON,
        help="픽셀 기준 다각형 관심영역. 예: --roi-polygon x1 y1 x2 y2 x3 y3 x4 y4"
    )
    parser.add_argument("--log-period-sec", type=float, default=1.0)
    parser.add_argument("--show", action="store_true", default=True, help="OpenCV 창으로 annotated image 표시")
    parser.add_argument("--save-dir", default=None, help="annotated image를 저장할 디렉터리")
    return parser


def main(args=None):
    parser = build_arg_parser()
    parsed, ros_args = parser.parse_known_args(args=args)

    rclpy.init(args=ros_args)
    node = DecisionAssemblyNode(parsed)

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
