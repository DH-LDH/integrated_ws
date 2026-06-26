import argparse
import json
import os
import time
from collections import Counter, deque

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Int32, String


# ============================================================
# 디버깅용 기본 변수
# 여기 값만 바꾸면 argparse 기본값도 같이 바뀜.
# 단, 실행 명령어에서 같은 옵션을 직접 넣으면 명령어 값이 우선.
# ============================================================

PACKAGE_NAME = "vision_assembly_pkg"

DEBUG_IMAGE_TOPIC = "/camera/camera/color/image_raw"
DEBUG_DEPTH_TOPIC = "/camera/camera/aligned_depth_to_color/image_raw"
DEBUG_ANNOTATED_TOPIC = "/decision_assembly/annotated_image"
DEBUG_COUNT_TOPIC = "/decision_assembly/block_count"
DEBUG_SUMMARY_TOPIC = "/decision_assembly/summary"

# ROI polygon: 좌상 → 우상 → 우하 → 좌하 순서
DEBUG_ROI_POLYGON = [234, 0, 478, 0, 640, 480, 90, 480]

# CV blob 필터
DEBUG_CV_MIN_AREA = 600.0

# 위치별 블럭 1개 기준 면적 대비 이 비율보다 작으면 잡음으로 본다.
DEBUG_CV_MIN_AREA_RATIO = 0.25

# 0이면 고정 최대값 제한 비활성화
# 이 값은 "절대 이 이상은 무조건 버림" 용도
DEBUG_CV_FIXED_MAX_AREA = 0.0

# y축 위치별 "블럭 1개 기준 면적"
# y=0 부근   → 기준 면적 900
# y=480 부근 → 기준 면적 5000
DEBUG_CV_AREA_Y_TOP = 0.0
DEBUG_CV_AREA_Y_BOTTOM = 480.0
DEBUG_CV_SINGLE_AREA_TOP = 900.0
DEBUG_CV_SINGLE_AREA_BOTTOM = 5000.0

# 큰 blob을 몇 개까지 추정할지
DEBUG_CV_MAX_ESTIMATED_COUNT = 4

# area / single_area가 이 값 이상일 때만 2개 이상으로 판단
DEBUG_CV_MULTI_COUNT_RATIO = 1.45

# HSV / edge / morphology
DEBUG_CV_SAT_MIN = 35
DEBUG_CV_USE_EDGE = True
DEBUG_CV_EDGE_LOW = 45
DEBUG_CV_EDGE_HIGH = 135
DEBUG_CV_MORPH_KERNEL = 3
DEBUG_CV_CLOSE_ITERATIONS = 0
DEBUG_CV_OPEN_ITERATIONS = 1

# Depth 사용 설정
DEBUG_USE_DEPTH = True
DEBUG_DEPTH_16UC1_SCALE = 0.001     # 16UC1 depth를 meter로 변환. RealSense 보통 1mm=0.001m
DEBUG_DEPTH_MIN_VALID_M = 0.10
DEBUG_DEPTH_MAX_VALID_M = 2.50
DEBUG_DEPTH_FLOOR_PERCENTILE = 80.0 # 같은 y-row에서 큰 depth 쪽을 바닥으로 추정
DEBUG_DEPTH_OBJECT_MARGIN_M = 0.01 # 바닥보다 이만큼 카메라 쪽에 가까우면 물체로 판단
DEBUG_DEPTH_ROW_MIN_VALID_PIXELS = 20
DEBUG_DEPTH_ROW_SMOOTH_KERNEL = 31
DEBUG_DEPTH_MASK_OPEN_ITERATIONS = 1
DEBUG_DEPTH_MASK_CLOSE_ITERATIONS = 1
DEBUG_DEPTH_RESIZE_IF_NEEDED = True

# depth와 RGB 마스크 결합 방식
# filter: RGB blob을 기본 검출로 쓰고, depth가 충분히 신뢰될 때만 평평한 노이즈를 제거.
# and   : colored blob이면서 바닥보다 튀어나온 것만 검출. 강하지만 먼 물체를 놓칠 수 있음.
# depth : depth만으로 검출.
# or    : RGB 또는 depth 중 하나라도 잡히면 검출. 민감하지만 오검출 가능.
DEBUG_DEPTH_COMBINE_MODE = "filter"
DEBUG_DEPTH_FILTER_MIN_VALID_RATIO = 0.20
DEBUG_DEPTH_FILTER_MIN_OBJECT_RATIO = 0.08

# ROI 경계 내부 여백 (픽셀)
# 검출 영역을 ROI 경계에서 이 픽셀만큼 안쪽으로 축소한다.
# Canny 가 ROI 폴리곤 경계를 엣지로 잡는 아티팩트를 차단한다.
DEBUG_ROI_INNER_MARGIN = 8

# 붙은 blob 분리
DEBUG_CV_WATERSHED = True
DEBUG_CV_WATERSHED_DIST_RATIO = 0.42
DEBUG_CV_WATERSHED_MAX_PARTS = 4
DEBUG_CV_SPLIT_MIN_AREA_RATIO = 0.70

# YOLO 기본값
DEBUG_YOLO_CONF = 0.5
DEBUG_YOLO_IOU = 0.45
DEBUG_YOLO_DEVICE = "0"

# 카운트 안정화 rolling 윈도우 크기 (1=비활성화)
# 최근 N 프레임의 중앙값을 발행해 모서리 노이즈로 인한 ±1 깜빡임을 제거한다.
DEBUG_COUNT_SMOOTH_N = 5

# 실행 / 표시
DEBUG_METHOD = "cv"
DEBUG_PROCESS_EVERY_N = 1
DEBUG_LOG_PERIOD_SEC = 1.0
DEBUG_SHOW = True
DEBUG_SAVE_DIR = None

WINDOW_NAME = "decision_assembly"

BOX_COLORS = [
    (0, 255, 255),
    (0, 180, 255),
    (80, 220, 80),
    (255, 160, 80),
    (220, 120, 255),
    (255, 220, 80),
]


# ============================================================
# 공통 유틸
# ============================================================

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
        depth=1,
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
        clamped.append(
            (
                max(0, min(width - 1, int(x))),
                max(0, min(height - 1, int(y))),
            )
        )

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
            dtype=np.int32,
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
        dtype=np.int32,
    )
    cv2.fillPoly(mask, [shifted], 255)

    return mask


def interpolate_by_y(y, top_y, bottom_y, top_value, bottom_value):
    if bottom_y <= top_y:
        return float(bottom_value)

    t = (float(y) - float(top_y)) / float(bottom_y - top_y)
    t = max(0.0, min(1.0, t))

    return float(top_value) + (float(bottom_value) - float(top_value)) * t


# ============================================================
# Depth 처리
# ============================================================

def depth_image_to_meters(depth_image, scale=DEBUG_DEPTH_16UC1_SCALE):
    """
    sensor_msgs/Image -> cv_bridge 변환 결과를 meter 단위 float32 depth로 변환.
    RealSense aligned depth는 보통 16UC1, 단위 mm라 scale=0.001 사용.
    """
    if depth_image is None:
        return None

    if depth_image.dtype == np.uint16:
        depth_m = depth_image.astype(np.float32) * float(scale)
    elif depth_image.dtype in (np.float32, np.float64):
        depth_m = depth_image.astype(np.float32)
    else:
        depth_m = depth_image.astype(np.float32) * float(scale)

    depth_m[~np.isfinite(depth_m)] = 0.0
    return depth_m


def resize_depth_if_needed(depth_m, target_shape, resize_if_needed=True):
    if depth_m is None:
        return None

    target_h, target_w = target_shape[:2]
    if depth_m.shape[:2] == (target_h, target_w):
        return depth_m

    if not resize_if_needed:
        return None

    return cv2.resize(depth_m, (target_w, target_h), interpolation=cv2.INTER_NEAREST)


def smooth_1d_nan(values, kernel_size):
    values = np.asarray(values, dtype=np.float32)
    valid = np.isfinite(values)

    if not np.any(valid):
        return values

    x = np.arange(len(values), dtype=np.float32)
    filled = values.copy()
    filled[~valid] = np.interp(x[~valid], x[valid], values[valid])

    k = max(1, int(kernel_size))
    if k % 2 == 0:
        k += 1

    if k > 1:
        filled = cv2.GaussianBlur(filled.reshape(-1, 1), (1, k), 0).reshape(-1)

    return filled


def make_depth_object_mask(
    depth_m,
    roi_mask,
    min_valid_m=DEBUG_DEPTH_MIN_VALID_M,
    max_valid_m=DEBUG_DEPTH_MAX_VALID_M,
    floor_percentile=DEBUG_DEPTH_FLOOR_PERCENTILE,
    object_margin_m=DEBUG_DEPTH_OBJECT_MARGIN_M,
    row_min_valid_pixels=DEBUG_DEPTH_ROW_MIN_VALID_PIXELS,
    row_smooth_kernel=DEBUG_DEPTH_ROW_SMOOTH_KERNEL,
    open_iterations=DEBUG_DEPTH_MASK_OPEN_ITERATIONS,
    close_iterations=DEBUG_DEPTH_MASK_CLOSE_ITERATIONS,
    morph_kernel=DEBUG_CV_MORPH_KERNEL,
):
    """
    각 y-row마다 바닥 depth를 robust하게 추정한 뒤,
    바닥보다 카메라에 더 가까운 픽셀을 물체로 판단한다.

    카메라가 비스듬히 보면 바닥 depth가 y마다 달라지므로 row별 floor profile을 만든다.
    """
    if depth_m is None:
        return None, None

    depth = depth_m.astype(np.float32)
    roi_valid = roi_mask > 0
    valid = (
        roi_valid
        & np.isfinite(depth)
        & (depth >= float(min_valid_m))
        & (depth <= float(max_valid_m))
    )

    h, w = depth.shape[:2]
    floor_profile = np.full(h, np.nan, dtype=np.float32)

    for yy in range(h):
        row_vals = depth[yy, valid[yy]]
        if row_vals.size >= int(row_min_valid_pixels):
            floor_profile[yy] = np.percentile(row_vals, float(floor_percentile))

    floor_profile = smooth_1d_nan(floor_profile, row_smooth_kernel)

    if not np.any(np.isfinite(floor_profile)):
        return None, None

    floor_img = floor_profile.reshape(h, 1)
    object_mask_bool = valid & (depth < (floor_img - float(object_margin_m)))
    object_mask = np.zeros((h, w), dtype=np.uint8)
    object_mask[object_mask_bool] = 255

    kernel_size = max(1, int(morph_kernel))
    if kernel_size % 2 == 0:
        kernel_size += 1
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))

    if open_iterations > 0:
        object_mask = cv2.morphologyEx(object_mask, cv2.MORPH_OPEN, kernel, iterations=int(open_iterations))
    if close_iterations > 0:
        object_mask = cv2.morphologyEx(object_mask, cv2.MORPH_CLOSE, kernel, iterations=int(close_iterations))

    object_mask = cv2.bitwise_and(object_mask, roi_mask)

    debug_info = {
        "depth_valid_ratio": float(np.count_nonzero(valid) / max(1, np.count_nonzero(roi_valid))),
        "floor_profile_valid_rows": int(np.count_nonzero(np.isfinite(floor_profile))),
        "object_pixels": int(np.count_nonzero(object_mask)),
    }

    return object_mask, debug_info


# ============================================================
# 검출 / 카운트
# ============================================================

def split_touching_contour_watershed(
    crop_bgr,
    contour,
    min_part_area=350.0,
    dist_ratio=0.42,
    max_parts=4,
):
    x, y, w, h = cv2.boundingRect(contour)

    pad = 6
    x0 = max(0, x - pad)
    y0 = max(0, y - pad)
    x1 = min(crop_bgr.shape[1], x + w + pad)
    y1 = min(crop_bgr.shape[0], y + h + pad)

    local_w = x1 - x0
    local_h = y1 - y0

    if local_w <= 3 or local_h <= 3:
        return [contour]

    local_mask = np.zeros((local_h, local_w), dtype=np.uint8)

    shifted_contour = contour.copy()
    shifted_contour[:, 0, 0] -= x0
    shifted_contour[:, 0, 1] -= y0

    cv2.drawContours(local_mask, [shifted_contour], -1, 255, -1)

    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    local_mask = cv2.morphologyEx(local_mask, cv2.MORPH_CLOSE, kernel, iterations=1)

    dist = cv2.distanceTransform(local_mask, cv2.DIST_L2, 5)

    if dist.max() <= 0:
        return [contour]

    _, sure_fg = cv2.threshold(
        dist,
        float(dist_ratio) * dist.max(),
        255,
        cv2.THRESH_BINARY,
    )
    sure_fg = np.uint8(sure_fg)

    num_labels, markers = cv2.connectedComponents(sure_fg)

    if num_labels <= 2:
        return [contour]

    sure_bg = cv2.dilate(local_mask, kernel, iterations=2)
    unknown = cv2.subtract(sure_bg, sure_fg)

    markers = markers + 1
    markers[unknown == 255] = 0

    local_bgr = crop_bgr[y0:y1, x0:x1].copy()
    markers = cv2.watershed(local_bgr, markers)

    split_contours = []

    for marker_id in range(2, markers.max() + 1):
        part_mask = np.zeros(local_mask.shape, dtype=np.uint8)
        part_mask[markers == marker_id] = 255
        part_mask = cv2.bitwise_and(part_mask, local_mask)

        part_contours, _ = cv2.findContours(
            part_mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )

        if not part_contours:
            continue

        part_contour = max(part_contours, key=cv2.contourArea)
        part_area = float(cv2.contourArea(part_contour))

        if part_area < float(min_part_area):
            continue

        part_contour[:, 0, 0] += x0
        part_contour[:, 0, 1] += y0

        split_contours.append(part_contour)

    if 2 <= len(split_contours) <= int(max_parts):
        return split_contours

    return [contour]


def count_blocks_yolo(model, image_bgr, conf, iou, device, roi=None, roi_polygon=None):
    inference_bgr, offset_x, offset_y, active_roi, active_polygon = crop_for_inference(
        image_bgr=image_bgr,
        roi=roi,
        roi_polygon=roi_polygon,
    )

    results = model(
        inference_bgr,
        conf=conf,
        iou=iou,
        device=device,
        verbose=False,
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

        detections.append(
            {
                "class_id": int(cls_id),
                "class_name": str(names[int(cls_id)]),
                "confidence": float(score),
                "estimated_count": 1,
                "bbox_xyxy": [
                    int(round(x1 + offset_x)),
                    int(round(y1 + offset_y)),
                    int(round(x2 + offset_x)),
                    int(round(y2 + offset_y)),
                ],
            }
        )

    return int(len(class_ids)), class_counts, detections, results, active_roi, active_polygon


def estimate_blob_count(area, single_area, max_count, multi_count_ratio=1.45):
    if single_area <= 0:
        return 1

    ratio = float(area) / float(single_area)

    if ratio < float(multi_count_ratio):
        return 1

    estimated = int(round(ratio))
    estimated = max(2, estimated)
    estimated = min(int(max_count), estimated)

    return estimated


def contour_depth_stats(contour, depth_valid_mask, depth_object_mask):
    if depth_valid_mask is None or depth_object_mask is None:
        return None

    x, y, w, h = cv2.boundingRect(contour)
    if w <= 0 or h <= 0:
        return None

    local_mask = np.zeros((h, w), dtype=np.uint8)
    shifted = contour.copy()
    shifted[:, 0, 0] -= x
    shifted[:, 0, 1] -= y
    cv2.drawContours(local_mask, [shifted], -1, 255, -1)

    contour_pixels = int(np.count_nonzero(local_mask))
    if contour_pixels <= 0:
        return None

    valid_crop = depth_valid_mask[y:y + h, x:x + w]
    object_crop = depth_object_mask[y:y + h, x:x + w]

    valid_pixels = int(np.count_nonzero((valid_crop > 0) & (local_mask > 0)))
    object_pixels = int(np.count_nonzero((object_crop > 0) & (local_mask > 0)))

    valid_ratio = float(valid_pixels / max(1, contour_pixels))
    object_ratio = float(object_pixels / max(1, valid_pixels))

    return {
        "contour_pixels": contour_pixels,
        "valid_pixels": valid_pixels,
        "object_pixels": object_pixels,
        "valid_ratio": valid_ratio,
        "object_ratio": object_ratio,
    }


def detection_quality(det):
    area = float(det.get("count_area", det.get("area", 0.0)))
    single_area = max(1.0, float(det.get("single_area", 1.0)))
    return (
        float(area / single_area),
        float(det.get("area", 0.0)),
    )


def limit_detections_to_count(detections, target_count):
    """단발성 노이즈 때문에 raw 검출이 더 많을 때 약한 후보부터 제외한다."""
    target_count = max(0, int(target_count))
    raw_count = sum(int(det.get("estimated_count", 1)) for det in detections)
    if target_count <= 0:
        return []
    if raw_count <= target_count:
        return detections

    ranked = sorted(detections, key=detection_quality, reverse=True)
    kept = []
    used = 0
    for det in ranked:
        est = max(1, int(det.get("estimated_count", 1)))
        remaining = target_count - used
        if remaining <= 0:
            break

        if est <= remaining:
            kept.append(det)
            used += est
            continue

        clipped = dict(det)
        clipped["estimated_count"] = remaining
        kept.append(clipped)
        used += remaining

    return kept


def count_blocks_cv(
    image_bgr,
    depth_m=None,
    roi=None,
    roi_polygon=None,
    min_area=DEBUG_CV_MIN_AREA,
    min_area_ratio=DEBUG_CV_MIN_AREA_RATIO,
    fixed_max_area=DEBUG_CV_FIXED_MAX_AREA,
    area_y_top=DEBUG_CV_AREA_Y_TOP,
    area_y_bottom=DEBUG_CV_AREA_Y_BOTTOM,
    single_area_top=DEBUG_CV_SINGLE_AREA_TOP,
    single_area_bottom=DEBUG_CV_SINGLE_AREA_BOTTOM,
    max_estimated_count=DEBUG_CV_MAX_ESTIMATED_COUNT,
    multi_count_ratio=DEBUG_CV_MULTI_COUNT_RATIO,
    sat_min=DEBUG_CV_SAT_MIN,
    use_edge=DEBUG_CV_USE_EDGE,
    edge_low=DEBUG_CV_EDGE_LOW,
    edge_high=DEBUG_CV_EDGE_HIGH,
    morph_kernel=DEBUG_CV_MORPH_KERNEL,
    close_iterations=DEBUG_CV_CLOSE_ITERATIONS,
    open_iterations=DEBUG_CV_OPEN_ITERATIONS,
    use_depth=DEBUG_USE_DEPTH,
    depth_combine_mode=DEBUG_DEPTH_COMBINE_MODE,
    depth_filter_min_valid_ratio=DEBUG_DEPTH_FILTER_MIN_VALID_RATIO,
    depth_filter_min_object_ratio=DEBUG_DEPTH_FILTER_MIN_OBJECT_RATIO,
    depth_min_valid_m=DEBUG_DEPTH_MIN_VALID_M,
    depth_max_valid_m=DEBUG_DEPTH_MAX_VALID_M,
    depth_floor_percentile=DEBUG_DEPTH_FLOOR_PERCENTILE,
    depth_object_margin_m=DEBUG_DEPTH_OBJECT_MARGIN_M,
    depth_row_min_valid_pixels=DEBUG_DEPTH_ROW_MIN_VALID_PIXELS,
    depth_row_smooth_kernel=DEBUG_DEPTH_ROW_SMOOTH_KERNEL,
    depth_mask_open_iterations=DEBUG_DEPTH_MASK_OPEN_ITERATIONS,
    depth_mask_close_iterations=DEBUG_DEPTH_MASK_CLOSE_ITERATIONS,
    watershed_enable=DEBUG_CV_WATERSHED,
    watershed_dist_ratio=DEBUG_CV_WATERSHED_DIST_RATIO,
    watershed_max_parts=DEBUG_CV_WATERSHED_MAX_PARTS,
    split_min_area_ratio=DEBUG_CV_SPLIT_MIN_AREA_RATIO,
    roi_inner_margin=DEBUG_ROI_INNER_MARGIN,
):
    crop_bgr, offset_x, offset_y, active_roi, active_polygon = crop_for_inference(
        image_bgr=image_bgr,
        roi=roi,
        roi_polygon=roi_polygon,
    )

    roi_mask = make_crop_mask(crop_bgr.shape, offset_x, offset_y, active_polygon)

    # ROI 경계 여백 마스크: Canny 가 폴리곤 경계를 엣지로 잡는 것을 차단.
    # roi_mask 를 roi_inner_margin 픽셀 침식해 경계 안쪽만 검출 영역으로 사용한다.
    if int(roi_inner_margin) > 0:
        _m = int(roi_inner_margin)
        _inner_k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (_m * 2 + 1, _m * 2 + 1))
        roi_mask_inner = cv2.erode(roi_mask, _inner_k, iterations=1)
    else:
        roi_mask_inner = roi_mask

    hsv = cv2.cvtColor(crop_bgr, cv2.COLOR_BGR2HSV)
    saturation = hsv[:, :, 1]
    sat_mask = cv2.inRange(saturation, int(sat_min), 255)
    rgb_foreground = cv2.bitwise_and(sat_mask, roi_mask_inner)

    if use_edge:
        gray = cv2.cvtColor(crop_bgr, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (5, 5), 0)
        edge_mask = cv2.Canny(gray, int(edge_low), int(edge_high))
        # Canny 경계 아티팩트 차단: edge_mask 에도 축소 마스크 적용
        edge_mask = cv2.bitwise_and(edge_mask, roi_mask_inner)
        rgb_foreground = cv2.bitwise_or(rgb_foreground, edge_mask)
        rgb_foreground = cv2.bitwise_and(rgb_foreground, roi_mask_inner)

    depth_debug = None
    depth_used = False
    depth_object_mask = None
    depth_valid_mask = None
    depth_mode = str(depth_combine_mode).lower().strip()

    if use_depth and depth_m is not None:
        crop_depth = depth_m[offset_y:offset_y + crop_bgr.shape[0], offset_x:offset_x + crop_bgr.shape[1]]
        depth_valid_mask = np.zeros(crop_depth.shape[:2], dtype=np.uint8)
        depth_valid = (
            (roi_mask > 0)
            & np.isfinite(crop_depth)
            & (crop_depth >= float(depth_min_valid_m))
            & (crop_depth <= float(depth_max_valid_m))
        )
        depth_valid_mask[depth_valid] = 255

        depth_object_mask, depth_debug = make_depth_object_mask(
            depth_m=crop_depth,
            roi_mask=roi_mask,
            min_valid_m=depth_min_valid_m,
            max_valid_m=depth_max_valid_m,
            floor_percentile=depth_floor_percentile,
            object_margin_m=depth_object_margin_m,
            row_min_valid_pixels=depth_row_min_valid_pixels,
            row_smooth_kernel=depth_row_smooth_kernel,
            open_iterations=depth_mask_open_iterations,
            close_iterations=depth_mask_close_iterations,
            morph_kernel=morph_kernel,
        )

        if depth_object_mask is not None:
            depth_used = True
            if depth_mode == "depth":
                foreground = depth_object_mask
            elif depth_mode == "or":
                foreground = cv2.bitwise_or(rgb_foreground, depth_object_mask)
            elif depth_mode == "and":
                foreground = cv2.bitwise_and(rgb_foreground, depth_object_mask)
            else:
                foreground = rgb_foreground
        else:
            foreground = rgb_foreground
    else:
        foreground = rgb_foreground

    kernel_size = max(1, int(morph_kernel))
    if kernel_size % 2 == 0:
        kernel_size += 1

    kernel = cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE,
        (kernel_size, kernel_size),
    )

    if close_iterations > 0:
        foreground = cv2.morphologyEx(
            foreground,
            cv2.MORPH_CLOSE,
            kernel,
            iterations=int(close_iterations),
        )

    if open_iterations > 0:
        foreground = cv2.morphologyEx(
            foreground,
            cv2.MORPH_OPEN,
            kernel,
            iterations=int(open_iterations),
        )

    # 모폴로지 이후 남아있을 수 있는 경계 픽셀 최종 제거
    foreground = cv2.bitwise_and(foreground, roi_mask_inner)

    contours, _ = cv2.findContours(
        foreground,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE,
    )

    detections = []
    rejected = []

    fixed_max_area = float(fixed_max_area)
    min_area = float(min_area)

    for contour in contours:
        parent_area = float(cv2.contourArea(contour))

        x, y, w, h = cv2.boundingRect(contour)
        if w <= 2 or h <= 2:
            continue

        center_y_parent = int(round(y + h / 2 + offset_y))

        if parent_area < min_area:
            rejected.append(
                {
                    "reason": "small",
                    "area": parent_area,
                    "single_area": 0.0,
                    "estimated_count": 0,
                    "center_y": int(center_y_parent),
                    "bbox_xyxy": [
                        int(x + offset_x),
                        int(y + offset_y),
                        int(x + w + offset_x),
                        int(y + h + offset_y),
                    ],
                }
            )
            continue

        if fixed_max_area > 0.0 and parent_area > fixed_max_area:
            rejected.append(
                {
                    "reason": "fixed_large",
                    "area": parent_area,
                    "single_area": 0.0,
                    "estimated_count": 0,
                    "center_y": int(center_y_parent),
                    "bbox_xyxy": [
                        int(x + offset_x),
                        int(y + offset_y),
                        int(x + w + offset_x),
                        int(y + h + offset_y),
                    ],
                }
            )
            continue

        if watershed_enable:
            split_contours = split_touching_contour_watershed(
                crop_bgr=crop_bgr,
                contour=contour,
                min_part_area=max(250.0, min_area * float(split_min_area_ratio)),
                dist_ratio=float(watershed_dist_ratio),
                max_parts=int(watershed_max_parts),
            )
        else:
            split_contours = [contour]

        for sub_contour in split_contours:
            sub_area = float(cv2.contourArea(sub_contour))

            sx, sy, sw, sh = cv2.boundingRect(sub_contour)
            if sw <= 2 or sh <= 2:
                continue

            center_y_img = int(round(sy + sh / 2 + offset_y))

            single_area = interpolate_by_y(
                y=center_y_img,
                top_y=area_y_top,
                bottom_y=area_y_bottom,
                top_value=single_area_top,
                bottom_value=single_area_bottom,
            )

            if sub_area < min_area:
                rejected.append(
                    {
                        "reason": "small",
                        "area": sub_area,
                        "single_area": float(single_area),
                        "estimated_count": 0,
                        "center_y": int(center_y_img),
                        "bbox_xyxy": [
                            int(sx + offset_x),
                            int(sy + offset_y),
                            int(sx + sw + offset_x),
                            int(sy + sh + offset_y),
                        ],
                    }
                )
                continue

            rgb_area_ratio = float(sub_area / single_area) if single_area > 0 else 0.0
            if rgb_area_ratio < float(min_area_ratio):
                rejected.append(
                    {
                        "reason": "small_ratio",
                        "area": sub_area,
                        "single_area": float(single_area),
                        "estimated_count": 0,
                        "center_y": int(center_y_img),
                        "bbox_xyxy": [
                            int(sx + offset_x),
                            int(sy + offset_y),
                            int(sx + sw + offset_x),
                            int(sy + sh + offset_y),
                        ],
                    }
                )
                continue

            if fixed_max_area > 0.0 and sub_area > fixed_max_area:
                rejected.append(
                    {
                        "reason": "fixed_large",
                        "area": sub_area,
                        "single_area": float(single_area),
                        "estimated_count": 0,
                        "center_y": int(center_y_img),
                        "bbox_xyxy": [
                            int(sx + offset_x),
                            int(sy + offset_y),
                            int(sx + sw + offset_x),
                            int(sy + sh + offset_y),
                        ],
                    }
                )
                continue

            depth_stats = None
            count_area = sub_area
            count_area_source = "rgb"
            if depth_used and depth_mode == "filter":
                depth_stats = contour_depth_stats(
                    sub_contour,
                    depth_valid_mask=depth_valid_mask,
                    depth_object_mask=depth_object_mask,
                )
                if depth_stats is not None:
                    has_reliable_depth = (
                        depth_stats["valid_ratio"] >= float(depth_filter_min_valid_ratio)
                    )
                    is_flat_on_floor = (
                        depth_stats["object_ratio"] < float(depth_filter_min_object_ratio)
                    )
                    if has_reliable_depth and is_flat_on_floor:
                        rejected.append(
                            {
                                "reason": "depth_flat",
                                "area": sub_area,
                                "single_area": float(single_area),
                                "estimated_count": 0,
                                "center_y": int(center_y_img),
                                "depth": depth_stats,
                                "bbox_xyxy": [
                                    int(sx + offset_x),
                                    int(sy + offset_y),
                                    int(sx + sw + offset_x),
                                    int(sy + sh + offset_y),
                                ],
                            }
                        )
                        continue

                    if has_reliable_depth:
                        count_area = sub_area * max(
                            float(depth_filter_min_object_ratio),
                            min(1.0, float(depth_stats["object_ratio"])),
                        )
                        count_area = min(float(sub_area), float(count_area))
                        count_area_source = "depth_object_ratio"

            count_area_ratio = float(count_area / single_area) if single_area > 0 else 0.0
            if count_area_ratio < float(min_area_ratio):
                rejected.append(
                    {
                        "reason": "depth_small_ratio" if count_area_source == "depth_object_ratio" else "small_ratio",
                        "area": count_area,
                        "single_area": float(single_area),
                        "estimated_count": 0,
                        "center_y": int(center_y_img),
                        "depth": depth_stats,
                        "bbox_xyxy": [
                            int(sx + offset_x),
                            int(sy + offset_y),
                            int(sx + sw + offset_x),
                            int(sy + sh + offset_y),
                        ],
                    }
                )
                continue

            estimated_count = estimate_blob_count(
                area=count_area,
                single_area=single_area,
                max_count=max_estimated_count,
                multi_count_ratio=multi_count_ratio,
            )

            moments = cv2.moments(sub_contour)

            if moments["m00"] != 0:
                center_x = int(round(moments["m10"] / moments["m00"])) + offset_x
                center_y = int(round(moments["m01"] / moments["m00"])) + offset_y
            else:
                center_x = int(round(sx + sw / 2)) + offset_x
                center_y = int(round(sy + sh / 2)) + offset_y

            detections.append(
                {
                    "class_id": 0,
                    "class_name": "blob",
                    "confidence": 1.0,
                    "area": sub_area,
                    "count_area": float(count_area),
                    "count_area_source": count_area_source,
                    "single_area": float(single_area),
                    "area_ratio": count_area_ratio,
                    "rgb_area_ratio": rgb_area_ratio,
                    "estimated_count": int(estimated_count),
                    "parent_area": parent_area,
                    "split_count": len(split_contours),
                    "depth_used": bool(depth_used),
                    "depth": depth_stats,
                    "center_xy": [center_x, center_y],
                    "bbox_xyxy": [
                        int(sx + offset_x),
                        int(sy + offset_y),
                        int(sx + sw + offset_x),
                        int(sy + sh + offset_y),
                    ],
                }
            )

    detections.sort(key=lambda det: det["area"], reverse=True)
    rejected.sort(key=lambda det: det["area"], reverse=True)

    total_count = sum(int(det.get("estimated_count", 1)) for det in detections)
    class_counts = Counter({"blob": total_count}) if detections else Counter()

    return total_count, class_counts, detections, rejected, foreground, active_roi, active_polygon, depth_debug


def draw_annotated(image_bgr, detections, total_count, roi=None, roi_polygon=None, rejected=None, depth_used=False):
    annotated = image_bgr.copy()

    if roi_polygon is not None:
        points = np.array(roi_polygon, dtype=np.int32)

        cv2.polylines(
            annotated,
            [points],
            isClosed=True,
            color=(255, 255, 0),
            thickness=2,
        )

        x1, y1 = roi_polygon[0]
        cv2.putText(
            annotated,
            "ROI",
            (x1 + 6, max(22, y1 + 22)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 0),
            2,
            cv2.LINE_AA,
        )

    elif roi is not None:
        x1, y1, x2, y2 = roi

        cv2.rectangle(
            annotated,
            (x1, y1),
            (x2, y2),
            (255, 255, 0),
            2,
        )

        cv2.putText(
            annotated,
            "ROI",
            (x1 + 6, max(22, y1 + 22)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 0),
            2,
            cv2.LINE_AA,
        )

    for det in detections:
        x1, y1, x2, y2 = det["bbox_xyxy"]
        color = BOX_COLORS[det["class_id"] % len(BOX_COLORS)]

        area = float(det.get("count_area", det.get("area", 0.0)))
        single_area = float(det.get("single_area", 0.0))
        estimated_count = int(det.get("estimated_count", 1))

        if estimated_count > 1:
            label = f'blobx{estimated_count} {area:.0f}/{single_area:.0f}'
        else:
            label = f'blob {area:.0f}/{single_area:.0f}'

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
            cv2.LINE_AA,
        )

    if rejected:
        for rej in rejected:
            x1, y1, x2, y2 = rej["bbox_xyxy"]
            area = float(rej.get("area", 0.0))
            single_area = float(rej.get("single_area", 0.0))
            reason = str(rej.get("reason", "rej"))

            cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 0, 255), 1)
            label = f'{reason} {area:.0f}/{single_area:.0f}'
            cv2.putText(
                annotated,
                label,
                (x1, min(annotated.shape[0] - 5, y2 + 16)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                (0, 0, 255),
                1,
                cv2.LINE_AA,
            )

    cv2.putText(
        annotated,
        f"total_blocks={total_count}",
        (12, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (0, 255, 255),
        2,
        cv2.LINE_AA,
    )

    depth_text = "depth=on" if depth_used else "depth=off"
    cv2.putText(
        annotated,
        depth_text,
        (12, 60),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.65,
        (0, 255, 255) if depth_used else (0, 0, 255),
        2,
        cv2.LINE_AA,
    )

    return annotated


class DecisionAssemblyNode(Node):
    def __init__(self, args):
        super().__init__("decision_assembly")

        self.image_topic = args.image_topic
        self.depth_topic = args.depth_topic
        self.annotated_topic = args.annotated_topic
        self.count_topic = args.count_topic
        self.summary_topic = args.summary_topic

        self.method = args.method
        self.conf = args.conf
        self.iou = args.iou
        self.device = args.device

        self.cv_min_area = args.cv_min_area
        self.cv_min_area_ratio = args.cv_min_area_ratio
        self.cv_fixed_max_area = args.cv_fixed_max_area
        self.cv_area_y_top = args.cv_area_y_top
        self.cv_area_y_bottom = args.cv_area_y_bottom
        self.cv_single_area_top = args.cv_single_area_top
        self.cv_single_area_bottom = args.cv_single_area_bottom
        self.cv_max_estimated_count = args.cv_max_estimated_count
        self.cv_multi_count_ratio = args.cv_multi_count_ratio

        self.cv_sat_min = args.cv_sat_min
        self.cv_use_edge = args.cv_use_edge
        self.cv_edge_low = args.cv_edge_low
        self.cv_edge_high = args.cv_edge_high
        self.cv_morph_kernel = args.cv_morph_kernel
        self.cv_close_iterations = args.cv_close_iterations
        self.cv_open_iterations = args.cv_open_iterations

        self.use_depth = args.use_depth
        self.depth_16uc1_scale = args.depth_16uc1_scale
        self.depth_min_valid_m = args.depth_min_valid_m
        self.depth_max_valid_m = args.depth_max_valid_m
        self.depth_floor_percentile = args.depth_floor_percentile
        self.depth_object_margin_m = args.depth_object_margin_m
        self.depth_row_min_valid_pixels = args.depth_row_min_valid_pixels
        self.depth_row_smooth_kernel = args.depth_row_smooth_kernel
        self.depth_mask_open_iterations = args.depth_mask_open_iterations
        self.depth_mask_close_iterations = args.depth_mask_close_iterations
        self.depth_resize_if_needed = args.depth_resize_if_needed
        self.depth_combine_mode = args.depth_combine_mode
        self.depth_filter_min_valid_ratio = args.depth_filter_min_valid_ratio
        self.depth_filter_min_object_ratio = args.depth_filter_min_object_ratio

        self.cv_watershed = args.cv_watershed
        self.cv_watershed_dist_ratio = args.cv_watershed_dist_ratio
        self.cv_watershed_max_parts = args.cv_watershed_max_parts
        self.cv_split_min_area_ratio = args.cv_split_min_area_ratio
        self.roi_inner_margin = args.roi_inner_margin

        self.process_every_n = max(1, args.process_every_n)

        self.roi = parse_roi(args.roi)
        self.roi_polygon = parse_roi_polygon(args.roi_polygon)

        self.show = args.show
        self.save_dir = args.save_dir

        self.frame_idx = 0
        self.last_log_time = 0.0
        self.log_period_sec = max(0.0, args.log_period_sec)

        self.count_smooth_n = max(1, args.count_smooth_n)
        self.count_history: deque = deque(maxlen=self.count_smooth_n)

        self.mouse_xy = None
        self.roi_clicks = []
        self.latest_depth_m = None
        self.latest_depth_stamp = None

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

        self.image_sub = self.create_subscription(Image, self.image_topic, self.image_cb, make_sensor_qos())

        self.depth_sub = None
        if self.use_depth:
            self.depth_sub = self.create_subscription(Image, self.depth_topic, self.depth_cb, make_sensor_qos())

        self.get_logger().info(f"[DECISION] subscribe image: {self.image_topic}")
        if self.use_depth:
            self.get_logger().info(f"[DECISION] subscribe depth: {self.depth_topic}")
        self.get_logger().info(f"[DECISION] publish count: {self.count_topic}")
        self.get_logger().info(f"[DECISION] publish summary: {self.summary_topic}")
        self.get_logger().info(f"[DECISION] publish annotated: {self.annotated_topic}")
        self.get_logger().info(f"[DECISION] method: {self.method}")

        if self.method == "cv":
            self.get_logger().info(
                "[DECISION] cv params: "
                f"min_area={self.cv_min_area}, "
                f"min_area_ratio={self.cv_min_area_ratio}, "
                f"fixed_max_area={self.cv_fixed_max_area}, "
                f"area_y_top={self.cv_area_y_top}, "
                f"area_y_bottom={self.cv_area_y_bottom}, "
                f"single_area_top={self.cv_single_area_top}, "
                f"single_area_bottom={self.cv_single_area_bottom}, "
                f"max_estimated_count={self.cv_max_estimated_count}, "
                f"multi_count_ratio={self.cv_multi_count_ratio}, "
                f"sat_min={self.cv_sat_min}, "
                f"use_edge={self.cv_use_edge}, "
                f"edge=({self.cv_edge_low},{self.cv_edge_high}), "
                f"use_depth={self.use_depth}, "
                f"depth_mode={self.depth_combine_mode}, "
                f"depth_margin_m={self.depth_object_margin_m}, "
                f"depth_filter_valid={self.depth_filter_min_valid_ratio}, "
                f"depth_filter_object={self.depth_filter_min_object_ratio}, "
                f"morph={self.cv_morph_kernel}, "
                f"close={self.cv_close_iterations}, "
                f"open={self.cv_open_iterations}, "
                f"watershed={self.cv_watershed}, "
                f"dist_ratio={self.cv_watershed_dist_ratio}, "
                f"max_parts={self.cv_watershed_max_parts}"
            )

        if self.roi is None:
            self.get_logger().info("[DECISION] roi: disabled")
        else:
            self.get_logger().info(f"[DECISION] roi xyxy: {self.roi}")

        if self.roi_polygon is not None:
            self.get_logger().info(f"[DECISION] roi polygon: {self.roi_polygon}")

    def depth_cb(self, msg):
        try:
            depth_raw = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            self.latest_depth_m = depth_image_to_meters(depth_raw, scale=self.depth_16uc1_scale)
            self.latest_depth_stamp = msg.header.stamp
        except Exception as e:
            self.get_logger().warn(f"[DECISION] depth 변환 실패: {e}")

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

            self.get_logger().info(f"[DECISION] ROI candidate: --roi {left} {top} {right} {bottom}")

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

        depth_m = None
        if self.use_depth and self.latest_depth_m is not None:
            depth_m = resize_depth_if_needed(
                self.latest_depth_m,
                target_shape=image_bgr.shape,
                resize_if_needed=self.depth_resize_if_needed,
            )

        rejected = []
        depth_debug = None

        if self.method == "yolo":
            total_count, class_counts, detections, _, active_roi, active_polygon = count_blocks_yolo(
                model=self.model,
                image_bgr=image_bgr,
                conf=self.conf,
                iou=self.iou,
                device=self.device,
                roi=self.roi,
                roi_polygon=self.roi_polygon,
            )
            depth_used = False
        else:
            total_count, class_counts, detections, rejected, _, active_roi, active_polygon, depth_debug = count_blocks_cv(
                image_bgr=image_bgr,
                depth_m=depth_m,
                roi=self.roi,
                roi_polygon=self.roi_polygon,
                min_area=self.cv_min_area,
                min_area_ratio=self.cv_min_area_ratio,
                fixed_max_area=self.cv_fixed_max_area,
                area_y_top=self.cv_area_y_top,
                area_y_bottom=self.cv_area_y_bottom,
                single_area_top=self.cv_single_area_top,
                single_area_bottom=self.cv_single_area_bottom,
                max_estimated_count=self.cv_max_estimated_count,
                multi_count_ratio=self.cv_multi_count_ratio,
                sat_min=self.cv_sat_min,
                use_edge=self.cv_use_edge,
                edge_low=self.cv_edge_low,
                edge_high=self.cv_edge_high,
                morph_kernel=self.cv_morph_kernel,
                close_iterations=self.cv_close_iterations,
                open_iterations=self.cv_open_iterations,
                use_depth=self.use_depth,
                depth_combine_mode=self.depth_combine_mode,
                depth_filter_min_valid_ratio=self.depth_filter_min_valid_ratio,
                depth_filter_min_object_ratio=self.depth_filter_min_object_ratio,
                depth_min_valid_m=self.depth_min_valid_m,
                depth_max_valid_m=self.depth_max_valid_m,
                depth_floor_percentile=self.depth_floor_percentile,
                depth_object_margin_m=self.depth_object_margin_m,
                depth_row_min_valid_pixels=self.depth_row_min_valid_pixels,
                depth_row_smooth_kernel=self.depth_row_smooth_kernel,
                depth_mask_open_iterations=self.depth_mask_open_iterations,
                depth_mask_close_iterations=self.depth_mask_close_iterations,
                watershed_enable=self.cv_watershed,
                watershed_dist_ratio=self.cv_watershed_dist_ratio,
                watershed_max_parts=self.cv_watershed_max_parts,
                split_min_area_ratio=self.cv_split_min_area_ratio,
                roi_inner_margin=self.roi_inner_margin,
            )
            depth_used = bool(depth_debug is not None)

        # rolling median으로 단발성 노이즈 제거 (모서리 깜빡임 방지)
        self.count_history.append(total_count)
        smoothed_count = int(round(np.median(list(self.count_history))))
        stable_detections = limit_detections_to_count(detections, smoothed_count)
        stable_class_counts = Counter({"blob": smoothed_count}) if stable_detections else Counter()

        annotated_bgr = draw_annotated(
            image_bgr=image_bgr,
            detections=stable_detections,
            total_count=smoothed_count,
            roi=active_roi,
            roi_polygon=active_polygon,
            rejected=rejected,
            depth_used=depth_used,
        )

        if self.show:
            self.draw_mouse_overlay(annotated_bgr)

        self.publish_results(
            total_count=smoothed_count,
            class_counts=stable_class_counts,
            detections=stable_detections,
            rejected=rejected,
            method=self.method,
            roi=active_roi,
            roi_polygon=active_polygon,
            annotated_bgr=annotated_bgr,
            stamp=msg.header.stamp,
            frame_id=msg.header.frame_id,
            depth_used=depth_used,
            depth_debug=depth_debug,
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
                f"[DECISION] total_blocks={smoothed_count} (raw={total_count}), "
                f"classes={dict(stable_class_counts)}, "
                f"accepted={len(stable_detections)} (raw={len(detections)}), rejected={len(rejected)}, "
                f"depth_used={depth_used}, depth_debug={depth_debug}, "
                f"roi={active_roi}, polygon={active_polygon}, "
                f"elapsed={elapsed:.3f}s"
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
                cv2.LINE_AA,
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
                2,
            )

    def publish_results(
        self,
        total_count,
        class_counts,
        detections,
        rejected,
        method,
        roi,
        roi_polygon,
        annotated_bgr,
        stamp,
        frame_id,
        depth_used=False,
        depth_debug=None,
    ):
        count_msg = Int32()
        count_msg.data = int(total_count)
        self.count_pub.publish(count_msg)

        summary = {
            "total_blocks": int(total_count),
            "method": method,
            "classes": dict(class_counts),
            "detections": detections,
            "rejected": rejected,
            "depth_used": bool(depth_used),
            "depth_debug": depth_debug,
            "cv_params": {
                "min_area": float(self.cv_min_area),
                "min_area_ratio": float(self.cv_min_area_ratio),
                "fixed_max_area": float(self.cv_fixed_max_area),
                "area_y_top": float(self.cv_area_y_top),
                "area_y_bottom": float(self.cv_area_y_bottom),
                "single_area_top": float(self.cv_single_area_top),
                "single_area_bottom": float(self.cv_single_area_bottom),
                "max_estimated_count": int(self.cv_max_estimated_count),
                "multi_count_ratio": float(self.cv_multi_count_ratio),
                "sat_min": int(self.cv_sat_min),
                "use_edge": bool(self.cv_use_edge),
                "edge_low": int(self.cv_edge_low),
                "edge_high": int(self.cv_edge_high),
                "use_depth": bool(self.use_depth),
                "depth_topic": self.depth_topic,
                "depth_combine_mode": self.depth_combine_mode,
                "depth_filter_min_valid_ratio": float(self.depth_filter_min_valid_ratio),
                "depth_filter_min_object_ratio": float(self.depth_filter_min_object_ratio),
                "depth_object_margin_m": float(self.depth_object_margin_m),
                "depth_floor_percentile": float(self.depth_floor_percentile),
                "morph_kernel": int(self.cv_morph_kernel),
                "close_iterations": int(self.cv_close_iterations),
                "open_iterations": int(self.cv_open_iterations),
                "watershed": bool(self.cv_watershed),
                "watershed_dist_ratio": float(self.cv_watershed_dist_ratio),
                "watershed_max_parts": int(self.cv_watershed_max_parts),
                "split_min_area_ratio": float(self.cv_split_min_area_ratio),
            },
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
        description="realsense-ros image/depth 토픽을 구독해 바닥 블럭 개수를 판단합니다."
    )

    parser.add_argument("--image-topic", default=DEBUG_IMAGE_TOPIC)
    parser.add_argument("--depth-topic", default=DEBUG_DEPTH_TOPIC)
    parser.add_argument("--annotated-topic", default=DEBUG_ANNOTATED_TOPIC)
    parser.add_argument("--count-topic", default=DEBUG_COUNT_TOPIC)
    parser.add_argument("--summary-topic", default=DEBUG_SUMMARY_TOPIC)

    parser.add_argument("--method", choices=("cv", "yolo"), default=DEBUG_METHOD)

    parser.add_argument("--conf", type=float, default=DEBUG_YOLO_CONF)
    parser.add_argument("--iou", type=float, default=DEBUG_YOLO_IOU)
    parser.add_argument("--device", default=DEBUG_YOLO_DEVICE)

    parser.add_argument("--cv-min-area", type=float, default=DEBUG_CV_MIN_AREA)
    parser.add_argument("--cv-min-area-ratio", type=float, default=DEBUG_CV_MIN_AREA_RATIO)
    parser.add_argument("--cv-fixed-max-area", type=float, default=DEBUG_CV_FIXED_MAX_AREA)
    parser.add_argument("--cv-max-area", dest="cv_fixed_max_area", type=float)

    parser.add_argument("--cv-area-y-top", type=float, default=DEBUG_CV_AREA_Y_TOP)
    parser.add_argument("--cv-area-y-bottom", type=float, default=DEBUG_CV_AREA_Y_BOTTOM)
    parser.add_argument("--cv-single-area-top", type=float, default=DEBUG_CV_SINGLE_AREA_TOP)
    parser.add_argument("--cv-single-area-bottom", type=float, default=DEBUG_CV_SINGLE_AREA_BOTTOM)
    parser.add_argument("--cv-max-estimated-count", type=int, default=DEBUG_CV_MAX_ESTIMATED_COUNT)
    parser.add_argument("--cv-multi-count-ratio", type=float, default=DEBUG_CV_MULTI_COUNT_RATIO)

    parser.add_argument("--cv-sat-min", type=int, default=DEBUG_CV_SAT_MIN)
    parser.add_argument("--cv-use-edge", action=argparse.BooleanOptionalAction, default=DEBUG_CV_USE_EDGE)
    parser.add_argument("--cv-edge-low", type=int, default=DEBUG_CV_EDGE_LOW)
    parser.add_argument("--cv-edge-high", type=int, default=DEBUG_CV_EDGE_HIGH)

    parser.add_argument("--cv-morph-kernel", type=int, default=DEBUG_CV_MORPH_KERNEL)
    parser.add_argument("--cv-close-iterations", type=int, default=DEBUG_CV_CLOSE_ITERATIONS)
    parser.add_argument("--cv-open-iterations", type=int, default=DEBUG_CV_OPEN_ITERATIONS)

    parser.add_argument("--use-depth", action=argparse.BooleanOptionalAction, default=DEBUG_USE_DEPTH)
    parser.add_argument("--depth-16uc1-scale", type=float, default=DEBUG_DEPTH_16UC1_SCALE)
    parser.add_argument("--depth-min-valid-m", type=float, default=DEBUG_DEPTH_MIN_VALID_M)
    parser.add_argument("--depth-max-valid-m", type=float, default=DEBUG_DEPTH_MAX_VALID_M)
    parser.add_argument("--depth-floor-percentile", type=float, default=DEBUG_DEPTH_FLOOR_PERCENTILE)
    parser.add_argument("--depth-object-margin-m", type=float, default=DEBUG_DEPTH_OBJECT_MARGIN_M)
    parser.add_argument("--depth-row-min-valid-pixels", type=int, default=DEBUG_DEPTH_ROW_MIN_VALID_PIXELS)
    parser.add_argument("--depth-row-smooth-kernel", type=int, default=DEBUG_DEPTH_ROW_SMOOTH_KERNEL)
    parser.add_argument("--depth-mask-open-iterations", type=int, default=DEBUG_DEPTH_MASK_OPEN_ITERATIONS)
    parser.add_argument("--depth-mask-close-iterations", type=int, default=DEBUG_DEPTH_MASK_CLOSE_ITERATIONS)
    parser.add_argument("--depth-resize-if-needed", action=argparse.BooleanOptionalAction, default=DEBUG_DEPTH_RESIZE_IF_NEEDED)
    parser.add_argument("--depth-combine-mode", choices=("filter", "and", "or", "depth"), default=DEBUG_DEPTH_COMBINE_MODE)
    parser.add_argument("--depth-filter-min-valid-ratio", type=float, default=DEBUG_DEPTH_FILTER_MIN_VALID_RATIO)
    parser.add_argument("--depth-filter-min-object-ratio", type=float, default=DEBUG_DEPTH_FILTER_MIN_OBJECT_RATIO)

    parser.add_argument("--cv-watershed", action=argparse.BooleanOptionalAction, default=DEBUG_CV_WATERSHED)
    parser.add_argument("--cv-watershed-dist-ratio", type=float, default=DEBUG_CV_WATERSHED_DIST_RATIO)
    parser.add_argument("--cv-watershed-max-parts", type=int, default=DEBUG_CV_WATERSHED_MAX_PARTS)
    parser.add_argument("--cv-split-min-area-ratio", type=float, default=DEBUG_CV_SPLIT_MIN_AREA_RATIO)
    parser.add_argument("--roi-inner-margin", type=int, default=DEBUG_ROI_INNER_MARGIN)

    parser.add_argument("--process-every-n", type=int, default=DEBUG_PROCESS_EVERY_N)
    parser.add_argument("--count-smooth-n", type=int, default=DEBUG_COUNT_SMOOTH_N)

    parser.add_argument("--roi", nargs=4, type=float, metavar=("X1", "Y1", "X2", "Y2"), default=None)
    parser.add_argument("--roi-polygon", nargs="+", type=float, metavar="XY", default=DEBUG_ROI_POLYGON)

    parser.add_argument("--log-period-sec", type=float, default=DEBUG_LOG_PERIOD_SEC)
    parser.add_argument("--show", action=argparse.BooleanOptionalAction, default=DEBUG_SHOW)
    parser.add_argument("--save-dir", default=DEBUG_SAVE_DIR)

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
