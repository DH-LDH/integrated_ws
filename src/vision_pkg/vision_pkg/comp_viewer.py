"""
comp_viewer.py — best_comp.pt 실시간 제품 확인 뷰어

실행:
    python3 comp_viewer.py

종료: 카메라 창에서 'q' 키
"""

import os
import sys

os.environ.setdefault("YOLO_CONFIG_DIR", "/tmp")

import cv2
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO

WORKSPACE_DIR   = os.environ.get("ROS2_WS", "/home/orinagx/ros2_ws")
COMP_MODEL_PATH = os.path.join(WORKSPACE_DIR, "best_comp.pt")

# conf 임계값 (낮추면 더 많이 감지, 높이면 확실한 것만)
CONF_THRESHOLD = 0.4

# 클래스별 색상 (BGR)
COLORS = [
    (0, 255, 0), (0, 128, 255), (255, 0, 0), (0, 0, 255),
    (255, 255, 0), (0, 255, 255), (255, 0, 255), (128, 255, 0),
    (0, 128, 128), (128, 0, 255), (255, 128, 0),
]


def get_color(class_id: int):
    return COLORS[class_id % len(COLORS)]


def main():
    if not os.path.exists(COMP_MODEL_PATH):
        print(f"[ERROR] best_comp.pt 없음: {COMP_MODEL_PATH}")
        sys.exit(1)

    print(f"[INFO] best_comp.pt 로드 중: {COMP_MODEL_PATH}")
    model = YOLO(COMP_MODEL_PATH)
    print("[INFO] 모델 로드 완료")

    pipeline = rs.pipeline()
    config   = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    pipeline.start(config)
    print("[INFO] RealSense 카메라 시작 — 'q' 키로 종료")

    try:
        while True:
            frames = pipeline.wait_for_frames(timeout_ms=1000)
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue

            image = np.asanyarray(color_frame.get_data())
            result = model(image, verbose=False, conf=CONF_THRESHOLD)[0]

            # 감지된 박스 그리기
            if result.boxes is not None:
                for box in result.boxes:
                    cls_id   = int(box.cls[0])
                    cls_name = result.names[cls_id]
                    conf     = float(box.conf[0])
                    xyxy     = box.xyxy[0].cpu().numpy().astype(int)

                    color = get_color(cls_id)
                    cv2.rectangle(image, (xyxy[0], xyxy[1]), (xyxy[2], xyxy[3]), color, 2)

                    label = f"{cls_name}  {conf:.0%}"
                    (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.65, 2)
                    cv2.rectangle(image,
                                  (xyxy[0], xyxy[1] - th - 8),
                                  (xyxy[0] + tw + 4, xyxy[1]),
                                  color, -1)
                    cv2.putText(image, label,
                                (xyxy[0] + 2, xyxy[1] - 4),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.65,
                                (0, 0, 0), 2, cv2.LINE_AA)

            # 감지 없을 때 안내 텍스트
            if result.boxes is None or len(result.boxes) == 0:
                cv2.putText(image, "감지된 제품 없음", (20, 40),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.9, (100, 100, 255), 2, cv2.LINE_AA)

            cv2.putText(image, "q: 종료", (image.shape[1] - 90, image.shape[0] - 12),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1, cv2.LINE_AA)

            cv2.imshow("Component Viewer (best_comp.pt)", image)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        pipeline.stop()
        cv2.destroyAllWindows()
        print("[INFO] 종료")


if __name__ == "__main__":
    main()
