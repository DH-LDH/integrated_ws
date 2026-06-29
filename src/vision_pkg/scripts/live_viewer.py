#!/usr/bin/env python3
"""
===============================================================================
File        : live_viewer.py
Package     : vision_pkg
Author      : Jeho Yoon, Chaerin Seong, Dahan Lee, Donghyuk Jeong, Deokhui Han, Donggil Lee
Created     : 2026-06-30
Environment : Ubuntu 22.04, ROS2 Humble, Python 3.10

Description
-----------
Standalone live viewer script that continuously runs the camera using
VisionManager and overlays detected block information (class_name, x/y_mm,
yaw_deg) on the camera feed for real-time visual verification.

Main Features
-------------
- Continuous camera loop using vision_pkg VisionManager
- Overlays class_name, x/y position (mm), and yaw (deg) on each detected block
- Press 'q' to exit

Required Nodes
--------------
- None (standalone script, does not require ROS2 runtime)

Notes
-----
Usage: cd /home/orinagx/integrated_ws && source install/setup.bash
       python3 src/vision_pkg/scripts/live_viewer.py

Revision History
----------------

===============================================================================
"""

import sys
import os
import time

import cv2
import numpy as np

# vision_pkg 경로 추가 (source 없이 직접 실행 시)
_ws = os.path.join(os.path.dirname(__file__), '..', '..', '..')
sys.path.insert(0, os.path.join(_ws, 'install', 'vision_pkg', 'lib', 'python3.10', 'dist-packages'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from vision_pkg.INUVisionCall import VisionManager

# ── 설정 ──────────────────────────────────────────────────────────────────────
CAMERA_SERIAL = "332322072441"   # 사용할 카메라 시리얼
CAMERA_MODE   = "mid_50"         # "coarse" | "fine" | "mid_50"
SEARCH_MODE   = "fine"           # search_bricks에 넘길 mode
FPS_LIMIT     = 2.0              # 초당 최대 탐지 횟수 (vision이 느리므로 제한)
WINDOW_TITLE  = "Live Vision Viewer  [q: 종료]"

# 클래스별 색상 (BGR)
_COLOR_PALETTE = [
    (0, 255, 0),   (0, 128, 255), (255, 0, 0),   (0, 255, 255),
    (255, 0, 255), (128, 255, 0), (0, 200, 200), (200, 100, 0),
]
_color_map: dict[str, tuple] = {}


def _get_color(class_name: str) -> tuple:
    if class_name not in _color_map:
        _color_map[class_name] = _COLOR_PALETTE[len(_color_map) % len(_COLOR_PALETTE)]
    return _color_map[class_name]


def draw_overlay(bgr: np.ndarray, pose_table: list) -> np.ndarray:
    """pose_table 항목마다 중심점·클래스·좌표·yaw를 이미지에 그린다."""
    out = bgr.copy()

    for item in pose_table:
        cls  = item.get("class_name", "?")
        x_mm = item.get("x_mm", 0.0)
        y_mm = item.get("y_mm", 0.0)
        yaw  = item.get("yaw_deg", 0.0)

        color = _get_color(cls)

        # ── 중심점 픽셀 결정 ──────────────────────────────────────────────
        # fine 모드: "uv_center" / assembly 모드: "center_uv"
        cuv = item.get("center_uv") or item.get("uv_center")
        if cuv is None:
            continue
        cx, cy = int(cuv[0]), int(cuv[1])

        # ── 원 그리기 ─────────────────────────────────────────────────────
        cv2.circle(out, (cx, cy), 7, color, -1)
        cv2.circle(out, (cx, cy), 7, (255, 255, 255), 1)

        # ── yaw 방향 화살표 (major axis 표현) ────────────────────────────
        arrow_len = 40
        angle_rad = np.radians(yaw)
        ex = int(cx + arrow_len * np.cos(angle_rad))
        ey = int(cy + arrow_len * np.sin(angle_rad))
        cv2.arrowedLine(out, (cx, cy), (ex, ey), color, 2, tipLength=0.3)

        # ── 텍스트 ────────────────────────────────────────────────────────
        label_lines = [
            cls,
            f"x:{x_mm:+.1f} y:{y_mm:+.1f}",
            f"yaw:{yaw:+.1f}",
        ]
        tx, ty = cx + 10, cy - 10
        for i, line in enumerate(label_lines):
            lx, ly = tx, ty + i * 18
            # 배경
            (tw, th), _ = cv2.getTextSize(line, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            cv2.rectangle(out, (lx - 2, ly - th - 2), (lx + tw + 2, ly + 2),
                          (0, 0, 0), -1)
            cv2.putText(out, line, (lx, ly),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)

    return out


def main():
    print(f"[INFO] VisionManager 초기화 (serial={CAMERA_SERIAL})")
    vm = VisionManager()
    vm.camera_serial = CAMERA_SERIAL

    print(f"[INFO] 카메라 세션 시작 (mode={CAMERA_MODE})")
    vm.start_camera(mode=CAMERA_MODE)
    print("[INFO] 카메라 준비 완료. 'q'로 종료.")

    cv2.namedWindow(WINDOW_TITLE, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WINDOW_TITLE, 960, 720)

    interval  = 1.0 / FPS_LIMIT
    t_next    = 0.0
    pose_table: list = []   # 첫 탐지 전 빈 리스트

    while True:
        now = time.monotonic()

        # ── 탐지 (FPS 제한) ──────────────────────────────────────────────
        if now >= t_next:
            t_next = now + interval
            try:
                color_rgb, depth, intrinsics, scale = vm.capture_camera(mode=CAMERA_MODE)
                pose_table, class_index = vm.run_search(SEARCH_MODE)
                print(f"[INFO] 감지 {len(pose_table)}개: "
                      f"{[p['class_name'] for p in pose_table]}")
            except Exception as e:
                print(f"[WARN] 탐지 오류: {e}")
                pose_table = []
                color_rgb  = vm.color_rgb if vm.color_rgb is not None else None

            if color_rgb is not None:
                bgr = cv2.cvtColor(color_rgb, cv2.COLOR_RGB2BGR)
                frame = draw_overlay(bgr, pose_table or [])
            else:
                frame = None
        else:
            # 탐지 주기가 아닐 때는 마지막 프레임 재사용
            if vm.color_rgb is not None:
                bgr = cv2.cvtColor(vm.color_rgb, cv2.COLOR_RGB2BGR)
                frame = draw_overlay(bgr, pose_table or [])
            else:
                frame = None

        if frame is not None:
            cv2.imshow(WINDOW_TITLE, frame)

        key = cv2.waitKey(30) & 0xFF
        if key == ord('q') or key == 27:
            break

    print("[INFO] 종료 중...")
    cv2.destroyAllWindows()
    vm.stop_camera()
    print("[INFO] 카메라 세션 종료.")


if __name__ == "__main__":
    main()
