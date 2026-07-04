"""
===============================================================================
File        : shared_config.py
Package     : vision_assembly_pkg
Author      : Jeho Yoon, Chaerin Seong, Dahan Lee, Donghyuk Jeong, Deokhui Han, Donggil Lee
Created     : 2026-06-30
Environment : Ubuntu 22.04, ROS2 Humble, Python 3.10

Description
-----------
Shared configuration module for vision_assembly_pkg.
Defines the ROI polygon used by both birdseye_assembly and decision_assembly,
ensuring a single source of truth for camera region-of-interest settings.

Main Features
-------------
- ROI_POLYGON : birdseye camera region-of-interest (clockwise: TL→TR→BR→BL)
- Single edit point for both birdseye_assembly and decision_assembly

Required Nodes
--------------
- None (imported as a module)

Notes
-----
- Coordinates assume camera installed at 180° rotation

Revision History
----------------

===============================================================================
"""

# ROI polygon: 좌상 → 우상 → 우하 → 좌하 순서 (카메라 180° 재설치 기준)
# ※ 여기 한 곳만 바꾸면 decision_assembly / birdseye_assembly 모두 적용됨.
ROI_POLYGON = [
    (55,  0),    # 좌상
    (550, 0),    # 우상
    (428, 425),  # 우하
    (182, 425),  # 좌하
]
