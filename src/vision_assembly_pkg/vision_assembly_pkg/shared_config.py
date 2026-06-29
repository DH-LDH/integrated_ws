# ROI polygon: 좌상 → 우상 → 우하 → 좌하 순서 (카메라 180° 재설치 기준)
# ※ 여기 한 곳만 바꾸면 decision_assembly / birdseye_assembly 모두 적용됨.
ROI_POLYGON = [
    (66,  0),    # 좌상
    (605, 0),    # 우상
    (425, 480),  # 우하
    (175, 480),  # 좌하
]
