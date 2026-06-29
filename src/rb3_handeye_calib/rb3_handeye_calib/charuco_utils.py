"""
===============================================================================
File        : charuco_utils.py
Package     : rb3_handeye_calib
Author      : Jeho Yoon, Chaerin Seong, Dahan Lee, Donghyuk Jeong, Deokhui Han, Donggil Lee
Created     : 2026-06-30
Environment : Ubuntu 22.04, ROS2 Humble, Python 3.10

Description
-----------
Utility module for ChArUco board detection and camera-frame pose estimation.
Handles OpenCV version differences transparently for cross-version compatibility.

Main Features
-------------
- ChArUco board detection (OpenCV >= 4.7: CharucoDetector; < 4.7: legacy API)
- Camera-frame pose estimation from detected board corners
- ArUco dictionary lookup by string name

Required Nodes
--------------
- None (utility module, imported by other nodes)

Notes
-----
- OpenCV >= 4.7: uses cv2.aruco.CharucoDetector (recommended)
- OpenCV <  4.7: detectMarkers → interpolateCornersCharuco → estimatePoseCharucoBoard

Revision History
----------------

===============================================================================
"""

import cv2
import numpy as np


def _get_aruco_dict(dictionary_name: str):
    """문자열로 ArUco dictionary 객체 반환."""
    name_map = {
        'DICT_4X4_50':   cv2.aruco.DICT_4X4_50,
        'DICT_4X4_100':  cv2.aruco.DICT_4X4_100,
        'DICT_4X4_250':  cv2.aruco.DICT_4X4_250,
        'DICT_4X4_1000': cv2.aruco.DICT_4X4_1000,
        'DICT_5X5_50':   cv2.aruco.DICT_5X5_50,
        'DICT_5X5_100':  cv2.aruco.DICT_5X5_100,
        'DICT_6X6_50':   cv2.aruco.DICT_6X6_50,
        'DICT_6X6_100':  cv2.aruco.DICT_6X6_100,
        'DICT_6X6_250':  cv2.aruco.DICT_6X6_250,
    }
    if dictionary_name not in name_map:
        raise ValueError(
            f"알 수 없는 dictionary: {dictionary_name}. "
            f"가능 목록: {list(name_map.keys())}"
        )
    return cv2.aruco.getPredefinedDictionary(name_map[dictionary_name])


def make_charuco_board(
    squares_x: int,
    squares_y: int,
    square_length_m: float,
    marker_length_m: float,
    dictionary_name: str = 'DICT_4X4_50',
):
    """
    ChArUco board 객체 생성.

    Returns
    -------
    board : cv2.aruco.CharucoBoard
    aruco_dict : cv2.aruco.Dictionary
    """
    aruco_dict = _get_aruco_dict(dictionary_name)
    board = cv2.aruco.CharucoBoard(
        (squares_x, squares_y),
        square_length_m,
        marker_length_m,
        aruco_dict,
    )
    return board, aruco_dict


def detect_charuco_pose(
    image_bgr: np.ndarray,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
    board,
    aruco_dict,
    min_corners: int = 8,
):
    """
    컬러 이미지에서 ChArUco 보드를 검출하고 pose를 추정한다.

    Parameters
    ----------
    image_bgr    : np.ndarray  – BGR 이미지
    camera_matrix: np.ndarray (3,3)
    dist_coeffs  : np.ndarray  – 왜곡 계수
    board        : cv2.aruco.CharucoBoard
    aruco_dict   : cv2.aruco.Dictionary
    min_corners  : int  – 최소 코너 수 (기본 8)

    Returns
    -------
    success       : bool
    rvec          : np.ndarray (3,1) or None  – target→camera rotation vector
    tvec          : np.ndarray (3,1) or None  – target→camera translation (m)
    annotated     : np.ndarray                – 주석 이미지
    n_corners     : int                       – 검출된 ChArUco 코너 수
    reason        : str                       – 실패 시 사유
    """
    gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
    annotated = image_bgr.copy()

    charuco_corners = None
    charuco_ids = None
    n_corners = 0

    # ── 검출: OpenCV 버전에 따라 분기 ────────────────────────────────────
    has_charuco_detector = hasattr(cv2.aruco, 'CharucoDetector')

    if has_charuco_detector:
        # OpenCV >= 4.7
        detector_params = cv2.aruco.DetectorParameters()
        charuco_params = cv2.aruco.CharucoParameters()
        detector = cv2.aruco.CharucoDetector(board, charuco_params, detector_params)
        charuco_corners, charuco_ids, marker_corners, marker_ids = detector.detectBoard(gray)
    else:
        # OpenCV < 4.7 fallback
        detector_params = cv2.aruco.DetectorParameters_create()
        marker_corners, marker_ids, _ = cv2.aruco.detectMarkers(
            gray, aruco_dict, parameters=detector_params
        )
        if marker_ids is not None and len(marker_ids) > 0:
            _, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
                marker_corners, marker_ids, gray, board
            )

    # ── ArUco 마커 표시 ───────────────────────────────────────────────────
    if has_charuco_detector:
        if 'marker_corners' in dir() and marker_corners and len(marker_corners) > 0:
            cv2.aruco.drawDetectedMarkers(annotated, marker_corners, marker_ids)
    else:
        if 'marker_corners' in dir() and marker_corners and len(marker_corners) > 0:
            cv2.aruco.drawDetectedMarkers(annotated, marker_corners, marker_ids)

    # ── 코너 수 확인 ─────────────────────────────────────────────────────
    if charuco_ids is None or len(charuco_ids) == 0:
        return False, None, None, annotated, 0, "ArUco 마커 검출 실패 (보드가 보이는지, 해상도/조명을 확인하세요)"

    n_corners = len(charuco_ids)
    cv2.aruco.drawDetectedCornersCharuco(annotated, charuco_corners, charuco_ids, (0, 255, 0))

    if n_corners < min_corners:
        return (
            False, None, None, annotated, n_corners,
            f"코너 수 부족: {n_corners} < min_corners={min_corners}. "
            "보드를 카메라에 더 가까이/정면으로 이동하세요."
        )

    # ── Pose 추정 ─────────────────────────────────────────────────────────
    try:
        ok, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(
            charuco_corners, charuco_ids, board, camera_matrix, dist_coeffs, None, None
        )
    except cv2.error as e:
        return False, None, None, annotated, n_corners, f"estimatePoseCharucoBoard 오류: {e}"

    if not ok:
        return (
            False, None, None, annotated, n_corners,
            "pose 추정 실패 (solvePnP 수렴 실패). 코너가 한쪽에 몰려 있으면 보드 방향을 바꾸세요."
        )

    # 축 그리기 (square_length_m 크기의 축)
    sq = board.getSquareLength()
    cv2.drawFrameAxes(annotated, camera_matrix, dist_coeffs, rvec, tvec, sq * 1.5)

    # tvec 단위 확인: board는 meter 단위이므로 tvec도 meter
    return True, rvec, tvec, annotated, n_corners, "OK"


def rvec_tvec_to_R_t(rvec, tvec):
    """
    Rodrigues rvec/tvec → R (3×3), t (3,).

    Returns
    -------
    R : np.ndarray (3, 3)
    t : np.ndarray (3,)  – meter 단위 (board가 m이면 m)
    """
    R, _ = cv2.Rodrigues(rvec)
    t = tvec.flatten()
    return R, t
