"""
transform_utils.py
==================
좌표 변환 유틸리티 모음.

단위 규칙
---------
* translation : meter (m)  ←→  mm 변환 함수 제공
* rotation    : degree 입력 → 내부적으로 radian 처리
* Euler order : 기본값 'xyz' (extrinsic / fixed-axis)
                Rainbow Robotics RB3의 TCP rx/ry/rz가 어떤 순서인지
                실제 로봇에서 확인 후 euler_order를 맞춰야 합니다.
                의심스러우면 'zyx'(RPY) / 'xyz' 둘 다 시도하고 결과를 비교하세요.

scipy 없는 환경에서도 동작하는 fallback 구현을 포함합니다.
"""

import math
import numpy as np

# scipy가 있으면 사용, 없으면 fallback
try:
    from scipy.spatial.transform import Rotation as _SciRot
    _USE_SCIPY = True
except ImportError:
    _USE_SCIPY = False


# ─────────────────────────────────────────────────────────────────────────────
# 기본 변환 함수
# ─────────────────────────────────────────────────────────────────────────────

def rpy_deg_to_rotation_matrix(rx_deg, ry_deg, rz_deg, euler_order='xyz'):
    """
    Euler 각도(degree) → 3×3 rotation matrix.

    Parameters
    ----------
    rx_deg, ry_deg, rz_deg : float  – 각도 (degree)
    euler_order : str  – 'xyz', 'zyx', 'zyz' 등 scipy 규격 문자열
                         RB3 기본값은 'xyz' (extrinsic X→Y→Z)
                         확인되지 않은 경우 실제 로봇과 비교 검증 필요

    Returns
    -------
    R : np.ndarray (3, 3)
    """
    angles_deg = [rx_deg, ry_deg, rz_deg]
    if _USE_SCIPY:
        # scipy는 extrinsic → 소문자 축 문자열
        R = _SciRot.from_euler(euler_order, angles_deg, degrees=True).as_matrix()
    else:
        R = _rpy_fallback(rx_deg, ry_deg, rz_deg, euler_order)
    return R


def _rpy_fallback(rx_deg, ry_deg, rz_deg, euler_order='xyz'):
    """scipy 없을 때 extrinsic X→Y→Z fallback."""
    cx, sx = math.cos(math.radians(rx_deg)), math.sin(math.radians(rx_deg))
    cy, sy = math.cos(math.radians(ry_deg)), math.sin(math.radians(ry_deg))
    cz, sz = math.cos(math.radians(rz_deg)), math.sin(math.radians(rz_deg))

    Rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
    Ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    Rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])

    order_map = {'x': Rx, 'y': Ry, 'z': Rz}
    R = np.eye(3)
    for axis in euler_order.lower():
        R = R @ order_map[axis]
    return R


def rotation_matrix_to_rpy_deg(R, euler_order='xyz'):
    """
    3×3 rotation matrix → Euler 각도 (degree).

    Returns
    -------
    [rx_deg, ry_deg, rz_deg] : list[float]
    """
    if _USE_SCIPY:
        angles = _SciRot.from_matrix(R).as_euler(euler_order, degrees=True)
    else:
        angles = _rotation_to_rpy_fallback(R)
    return list(angles)


def _rotation_to_rpy_fallback(R):
    """extrinsic XYZ 가정 fallback."""
    sy = math.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    singular = sy < 1e-6
    if not singular:
        rx = math.degrees(math.atan2(R[2, 1], R[2, 2]))
        ry = math.degrees(math.atan2(-R[2, 0], sy))
        rz = math.degrees(math.atan2(R[1, 0], R[0, 0]))
    else:
        rx = math.degrees(math.atan2(-R[1, 2], R[1, 1]))
        ry = math.degrees(math.atan2(-R[2, 0], sy))
        rz = 0.0
    return [rx, ry, rz]


def rotation_matrix_to_quaternion(R):
    """
    3×3 rotation matrix → quaternion [x, y, z, w].

    Returns
    -------
    [qx, qy, qz, qw] : list[float]
    """
    if _USE_SCIPY:
        q = _SciRot.from_matrix(R).as_quat()  # [x, y, z, w]
    else:
        q = _rot_to_quat_fallback(R)
    return list(q)


def _rot_to_quat_fallback(R):
    """Shepperd's method."""
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    if trace > 0:
        s = 0.5 / math.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return [x, y, z, w]


def make_homogeneous(R, t):
    """
    4×4 동차 변환 행렬 생성.

    Parameters
    ----------
    R : np.ndarray (3, 3)
    t : array-like (3,) – translation (m)

    Returns
    -------
    T : np.ndarray (4, 4)
    """
    T = np.eye(4)
    T[:3, :3] = np.asarray(R)
    T[:3, 3] = np.asarray(t).flatten()
    return T


def invert_transform(T):
    """
    4×4 변환 행렬 역변환.

    T_inv = [R^T  -R^T t]
            [  0       1]

    Returns
    -------
    T_inv : np.ndarray (4, 4)
    """
    R = T[:3, :3]
    t = T[:3, 3]
    T_inv = np.eye(4)
    T_inv[:3, :3] = R.T
    T_inv[:3, 3] = -R.T @ t
    return T_inv


# ─────────────────────────────────────────────────────────────────────────────
# 로봇 TCP pose → 변환 행렬
# ─────────────────────────────────────────────────────────────────────────────

def tcp_pose_to_gripper2base(pose_mm_deg, euler_order='xyz'):
    """
    RB3 TCP pose [x_mm, y_mm, z_mm, rx_deg, ry_deg, rz_deg] →
    R_gripper2base (3×3), t_gripper2base (m).

    Parameters
    ----------
    pose_mm_deg : array-like (6,)
        [x_mm, y_mm, z_mm, rx_deg, ry_deg, rz_deg]
        x/y/z 단위: mm  /  rx/ry/rz 단위: degree
    euler_order : str
        RB3 TCP rx/ry/rz Euler 순서. 확인 후 적절히 설정.
        기본값 'xyz' (extrinsic X→Y→Z)

    Returns
    -------
    R_gripper2base : np.ndarray (3, 3)
    t_gripper2base : np.ndarray (3,)  – meter 단위
    """
    x_mm, y_mm, z_mm, rx_deg, ry_deg, rz_deg = pose_mm_deg
    # mm → m
    t = np.array([x_mm, y_mm, z_mm]) / 1000.0
    R = rpy_deg_to_rotation_matrix(rx_deg, ry_deg, rz_deg, euler_order)
    return R, t


def gripper2base_to_tcp_pose_mm_deg(R, t_m, euler_order='xyz'):
    """
    역방향: R, t(m) → [x_mm, y_mm, z_mm, rx_deg, ry_deg, rz_deg].
    """
    t_mm = np.asarray(t_m) * 1000.0
    rpy = rotation_matrix_to_rpy_deg(R, euler_order)
    return list(t_mm) + rpy
