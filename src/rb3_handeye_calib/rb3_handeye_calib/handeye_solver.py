"""
===============================================================================
File        : handeye_solver.py
Package     : rb3_handeye_calib
Author      : Jeho Yoon, Chaerin Seong, Dahan Lee, Donghyuk Jeong, Deokhui Han, Donggil Lee
Created     : 2026-06-30
Environment : Ubuntu 22.04, ROS2 Humble, Python 3.10

Description
-----------
ROS2 node that reads samples.yaml collected by sample_collector and solves
the eye-in-hand calibration problem using cv2.calibrateHandEye.
Saves the result T_cam2gripper matrix to result_handeye.yaml.

Main Features
-------------
- Reads TCP pose + ChArUco pose pairs from samples.yaml
- Solves eye-in-hand: T_cam2gripper (camera in gripper frame)
- Supports multiple methods: Tsai, Park, Horaud, Andreff, Daniilidis
- ROS parameters: samples_yaml, output_yaml, method, euler_order

Required Nodes
--------------
- None (run after sample collection is complete)

Notes
-----
- Minimum 3 samples required; 15+ recommended for accuracy
- Usage: ros2 run rb3_handeye_calib handeye_solver \
           --ros-args -p samples_yaml:=<path> -p method:=Tsai

Revision History
----------------

===============================================================================
"""

import os
import sys

import rclpy
from rclpy.node import Node

import cv2
import numpy as np
import yaml

from rb3_handeye_calib.transform_utils import (
    tcp_pose_to_gripper2base,
    rotation_matrix_to_rpy_deg,
    rotation_matrix_to_quaternion,
    make_homogeneous,
)


METHOD_MAP = {
    'Tsai'      : cv2.CALIB_HAND_EYE_TSAI,
    'Park'      : cv2.CALIB_HAND_EYE_PARK,
    'Horaud'    : cv2.CALIB_HAND_EYE_HORAUD,
    'Andreff'   : cv2.CALIB_HAND_EYE_ANDREFF,
    'Daniilidis': cv2.CALIB_HAND_EYE_DANIILIDIS,
}


class HandeyeSolver(Node):
    def __init__(self):
        super().__init__('handeye_solver')

        self.declare_parameter('samples_yaml', '')
        self.declare_parameter('output_yaml', '')
        self.declare_parameter('method', 'Tsai')
        self.declare_parameter('euler_order', 'xyz')

        samples_yaml = self.get_parameter('samples_yaml').value
        output_yaml  = self.get_parameter('output_yaml').value
        method_str   = self.get_parameter('method').value
        euler_order  = self.get_parameter('euler_order').value

        if not samples_yaml:
            self.get_logger().error('samples_yaml 파라미터가 비어있습니다.')
            rclpy.shutdown()
            return

        samples_yaml = os.path.expanduser(samples_yaml)
        if not os.path.exists(samples_yaml):
            self.get_logger().error(f'파일 없음: {samples_yaml}')
            rclpy.shutdown()
            return

        if not output_yaml:
            output_yaml = os.path.join(
                os.path.dirname(samples_yaml), 'result_handeye.yaml'
            )
        output_yaml = os.path.expanduser(output_yaml)

        if method_str not in METHOD_MAP:
            self.get_logger().error(
                f'알 수 없는 method: {method_str}. 가능: {list(METHOD_MAP.keys())}'
            )
            rclpy.shutdown()
            return

        method = METHOD_MAP[method_str]

        # ── 샘플 로드 ────────────────────────────────────────────────────
        with open(samples_yaml, 'r') as f:
            samples = yaml.safe_load(f) or []

        self.get_logger().info(f'{len(samples)}개 샘플 로드: {samples_yaml}')

        if len(samples) < 3:
            self.get_logger().error(
                f'샘플이 {len(samples)}개뿐입니다. 최소 3개 필요 (권장 15개 이상).'
            )
            rclpy.shutdown()
            return

        # ── 행렬 배열 구성 ───────────────────────────────────────────────
        R_g2b_list, t_g2b_list = [], []
        R_t2c_list, t_t2c_list = [], []
        skipped = 0

        for s in samples:
            try:
                tcp = s['tcp_mm_deg']
                rvec = np.array(s['rvec_target2cam'], dtype=np.float64).reshape(3, 1)
                tvec = np.array(s['tvec_target2cam_m'], dtype=np.float64).reshape(3, 1)

                R_g2b, t_g2b = tcp_pose_to_gripper2base(tcp, euler_order)
                R_t2c, _ = cv2.Rodrigues(rvec)
                t_t2c = tvec.flatten()

                R_g2b_list.append(R_g2b)
                t_g2b_list.append(t_g2b.reshape(3, 1))
                R_t2c_list.append(R_t2c)
                t_t2c_list.append(t_t2c.reshape(3, 1))
            except Exception as e:
                self.get_logger().warn(f'샘플 #{s.get("index","?")} 파싱 실패: {e}')
                skipped += 1

        valid = len(R_g2b_list)
        self.get_logger().info(
            f'유효 샘플: {valid}개 (스킵: {skipped}개)'
        )
        if valid < 3:
            self.get_logger().error(f'유효 샘플 {valid}개 < 3. 종료.')
            rclpy.shutdown()
            return

        # ── calibrateHandEye ─────────────────────────────────────────────
        self.get_logger().info(f'calibrateHandEye 실행 (method={method_str}) ...')
        try:
            R_cam2grip, t_cam2grip = cv2.calibrateHandEye(
                R_g2b_list, t_g2b_list,
                R_t2c_list, t_t2c_list,
                method=method,
            )
        except cv2.error as e:
            self.get_logger().error(f'calibrateHandEye 실패: {e}')
            rclpy.shutdown()
            return

        # ── 잔차 계산 ────────────────────────────────────────────────────
        rot_errs, trans_errs = self._compute_residuals(
            R_g2b_list, t_g2b_list,
            R_t2c_list, t_t2c_list,
            R_cam2grip, t_cam2grip,
        )
        mean_rot_deg   = float(np.mean(rot_errs))
        mean_trans_mm  = float(np.mean(trans_errs)) * 1000.0
        self.get_logger().info(
            f'잔차 → 회전: {mean_rot_deg:.4f} deg (mean), '
            f'이동: {mean_trans_mm:.3f} mm (mean)'
        )

        # ── 결과 저장 ────────────────────────────────────────────────────
        rpy_deg = rotation_matrix_to_rpy_deg(R_cam2grip, euler_order)
        quat    = rotation_matrix_to_quaternion(R_cam2grip)   # [x,y,z,w]
        t_mm    = t_cam2grip.flatten() * 1000.0

        result = {
            'method'          : method_str,
            'euler_order'     : euler_order,
            'n_samples'       : valid,
            'residual_rotation_deg_mean'  : mean_rot_deg,
            'residual_translation_mm_mean': mean_trans_mm,
            'T_cam2gripper': {
                'rotation_matrix': R_cam2grip.tolist(),
                'quaternion_xyzw': quat,
                'rpy_deg'        : rpy_deg,
                'translation_mm' : t_mm.tolist(),
                'translation_m'  : t_cam2grip.flatten().tolist(),
            },
        }

        with open(output_yaml, 'w') as f:
            yaml.dump(result, f, allow_unicode=True, sort_keys=False)

        self.get_logger().info(
            f'\n'
            f'━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n'
            f' T_cam2gripper (camera in gripper frame)\n'
            f'━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n'
            f' Translation (mm): {t_mm[0]:.3f}, {t_mm[1]:.3f}, {t_mm[2]:.3f}\n'
            f' RPY ({euler_order}, deg): {rpy_deg[0]:.4f}, {rpy_deg[1]:.4f}, {rpy_deg[2]:.4f}\n'
            f' Quaternion (x,y,z,w): {quat[0]:.6f}, {quat[1]:.6f}, {quat[2]:.6f}, {quat[3]:.6f}\n'
            f' 잔차 회전: {mean_rot_deg:.4f} deg  이동: {mean_trans_mm:.3f} mm\n'
            f' 결과 저장: {output_yaml}\n'
            f'━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━'
        )

        rclpy.shutdown()

    # ── 잔차 계산 ────────────────────────────────────────────────────────────

    def _compute_residuals(
        self,
        R_g2b_list, t_g2b_list,
        R_t2c_list, t_t2c_list,
        R_cam2grip, t_cam2grip,
    ):
        """
        AX = XB 형태 검증:
        T_g2b_i * T_cam2grip * T_board2cam_i ≈ const (= T_board2base)

        쌍 i,j 에 대해 A_ij * X = X * B_ij 오차 계산.
        """
        T_X = make_homogeneous(R_cam2grip, t_cam2grip.flatten())

        T_g2b = [make_homogeneous(R, t.flatten()) for R, t in zip(R_g2b_list, t_g2b_list)]
        T_t2c = [make_homogeneous(R, t.flatten()) for R, t in zip(R_t2c_list, t_t2c_list)]

        rot_errs, trans_errs = [], []
        n = len(T_g2b)
        for i in range(n):
            for j in range(i + 1, n):
                A = np.linalg.inv(T_g2b[i]) @ T_g2b[j]
                B = T_t2c[i] @ np.linalg.inv(T_t2c[j])

                lhs = A @ T_X
                rhs = T_X @ B
                diff = np.linalg.inv(lhs) @ rhs

                R_err = diff[:3, :3]
                t_err = diff[:3, 3]

                # 회전 오차: Rodrigues 각도
                angle = abs(np.arccos(
                    np.clip((np.trace(R_err) - 1.0) / 2.0, -1.0, 1.0)
                ))
                rot_errs.append(np.degrees(angle))
                trans_errs.append(np.linalg.norm(t_err))

        if not rot_errs:
            return [0.0], [0.0]
        return rot_errs, trans_errs


def main(args=None):
    rclpy.init(args=args)
    node = HandeyeSolver()
    try:
        if rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
    except Exception:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
