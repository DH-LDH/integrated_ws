"""
===============================================================================
File        : sample_collector.py
Package     : rb3_handeye_calib
Author      : Jeho Yoon, Chaerin Seong, Dahan Lee, Donghyuk Jeong, Deokhui Han, Donggil Lee
Created     : 2026-06-30
Environment : Ubuntu 22.04, ROS2 Humble, Python 3.10

Description
-----------
ROS2 node that collects hand-eye calibration samples by pairing TCP poses
from the robot with ChArUco board poses detected from the camera image.
Saves collected samples to a YAML file for use by handeye_solver.

Main Features
-------------
- Subscribes to camera image, camera_info, and TCP pose topics
- /handeye/capture_sample (Trigger): captures one TCP + ChArUco pose pair
- /handeye/reset_samples  (Trigger): clears all samples in current session
- Saves annotated images and samples.yaml to session directory

Required Nodes
--------------
- Camera driver : /camera/camera/color/image_raw, /camera/camera/color/camera_info
- tcp_pose_publisher : /<robot_name>/tcp_pose_array

Notes
-----
- Parameters: robot_name, session_dir, squares_x/y, square/marker_length_mm,
              dictionary, min_corners, euler_order, save_annotated

Revision History
----------------

===============================================================================
"""

import os
import time
import threading
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger

from cv_bridge import CvBridge
import cv2
import numpy as np
import yaml

from rb3_handeye_calib.charuco_utils import make_charuco_board, detect_charuco_pose
from rb3_handeye_calib.transform_utils import rpy_deg_to_rotation_matrix


class SampleCollector(Node):
    def __init__(self):
        super().__init__('sample_collector')

        # ── 파라미터 ────────────────────────────────────────────────────────
        self.declare_parameter('robot_name', 'robot1')
        default_session = os.path.expanduser(
            f'~/handeye_samples/{datetime.now().strftime("%Y%m%d_%H%M%S")}'
        )
        self.declare_parameter('session_dir', default_session)
        self.declare_parameter('squares_x', 7)
        self.declare_parameter('squares_y', 5)
        self.declare_parameter('square_length_mm', 30.0)
        self.declare_parameter('marker_length_mm', 22.0)
        self.declare_parameter('dictionary', 'DICT_4X4_50')
        self.declare_parameter('min_corners', 8)
        self.declare_parameter('euler_order', 'xyz')
        self.declare_parameter('save_annotated', True)

        robot_name      = self.get_parameter('robot_name').value
        _session_dir    = self.get_parameter('session_dir').value
        if not _session_dir:
            _session_dir = default_session
        self.session_dir= os.path.expanduser(_session_dir)
        squares_x       = self.get_parameter('squares_x').value
        squares_y       = self.get_parameter('squares_y').value
        sq_mm           = self.get_parameter('square_length_mm').value
        mk_mm           = self.get_parameter('marker_length_mm').value
        dictionary      = self.get_parameter('dictionary').value
        self.min_corners= self.get_parameter('min_corners').value
        self.euler_order= self.get_parameter('euler_order').value
        self.save_ann   = self.get_parameter('save_annotated').value

        # ── 세션 디렉토리 생성 ─────────────────────────────────────────────
        os.makedirs(self.session_dir, exist_ok=True)
        self.samples_yaml = os.path.join(self.session_dir, 'samples.yaml')
        self.get_logger().info(f'세션 디렉토리: {self.session_dir}')

        # ── ChArUco 보드 ───────────────────────────────────────────────────
        self.board, self.aruco_dict = make_charuco_board(
            squares_x, squares_y,
            sq_mm / 1000.0,
            mk_mm / 1000.0,
            dictionary,
        )
        self.get_logger().info(
            f'ChArUco 보드: {squares_x}x{squares_y}, '
            f'square={sq_mm}mm, marker={mk_mm}mm, {dictionary}'
        )

        # ── 상태 ───────────────────────────────────────────────────────────
        self._lock = threading.Lock()
        self._latest_image: np.ndarray = None
        self._latest_camera_matrix: np.ndarray = None
        self._latest_dist_coeffs: np.ndarray = None
        self._latest_tcp: list = None       # [x_mm,y_mm,z_mm,rx,ry,rz]
        self._sample_count = 0
        self._bridge = CvBridge()

        # ── 기존 샘플 로드 ─────────────────────────────────────────────────
        if os.path.exists(self.samples_yaml):
            with open(self.samples_yaml, 'r') as f:
                existing = yaml.safe_load(f) or []
            self._sample_count = len(existing)
            self.get_logger().info(f'기존 샘플 {self._sample_count}개 로드됨')

        # ── 콜백 그룹 (서비스 중 subscribe 가능하도록 Reentrant) ──────────
        cb_group = ReentrantCallbackGroup()

        # RealSense publishes with BEST_EFFORT — match it to receive frames
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # ── Subscribers ────────────────────────────────────────────────────
        self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self._image_cb,
            sensor_qos,
            callback_group=cb_group,
        )
        self.create_subscription(
            CameraInfo,
            '/camera/camera/color/camera_info',
            self._camera_info_cb,
            sensor_qos,
            callback_group=cb_group,
        )
        self.create_subscription(
            Float64MultiArray,
            f'/{robot_name}/tcp_pose_array',
            self._tcp_cb,
            10,
            callback_group=cb_group,
        )

        # ── Services ───────────────────────────────────────────────────────
        self.create_service(
            Trigger,
            '/handeye/capture_sample',
            self._capture_cb,
            callback_group=cb_group,
        )
        self.create_service(
            Trigger,
            '/handeye/reset_samples',
            self._reset_cb,
            callback_group=cb_group,
        )

        self.get_logger().info(
            '준비 완료. 샘플 캡처: ros2 service call /handeye/capture_sample std_srvs/srv/Trigger'
        )

    # ── Subscriber 콜백 ──────────────────────────────────────────────────────

    def _image_cb(self, msg: Image):
        try:
            img = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'이미지 변환 실패: {e}')
            return
        with self._lock:
            self._latest_image = img

    def _camera_info_cb(self, msg: CameraInfo):
        K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        D = np.array(msg.d, dtype=np.float64)
        with self._lock:
            self._latest_camera_matrix = K
            self._latest_dist_coeffs = D

    def _tcp_cb(self, msg: Float64MultiArray):
        if len(msg.data) != 6:
            return
        with self._lock:
            self._latest_tcp = list(msg.data)

    # ── 서비스 콜백 ──────────────────────────────────────────────────────────

    def _capture_cb(self, request, response):
        with self._lock:
            image = self._latest_image.copy() if self._latest_image is not None else None
            K     = self._latest_camera_matrix.copy() if self._latest_camera_matrix is not None else None
            D     = self._latest_dist_coeffs.copy()   if self._latest_dist_coeffs  is not None else None
            tcp   = list(self._latest_tcp) if self._latest_tcp is not None else None

        # 데이터 가용성 확인
        if image is None:
            response.success = False
            response.message = '이미지가 수신되지 않았습니다. /camera/camera/color/image_raw 확인'
            return response
        if K is None:
            response.success = False
            response.message = 'CameraInfo가 수신되지 않았습니다. /camera/camera/color/camera_info 확인'
            return response
        if tcp is None:
            response.success = False
            response.message = 'TCP pose가 수신되지 않았습니다. tcp_pose_publisher 노드 실행 확인'
            return response

        # ChArUco 검출
        ok, rvec, tvec, annotated, n_corners, reason = detect_charuco_pose(
            image, K, D, self.board, self.aruco_dict, self.min_corners
        )
        if not ok:
            debug_path = os.path.join(self.session_dir, 'debug_last_frame.jpg')
            cv2.imwrite(debug_path, annotated)
            response.success = False
            response.message = f'검출 실패 (n_corners={n_corners}): {reason} | 디버그 이미지: {debug_path}'
            return response

        # 샘플 저장
        idx = self._sample_count
        ts  = time.time()

        sample = {
            'index'      : idx,
            'timestamp'  : ts,
            'tcp_mm_deg' : [float(v) for v in tcp],
            'rvec_target2cam': rvec.flatten().tolist(),
            'tvec_target2cam_m': tvec.flatten().tolist(),
            'n_corners'  : int(n_corners),
            'camera_matrix': K.tolist(),
            'dist_coeffs' : D.tolist(),
        }

        # samples.yaml 에 추가
        if os.path.exists(self.samples_yaml):
            with open(self.samples_yaml, 'r') as f:
                all_samples = yaml.safe_load(f) or []
        else:
            all_samples = []
        all_samples.append(sample)
        with open(self.samples_yaml, 'w') as f:
            yaml.dump(all_samples, f, allow_unicode=True, sort_keys=False)

        # annotated 이미지 저장
        if self.save_ann:
            img_path = os.path.join(self.session_dir, f'sample_{idx:03d}.png')
            cv2.imwrite(img_path, annotated)

        self._sample_count += 1
        x, y, z, rx, ry, rz = tcp
        t_m = tvec.flatten()
        msg_str = (
            f'샘플 #{idx} 저장 완료. '
            f'TCP=({x:.1f},{y:.1f},{z:.1f}mm, {rx:.2f},{ry:.2f},{rz:.2f}deg) | '
            f'board_t=({t_m[0]*1000:.1f},{t_m[1]*1000:.1f},{t_m[2]*1000:.1f}mm) | '
            f'corners={n_corners} | 총 {self._sample_count}개'
        )
        self.get_logger().info(msg_str)
        response.success = True
        response.message = msg_str
        return response

    def _reset_cb(self, request, response):
        with self._lock:
            old_count = self._sample_count
            self._sample_count = 0

        if os.path.exists(self.samples_yaml):
            os.remove(self.samples_yaml)

        msg_str = f'샘플 {old_count}개 삭제 완료. 세션 초기화됨.'
        self.get_logger().info(msg_str)
        response.success = True
        response.message = msg_str
        return response


def main(args=None):
    rclpy.init(args=args)
    node = SampleCollector()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
