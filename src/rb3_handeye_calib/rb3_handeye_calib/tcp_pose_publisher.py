"""
===============================================================================
File        : tcp_pose_publisher.py
Package     : rb3_handeye_calib
Author      : Jeho Yoon, Chaerin Seong, Dahan Lee, Donghyuk Jeong, Deokhui Han, Donggil Lee
Created     : 2026-06-30
Environment : Ubuntu 22.04, ROS2 Humble, Python 3.10

Description
-----------
ROS2 node that reads the TCP pose from a Rainbow Robotics RB3 cobot via
rbpodo and publishes it as ROS2 topics for use in hand-eye calibration.

Main Features
-------------
- Publishes /<robot_name>/tcp_pose_array (Float64MultiArray): [x_mm, y_mm, z_mm, rx, ry, rz deg]
- Publishes /<robot_name>/tcp_pose (PoseStamped): position in meters, orientation as quaternion
- Dual fallback TCP read: CobotData.request_data() → SystemVariable
- ROS parameters: robot_ip, robot_name, publish_rate, frame_id, euler_order

Required Nodes
--------------
- rbpodo Cobot driver (robot connection)

Notes
-----
- Default robot IP: 10.0.2.7, publish rate: 20 Hz
- Falls back to SystemVariable read if CobotData fails

Revision History
----------------

===============================================================================
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseStamped

import numpy as np

from rb3_handeye_calib.transform_utils import (
    rpy_deg_to_rotation_matrix,
    rotation_matrix_to_quaternion,
)


class TcpPosePublisher(Node):
    def __init__(self):
        super().__init__('tcp_pose_publisher')

        # ── 파라미터 ────────────────────────────────────────────────────────
        self.declare_parameter('robot_ip', '10.0.2.7')
        self.declare_parameter('robot_name', 'robot1')
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('frame_id', 'base')
        self.declare_parameter('euler_order', 'xyz')

        self.robot_ip = self.get_parameter('robot_ip').value
        self.robot_name = self.get_parameter('robot_name').value
        rate = self.get_parameter('publish_rate').value
        self.frame_id = self.get_parameter('frame_id').value
        self.euler_order = self.get_parameter('euler_order').value

        # ── Publishers ─────────────────────────────────────────────────────
        ns = f'/{self.robot_name}'
        self.pub_array = self.create_publisher(
            Float64MultiArray, f'{ns}/tcp_pose_array', 10
        )
        self.pub_pose = self.create_publisher(
            PoseStamped, f'{ns}/tcp_pose', 10
        )

        # ── rbpodo 연결 ─────────────────────────────────────────────────────
        self.robot = None
        self.rc = None
        self.cobot_data = None
        self._tcp_read_mode = None   # 'cobot_data' | 'sysvar' | None

        self._connect_robot()

        # ── 타이머 ─────────────────────────────────────────────────────────
        period = 1.0 / max(rate, 0.1)
        self.timer = self.create_timer(period, self._publish_cb)

        self.get_logger().info(
            f'[TcpPosePublisher] robot={self.robot_name} ip={self.robot_ip} '
            f'rate={rate:.1f}Hz frame_id={self.frame_id} '
            f'euler_order={self.euler_order}'
        )
        self.get_logger().info(
            f'  → gripper_tcp frame publish to {ns}/tcp_pose_array, {ns}/tcp_pose'
        )

    # ── 로봇 연결 ────────────────────────────────────────────────────────────

    def _connect_robot(self):
        try:
            import rbpodo as rb
        except ImportError:
            self.get_logger().error(
                'rbpodo를 import할 수 없습니다. '
                'pip install rbpodo 또는 환경을 확인하세요.'
            )
            return

        try:
            self.robot = rb.Cobot(self.robot_ip)
            self.rc = rb.ResponseCollector()
            self.robot.set_operation_mode(self.rc, rb.OperationMode.Real)
            self.get_logger().info(f'rbpodo Cobot 연결 완료: {self.robot_ip}')
        except Exception as e:
            self.get_logger().error(f'Cobot 연결 실패: {e}')
            return

        # CobotData 방식 시도
        # request_data() 가 SystemState 를 직접 반환한다 (None 이면 실패)
        try:
            self.cobot_data = rb.CobotData(self.robot_ip)
            state = self.cobot_data.request_data()
            if state is None:
                raise RuntimeError('request_data() 가 None 반환 (타임아웃 또는 연결 실패)')
            _ = state.sdata.tcp_pos
            self._tcp_read_mode = 'cobot_data'
            self.get_logger().info('TCP 읽기 방식: CobotData.request_data() → state.sdata.tcp_pos')
        except Exception as e:
            self.get_logger().warn(
                f'CobotData 초기화 실패: {e}\n'
                '  → SystemVariable 방식으로 fallback 시도합니다.'
            )
            self._try_sysvar_mode()

    def _try_sysvar_mode(self):
        """SystemVariable 방식으로 TCP 읽기 시도."""
        try:
            import rbpodo as rb
            # get_system_variable 은 (ReturnType, float) 튜플 반환
            ret, val = self.robot.get_system_variable(self.rc, rb.SystemVariable.SD_TCP_X)
            self._tcp_read_mode = 'sysvar'
            self.get_logger().info('TCP 읽기 방식: SystemVariable (rb.SystemVariable.SD_TCP_X 등)')
        except Exception as e:
            self.get_logger().warn(
                f'SystemVariable 방식도 실패: {e}\n'
                '  → TCP publish를 할 수 없습니다. rbpodo API를 확인하세요.'
            )
            self._tcp_read_mode = None

    # ── TCP 읽기 ─────────────────────────────────────────────────────────────

    def _read_tcp(self):
        """
        TCP pose를 읽어 [x_mm, y_mm, z_mm, rx_deg, ry_deg, rz_deg] 반환.
        실패 시 None.
        """
        if self.robot is None:
            return None

        if self._tcp_read_mode == 'cobot_data':
            return self._read_tcp_cobot_data()
        elif self._tcp_read_mode == 'sysvar':
            return self._read_tcp_sysvar()
        else:
            return None

    def _read_tcp_cobot_data(self):
        """CobotData 방식으로 TCP 읽기."""
        try:
            state = self.cobot_data.request_data()
            if state is None:
                self.get_logger().warn('CobotData.request_data() 타임아웃')
                return None
            tcp = state.sdata.tcp_pos   # [x_mm, y_mm, z_mm, rx_deg, ry_deg, rz_deg]
            return list(tcp)
        except Exception as e:
            self.get_logger().warn(f'CobotData read 실패: {e}')
            return None

    def _read_tcp_sysvar(self):
        """SystemVariable 방식으로 TCP 읽기."""
        try:
            import rbpodo as rb
            sv = rb.SystemVariable
            # get_system_variable 은 (ReturnType, float) 반환
            _, x  = self.robot.get_system_variable(self.rc, sv.SD_TCP_X)
            _, y  = self.robot.get_system_variable(self.rc, sv.SD_TCP_Y)
            _, z  = self.robot.get_system_variable(self.rc, sv.SD_TCP_Z)
            _, rx = self.robot.get_system_variable(self.rc, sv.SD_TCP_RX)
            _, ry = self.robot.get_system_variable(self.rc, sv.SD_TCP_RY)
            _, rz = self.robot.get_system_variable(self.rc, sv.SD_TCP_RZ)
            return [float(x), float(y), float(z), float(rx), float(ry), float(rz)]
        except Exception as e:
            self.get_logger().warn(f'SystemVariable read 실패: {e}')
            return None

    # ── Publish 콜백 ─────────────────────────────────────────────────────────

    def _publish_cb(self):
        pose_mm_deg = self._read_tcp()
        if pose_mm_deg is None:
            return

        if len(pose_mm_deg) != 6:
            self.get_logger().warn(
                f'TCP pose 길이 오류: {len(pose_mm_deg)} (예상 6)'
            )
            return

        x_mm, y_mm, z_mm, rx_deg, ry_deg, rz_deg = pose_mm_deg

        # ── Float64MultiArray ──────────────────────────────────────────────
        arr_msg = Float64MultiArray()
        arr_msg.data = [float(x_mm), float(y_mm), float(z_mm), float(rx_deg), float(ry_deg), float(rz_deg)]
        self.pub_array.publish(arr_msg)

        # ── PoseStamped ────────────────────────────────────────────────────
        R = rpy_deg_to_rotation_matrix(rx_deg, ry_deg, rz_deg, self.euler_order)
        q = rotation_matrix_to_quaternion(R)   # [x, y, z, w]

        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = self.frame_id
        pose_msg.pose.position.x = x_mm / 1000.0   # mm → m
        pose_msg.pose.position.y = y_mm / 1000.0
        pose_msg.pose.position.z = z_mm / 1000.0
        pose_msg.pose.orientation.x = q[0]
        pose_msg.pose.orientation.y = q[1]
        pose_msg.pose.orientation.z = q[2]
        pose_msg.pose.orientation.w = q[3]
        self.pub_pose.publish(pose_msg)


def main(args=None):
    rclpy.init(args=args)
    node = TcpPosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
