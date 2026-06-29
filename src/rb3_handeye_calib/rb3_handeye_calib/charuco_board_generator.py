"""
===============================================================================
File        : charuco_board_generator.py
Package     : rb3_handeye_calib
Author      : Jeho Yoon, Chaerin Seong, Dahan Lee, Donghyuk Jeong, Deokhui Han, Donggil Lee
Created     : 2026-06-30
Environment : Ubuntu 22.04, ROS2 Humble, Python 3.10

Description
-----------
ROS2 node that generates a ChArUco calibration board and saves it as a PNG file.
Used as the first step in the hand-eye calibration pipeline.

Main Features
-------------
- Generates ChArUco board image with configurable grid size and marker dimensions
- Saves output as PNG at a specified path (default: /tmp/charuco_board.png)
- ROS parameters: squares_x, squares_y, square_length_mm, marker_length_mm, dictionary, dpi, output_path

Required Nodes
--------------
- None (standalone utility node)

Notes
-----
Usage: ros2 run rb3_handeye_calib charuco_board_generator \
         --ros-args -p squares_x:=7 -p squares_y:=5 \
         -p square_length_mm:=30.0 -p marker_length_mm:=22.0

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

from rb3_handeye_calib.charuco_utils import make_charuco_board


MM_PER_INCH = 25.4


class CharucoBoardGenerator(Node):
    def __init__(self):
        super().__init__('charuco_board_generator')

        self.declare_parameter('squares_x', 7)
        self.declare_parameter('squares_y', 5)
        self.declare_parameter('square_length_mm', 30.0)
        self.declare_parameter('marker_length_mm', 22.0)
        self.declare_parameter('dictionary', 'DICT_4X4_50')
        self.declare_parameter('dpi', 300)
        self.declare_parameter('output_path', '/tmp/charuco_board.png')

        squares_x = self.get_parameter('squares_x').value
        squares_y = self.get_parameter('squares_y').value
        sq_mm = self.get_parameter('square_length_mm').value
        mk_mm = self.get_parameter('marker_length_mm').value
        dictionary = self.get_parameter('dictionary').value
        dpi = int(self.get_parameter('dpi').value)
        output_path = self.get_parameter('output_path').value

        if mk_mm >= sq_mm:
            self.get_logger().error(
                f'marker_length_mm({mk_mm}) >= square_length_mm({sq_mm}). '
                'marker는 square보다 작아야 합니다.'
            )
            rclpy.shutdown()
            return

        self.get_logger().info(
            f'보드 생성 중: {squares_x}x{squares_y}, '
            f'square={sq_mm}mm, marker={mk_mm}mm, '
            f'{dictionary}, dpi={dpi}'
        )

        img = self._generate(
            squares_x, squares_y,
            sq_mm, mk_mm,
            dictionary, dpi,
        )

        os.makedirs(os.path.dirname(os.path.abspath(output_path)), exist_ok=True)
        cv2.imwrite(output_path, img)

        px_h, px_w = img.shape[:2]
        w_mm = px_w / dpi * MM_PER_INCH
        h_mm = px_h / dpi * MM_PER_INCH
        self.get_logger().info(
            f'보드 저장 완료: {output_path}\n'
            f'  해상도: {px_w}x{px_h} px  ({w_mm:.1f}x{h_mm:.1f} mm @ {dpi} dpi)\n'
            f'  A4 = 210x297mm 기준으로 여백 포함 여부를 확인하세요.\n'
            f'  실제 인쇄 후 ruler로 square_length_mm={sq_mm}mm 를 반드시 검증하세요.'
        )

        rclpy.shutdown()

    def _generate(
        self,
        squares_x: int,
        squares_y: int,
        sq_mm: float,
        mk_mm: float,
        dictionary_name: str,
        dpi: int,
    ) -> np.ndarray:
        # 단위: mm → m (make_charuco_board은 m 단위)
        board, aruco_dict = make_charuco_board(
            squares_x, squares_y,
            sq_mm / 1000.0,
            mk_mm / 1000.0,
            dictionary_name,
        )

        # 픽셀 크기 계산 (패딩 포함)
        px_per_mm = dpi / MM_PER_INCH
        board_w_px = int(round(squares_x * sq_mm * px_per_mm))
        board_h_px = int(round(squares_y * sq_mm * px_per_mm))

        # 최소 A4 여백: 상하좌우 5mm
        pad_px = int(round(5.0 * px_per_mm))
        img_w = board_w_px + 2 * pad_px
        img_h = board_h_px + 2 * pad_px

        # generateImage: size는 (width, height)
        board_img = board.generateImage((board_w_px, board_h_px), marginSize=0, borderBits=1)

        # 흰 캔버스에 패딩 추가
        canvas = np.full((img_h, img_w), 255, dtype=np.uint8)
        canvas[pad_px:pad_px + board_h_px, pad_px:pad_px + board_w_px] = board_img

        return canvas


def main(args=None):
    rclpy.init(args=args)
    node = CharucoBoardGenerator()
    # 생성 완료 후 shutdown 호출됨 → spin 불필요
    # 단 rclpy.ok()가 False면 spin이 바로 종료됨
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
