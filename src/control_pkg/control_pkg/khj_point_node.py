"""
===============================================================================
File        : khj_point_node.py
Package     : control_pkg
Author      : Jeho Yoon, Chaerin Seong, Dahan Lee, Donghyuk Jeong, Deokhui Han, Donggil Lee
Advisor     : Hyeongjin Kim
Created     : 2026-06-30
Environment : Ubuntu 22.04, ROS2 Humble, Python 3.10

Description
-----------
Aggregation node that merges /target_id_map and /birdseye_assembly/object_positions
by matching object IDs, then publishes the combined result (class_name, dist_cm,
offset_cm) to /khj_point for downstream assembly planning.

Main Features
-------------
- Subscribes to /target_id_map and /birdseye_assembly/object_positions
- Matches objects by ID and merges class_name + offset_cm data
- Publishes merged result to /khj_point as JSON string
- Camera offset correction: birdseye camera is +14 cm in robot1 X direction

Required Nodes
--------------
- birdseye_assembly node : /birdseye_assembly/object_positions
- target_id_map publisher : /target_id_map

Notes
-----
Published format:
  {"1": {"class_name": "block", "dist_cm": 15.2, "offset_cm": {"x": -29.1, "y": 6.9}}, ...}

Revision History
----------------

===============================================================================
"""

import json
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

ID_MAP_TOPIC = "/target_id_map"
BIRD_TOPIC   = "/birdseye_assembly/object_positions"
OUTPUT_TOPIC = "/khj_point"
PUBLISH_HZ   = 0.5   # seconds

# 버드아이뷰 probe(빨간 점) 기준 robot1 카메라 위치 (robot1 글로벌 좌표계)
# 카메라는 probe에서 Y 음수 방향으로 14cm
CAMERA_OFFSET_X_CM =   0.0
CAMERA_OFFSET_Y_CM = -10.0


class KhjPointNode(Node):
    def __init__(self):
        super().__init__("khj_point_node")

        self._id_map: dict   = {}   # {"1": "block", "2": "component"}
        self._bird_objs: dict = {}  # {"1": {"x": dx_cm, "y": dy_cm}}

        self.create_subscription(String, ID_MAP_TOPIC, self._id_map_cb, 10)
        self.create_subscription(String, BIRD_TOPIC,   self._bird_cb,   10)

        self._pub   = self.create_publisher(String, OUTPUT_TOPIC, 10)
        self._timer = self.create_timer(PUBLISH_HZ, self._publish)

        self.get_logger().info(f"[KHJ] sub : {ID_MAP_TOPIC}")
        self.get_logger().info(f"[KHJ] sub : {BIRD_TOPIC}")
        self.get_logger().info(f"[KHJ] pub : {OUTPUT_TOPIC} @ {PUBLISH_HZ}s")
        self.get_logger().info(
            f"[KHJ] 카메라 offset: x={CAMERA_OFFSET_X_CM} cm, y={CAMERA_OFFSET_Y_CM} cm"
        )

    def _id_map_cb(self, msg: String):
        try:
            self._id_map = json.loads(msg.data)
        except Exception as e:
            self.get_logger().warn(f"[KHJ] id_map 파싱 실패: {e}")

    def _bird_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
            self._bird_objs = {
                str(obj["id"]): obj["offset_cm"]
                for obj in data.get("objects", [])
                if "offset_cm" in obj
            }
        except Exception as e:
            self.get_logger().warn(f"[KHJ] bird 파싱 실패: {e}")

    @staticmethod
    def _dist_from_camera(offset_cm: dict) -> float:
        dx = offset_cm.get("x", 0.0) - CAMERA_OFFSET_X_CM
        dy = offset_cm.get("y", 0.0) - CAMERA_OFFSET_Y_CM
        return round(math.sqrt(dx * dx + dy * dy), 2)

    def _publish(self):
        if not self._id_map or not self._bird_objs:
            self.get_logger().warn(
                f"[KHJ] 발행 스킵: id_map={len(self._id_map)}개, bird_objs={len(self._bird_objs)}개",
                throttle_duration_sec=5.0,
            )
            return

        matched = {}
        for id_str, class_name in self._id_map.items():
            if id_str in self._bird_objs:
                offset = self._bird_objs[id_str]
                matched[id_str] = {
                    "class_name": class_name,
                    "dist_cm":    self._dist_from_camera(offset),
                    "offset_cm":  {"x":  round(offset.get("x", 0.0), 2),
                                   "y":  round(offset.get("y", 0.0) - CAMERA_OFFSET_Y_CM, 2)},
                }

        if matched:
            self._pub.publish(String(data=json.dumps(matched, ensure_ascii=False)))
        else:
            self.get_logger().warn(
                f"[KHJ] 매칭 없음: id_map 키={list(self._id_map.keys())}, "
                f"bird_objs 키={list(self._bird_objs.keys())}",
                throttle_duration_sec=5.0,
            )


def main(args=None):
    rclpy.init(args=args)
    node = KhjPointNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
