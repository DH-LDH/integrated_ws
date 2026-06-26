"""
/target_id_map      : {"1": "block", "2": "component", ...}
/birdseye_assembly/object_positions
                    : {"objects": [{"id":1, "offset_cm":{"x":..,"y":..}, ...}, ...]}

두 토픽에서 id가 일치하는 항목의 class_name, 카메라 기준 dist_cm, offset_cm 을 묶어
/khj_point 로 발행한다.

카메라는 버드아이뷰 중앙점(probe_bird)보다 robot1 X 양수 방향 14cm.
버드아이 offset_cm 기준: camera = center + (x=+14 cm, y=0 cm)

발행 형식:
{
  "1": {"class_name": "block",     "dist_cm": 15.2, "offset_cm": {"x": -29.1, "y": 6.9}},
  "2": {"class_name": "component", "dist_cm": 22.1, "offset_cm": {"x": -18.0, "y": -4.3}}
}
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
CAMERA_OFFSET_Y_CM = -14.0


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
            return

        matched = {}
        for id_str, class_name in self._id_map.items():
            if id_str in self._bird_objs:
                offset = self._bird_objs[id_str]
                matched[id_str] = {
                    "class_name": class_name,
                    "dist_cm":    self._dist_from_camera(offset),
                    "offset_cm":  {"x": -round(offset.get("x", 0.0), 2),   # x 부호 반전
                                   "y":  round(offset.get("y", 0.0), 2)},
                }

        if matched:
            self._pub.publish(String(data=json.dumps(matched, ensure_ascii=False)))


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
