import json
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from srvs_pkg.srv import GetTargetPose

UPSTREAM_SRV = "/get_target_pose"
MY_SRV       = "/get_target_pose_mapped"
MAP_TOPIC    = "/target_id_map"
PUBLISH_HZ   = 0.5   # seconds
SRV_TIMEOUT  = 10.0  # seconds


class TargetIdMapperNode(Node):
    def __init__(self):
        super().__init__("target_id_mapper")

        self._cbg = ReentrantCallbackGroup()

        self._client = self.create_client(
            GetTargetPose, UPSTREAM_SRV, callback_group=self._cbg
        )
        self._srv = self.create_service(
            GetTargetPose, MY_SRV, self._srv_cb, callback_group=self._cbg
        )
        self._pub   = self.create_publisher(String, MAP_TOPIC, 10)
        self._timer = self.create_timer(PUBLISH_HZ, self._publish_map, callback_group=self._cbg)

        # key: "target_color__target_size"  value: {x, class_name}
        self._entries: dict = {}

        self.get_logger().info(f"[MAPPER] proxy : {MY_SRV} → {UPSTREAM_SRV}")
        self.get_logger().info(f"[MAPPER] topic : {MAP_TOPIC} @ {PUBLISH_HZ}s")

    def _srv_cb(self, request, response):
        if not self._client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error(f"[MAPPER] {UPSTREAM_SRV} 서비스 없음")
            response.success = False
            return response

        future = self._client.call_async(request)

        deadline = time.time() + SRV_TIMEOUT
        while not future.done():
            if time.time() > deadline:
                self.get_logger().error("[MAPPER] vision_node 응답 타임아웃")
                response.success = False
                return response
            time.sleep(0.01)

        res = future.result()
        if res is None:
            response.success = False
            return response

        if res.success:
            key = f"{request.target_color}__{request.target_size}"
            self._entries[key] = {"x": res.x, "class_name": res.class_name}
            self.get_logger().info(
                f"[MAPPER] {key} x={res.x:.3f}m class={res.class_name}"
            )

        response.success    = res.success
        response.x          = res.x
        response.y          = res.y
        response.z          = res.z
        response.yaw        = res.yaw
        response.class_name = res.class_name
        response.layer      = res.layer
        return response

    def _publish_map(self):
        if not self._entries:
            return
        sorted_entries = sorted(self._entries.values(), key=lambda e: e["x"])
        payload = {str(i + 1): e["class_name"] for i, e in enumerate(sorted_entries)}
        self._pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))


def main(args=None):
    rclpy.init(args=args)
    node = TargetIdMapperNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
