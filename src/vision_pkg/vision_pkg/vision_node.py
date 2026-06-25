# vision_node.py
import time
import rclpy
from rclpy.node import Node
from srvs_pkg.srv import GetTargetPose
from vision_pkg import INUVisionCall as ivc

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        self.srv = self.create_service(GetTargetPose, '/get_target_pose', self.get_pose_cb)
        self.get_logger().info('[VISION] 초기화 중... VisionManager 로드')
        
        self.vision = ivc.VisionManager()
        self.get_logger().info('[VISION] RealSense 카메라 지속 스트리밍 시작 중...')
        self.vision.start_camera(mode="mid_50", V_visualize=False)
        self.get_logger().info('[VISION] vision_node 시작 완료 (카메라 지속 구동 모드)')


    def destroy_node(self):
        if hasattr(self, 'vision'):
            self.vision.close()
        super().destroy_node()


    def get_pose_cb(self, request, response):
        t_total = time.perf_counter()
        target_str = request.target_color.strip()
        self.get_logger().info(f'[VISION] 서비스 요청 수신 - target ID: {target_str}')

        try:
            if not target_str.isdigit():
                self.get_logger().error(
                    f'[VISION] 잘못된 입력입니다. 숫자 ID를 입력하세요: {target_str}'
                )
                response.success = False
                self.get_logger().info(
                    f'[TIME] node.service_cb.total: {time.perf_counter() - t_total:.3f}s'
                )
                return response

            target_id = int(target_str)

            t_step = time.perf_counter()
            result = self.vision.run_pipeline_by_id(
                target_id=target_id,
                local_id=0,
                camera_mode="mid_50",
                brick_search_mode="fine",
                V_visualize_capture=False,
                V_visualize_search=False 
            )
            self.get_logger().info(
                f'[TIME] node.run_pipeline_by_id: {time.perf_counter() - t_step:.3f}s'
            )

            if result["success"]:
                response.success = True

                # 내부 단위: mm
                # ROS 응답 단위: m
                response.x = float(result["x_mm"] / 1000.0)
                response.y = float(result["y_mm"] / 1000.0)
                response.z = float(result["z_mm"] / 1000.0)
                response.yaw = float(result["yaw_deg"] - 90.0 ) 
                response.class_name = str(result["class_name"])

                self.get_logger().info(
                    f'[VISION] 타겟 발견! '
                    f'ID={result["target_id"]}, '
                    f'Class={result["class_name"]}, '
                    f'X={result["x_mm"]:.1f}mm, '
                    f'Y={result["y_mm"]:.1f}mm, '
                    f'Z={result["z_mm"]:.1f}mm, '
                    f'Yaw={result["yaw_deg"]:.2f}deg'
                )

            else:
                response.success = False
                self.get_logger().error(
                    f'[VISION] 타겟 탐색 실패: '
                    f'ID={result.get("target_id")}, '
                    f'Class={result.get("class_name")}, '
                    f'Reason={result.get("reason")}'
                )

        except Exception as e:
            self.get_logger().error(f'[VISION] 처리 중 심각한 오류 발생: {e}')
            response.success = False

        self.get_logger().info(
            f'[TIME] node.service_cb.total: {time.perf_counter() - t_total:.3f}s'
        )
        return response


def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
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
