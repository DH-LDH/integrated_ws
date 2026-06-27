import rclpy
from rclpy.node import Node
from srvs_pkg.srv import GetTargetPose
from vision_pkg.vision_6d_manager import (
    COMP_MODEL_PATH,
    DET_MODEL_PATH,
    SEG_MODEL_PATH,
    Vision6DPoseManager,
)


class VisionNodeDis(Node):
    def __init__(self):
        super().__init__('vision_node_dis')
        self.srv = self.create_service(GetTargetPose, '/get_target_pose_dis', self.get_pose_cb)
        self.get_logger().info('[VISION_DIS] 6D 앙상블 비전 초기화 중')

        self.declare_parameter('det_model_path',  DET_MODEL_PATH)
        self.declare_parameter('seg_model_path',  SEG_MODEL_PATH)
        self.declare_parameter('comp_model_path', COMP_MODEL_PATH)
        self.declare_parameter('visualize', False)

        self.vision     = None
        self.init_error = None
        try:
            self.vision = Vision6DPoseManager(
                logger=self.get_logger(),
                det_model_path=self.get_parameter('det_model_path').value,
                seg_model_path=self.get_parameter('seg_model_path').value,
                comp_model_path=self.get_parameter('comp_model_path').value,
                visualize=bool(self.get_parameter('visualize').value),
            )
            self.get_logger().info('[VISION_DIS] 초기화 완료 (6D 앙상블)')
        except Exception as e:
            self.init_error = str(e)
            self.get_logger().error(f'[VISION_DIS] 초기화 실패: {e}')

    def get_pose_cb(self, request, response):
        target_str = request.target_color.strip()
        self.get_logger().info(f'[VISION_DIS] 서비스 요청: target ID={target_str}')

        try:
            if self.vision is None:
                response.success = False
                self.get_logger().error(f'[VISION_DIS] 사용 불가: {self.init_error}')
                return response

            if not target_str.isdigit():
                response.success = False
                self.get_logger().error(f'[VISION_DIS] 잘못된 입력 (숫자 ID 필요): {target_str}')
                return response

            target_id = int(target_str)
            result    = self.vision.run_pipeline_by_id(target_id=target_id)

            if result.success:
                response.success    = True
                response.x          = float(result.x_m)
                response.y          = float(result.y_m)
                response.z          = float(result.z_m)
                response.yaw        = float(result.yaw_deg)
                response.class_name = str(result.class_name)
                response.layer      = int(result.layer) if result.layer is not None else 0
                self.get_logger().info(
                    f'[VISION_DIS] 타겟 발견: ID={result.target_id} class={result.class_name} '
                    f'x={result.x_m*1000:.1f}mm y={result.y_m*1000:.1f}mm '
                    f'z={result.z_m*1000:.1f}mm yaw={result.yaw_deg:.1f}deg layer={result.layer}'
                )
            else:
                response.success = False
                self.get_logger().error(
                    f'[VISION_DIS] 탐색 실패: ID={result.target_id} '
                    f'class={result.class_name} reason={result.reason}'
                )

        except Exception as e:
            self.get_logger().error(f'[VISION_DIS] 오류: {e}')
            response.success = False

        return response

    def destroy_node(self):
        if self.vision is not None:
            self.vision.shutdown()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VisionNodeDis()
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
