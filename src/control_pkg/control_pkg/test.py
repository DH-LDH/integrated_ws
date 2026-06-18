import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import math

class JointTesterNode(Node):
    def __init__(self):
        super().__init__('joint_tester_node')
        
        # ⚠️ 만약 로봇이 안 움직이면 'ros2 topic list'를 쳐서 
        # 실제 사용 중인 trajectory 토픽 이름으로 아래 문자열을 바꿔주세요!
        self.publisher_ = self.create_publisher(
            JointTrajectory, 
            '/arm_controller/joint_trajectory', 
            10)
        
        # 노드 켜지고 1초 뒤에 딱 한 번 퍼블리시
        self.timer = self.create_timer(1.0, self.publish_joint_angles)
        self.done = False

    def publish_joint_angles(self):
        if self.done: return
        self.done = True

        msg = JointTrajectory()
        # RB3-730E의 기본 조인트 이름 (세팅에 따라 다를 수 있으니 필요시 수정)
        msg.joint_names = ['joint0', 'joint1', 'joint2', 'joint3', 'joint4', 'joint5']
        
        # ==========================================================
        # 🔥 여기서 각도를 직접 수정하며 테스트하세요! (단위: Degree)
        # ==========================================================
        j1_deg = 0.0   # Joint 1 (Base)
        j2_deg = 0.0   # Joint 2 (Shoulder)
        j3_deg = -90.0 # Joint 3 (Elbow) - 예시: -90도
        j4_deg = 0.0   # Joint 4 (Wrist 1)
        j5_deg = 90.0  # Joint 5 (Wrist 2) - 예시: 90도
        j6_deg = 0.0   # Joint 6 (Wrist 3)
        # ==========================================================

        point = JointTrajectoryPoint()
        # 입력한 Degree 각도를 ROS 표준인 Radian으로 자동 변환
        point.positions = [
            math.radians(j1_deg),
            math.radians(j2_deg),
            math.radians(j3_deg),
            math.radians(j4_deg),
            math.radians(j5_deg),
            math.radians(j6_deg)
        ]
        
        # 3초 동안 부드럽게 이동 (너무 확 움직이면 위험하므로)
        point.time_from_start = Duration(sec=3, nanosec=0)

        msg.points.append(point)
        self.publisher_.publish(msg)
        
        self.get_logger().info(f'🚀 다음 각도로 이동 명령을 쐈습니다!')
        self.get_logger().info(f'[ {j1_deg}°, {j2_deg}°, {j3_deg}°, {j4_deg}°, {j5_deg}°, {j6_deg}° ]')

def main():
    rclpy.init()
    node = JointTesterNode()
    
    # 퍼블리시가 완료될 때까지 잠깐 대기 후 종료
    while not node.done and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
        
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()