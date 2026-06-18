# # import rclpy
# # from rclpy.node import Node
# # from srvs_pkg.srv import GetTargetPose
# # from std_srvs.srv import SetBool, Trigger
# # import time
# # import math

# # class MasterNode(Node):
# #     def __init__(self):
# #         super().__init__('master_node')
# #         self.cli_v = self.create_client(GetTargetPose, '/get_target_pose')
# #         self.cli_r = self.create_client(GetTargetPose, '/robot1/robot_move_step')
# #         self.cli_g = self.create_client(SetBool, '/control_gripper')
# #         self.cli_h = self.create_client(Trigger, '/robot1/robot_home')
       
# #         self.Z_OFF = -95.0
# #         self.Z_MARGIN = 20.0
# #         self.BLOCK_H = 20.0
# #         self.WAIT_TIME = 1.5
# #         self.PRE_XY_LOWER = 70.0
       
# #         self.STUD_PITCH = 0.016
# #         self.YAW_TUNE = 0.0

# #         self.CLASS_TO_TARGET_ID = {
# #             "2x2_red": "1",
# #             "2x2_green": "2",
# #             "2x2_blue": "3",
# #             "2x2_yellow": "4",
# #             "4x2_red": "5",
# #             "4x2_green": "6",
# #             "4x2_blue": "7",
# #             "4x2_yellow": "8",
# #             "2x4_red": "5",
# #             "2x4_green": "6",
# #             "2x4_blue": "7",
# #             "2x4_yellow": "8",
# #             "assembly": "999",
# #         }
       
# #         self.last_perfect_pose = None

# #     def call(self, cli, req):
# #         while not cli.wait_for_service(timeout_sec=1.0):
# #             self.get_logger().info(f'Waiting for {cli.srv_name}...')
# #         future = cli.call_async(req)
# #         rclpy.spin_until_future_complete(self, future)
# #         return future.result()

# #     def to_vision_target_id(self, target):
# #         target = str(target).strip()
# #         for prefix in ("count_", "far_"):
# #             if target.startswith(prefix):
# #                 target = target[len(prefix):]

# #         if target.isdigit():
# #             return target

# #         target_id = self.CLASS_TO_TARGET_ID.get(target)
# #         if target_id is None:
# #             self.get_logger().error(f"❌ vision_node.py ID 매핑 없음: {target}")
# #             return target
# #         return target_id

# #     def request_target_pose(self, target):
# #         return self.call(
# #             self.cli_v,
# #             GetTargetPose.Request(target_color=self.to_vision_target_id(target))
# #         )

# #     def find_target_with_retry(self, color):
# #         p = self.request_target_pose(color)
# #         if p.success:
# #             return p
# #         self.get_logger().error(f"❌ [{color}] 타겟 인식 실패")
# #         return None

# #     def normalize_yaw(self, yaw):
# #         while yaw > 90.0:
# #             yaw -= 180.0
# #         while yaw < -90.0:
# #             yaw += 180.0
# #         return yaw

# #     def is_2x2_pose(self, pose):
# #         return str(getattr(pose, "class_name", "")).startswith("2x2_")

# #     def fold_2x2_yaw(self, yaw):
# #         yaw = self.normalize_yaw(yaw)
# #         if yaw > 45.0:
# #             yaw -= 90.0
# #         elif yaw < -45.0:
# #             yaw += 90.0
# #         return yaw

# #     def pose_yaw_for_xy_offset(self, pose):
# #         if self.is_2x2_pose(pose):
# #             return self.fold_2x2_yaw(pose.yaw)
# #         return pose.yaw

# #     def calc_target_xy(self, pose, offset_studs_x=0.0, offset_studs_y=0.0):
# #         dx = offset_studs_x * self.STUD_PITCH
# #         dy = offset_studs_y * self.STUD_PITCH
# #         yaw_rad = math.radians(self.pose_yaw_for_xy_offset(pose))
# #         real_offset_x = dx * math.cos(yaw_rad) - dy * math.sin(yaw_rad)
# #         real_offset_y = dx * math.sin(yaw_rad) + dy * math.cos(yaw_rad)
# #         return pose.x + real_offset_x, pose.y + real_offset_y

# #     def move_fast_from_pose(
# #         self,
# #         pose,
# #         layer_index=0,
# #         yaw_offset=0.0,
# #         offset_studs_x=0.0,
# #         offset_studs_y=0.0,
# #         pre_xy_lower=False,  # (호환용) 더 이상 사용 안 함: Z 선하강/중간 재촬영 제거됨
# #     ):
# #         # AMR 탑재 로봇팔 load_node 방식:
# #         # HOME에서 1회 측정한 pose 하나로 YAW + XY + Z 접근을 한 모션에 합친다.
# #         # (Z 7cm 선하강 / 중간 재촬영 모두 제거)
# #         target_x, target_y = self.calc_target_xy(pose, offset_studs_x, offset_studs_y)
# #         if self.is_2x2_pose(pose):
# #             target_yaw = self.fold_2x2_yaw(pose.yaw + yaw_offset + self.YAW_TUNE)
# #             self.get_logger().info(
# #                 f"🔄 [2x2 YAW FOLD] {pose.class_name} vision_yaw={pose.yaw:.1f}도 -> wrist_yaw={target_yaw:.1f}도"
# #             )
# #         else:
# #             target_yaw = self.normalize_yaw(pose.yaw + yaw_offset + self.YAW_TUNE)

# #         z_move = (pose.z * 1000.0 + self.Z_OFF) - (self.BLOCK_H * layer_index)

# #         # [1] YAW + XY + Z 동시 이동 (물체 바로 위 Z_MARGIN 지점까지 대각선 접근)
# #         self.get_logger().info(
# #             f"➡️ [ONE-SHOT] x={target_x:.4f}m, y={target_y:.4f}m, "
# #             f"yaw={target_yaw:.1f}도, z_approach={z_move - self.Z_MARGIN:.1f}mm"
# #         )
# #         self.call(self.cli_r, GetTargetPose.Request(
# #             x=target_x, y=target_y, z=z_move - self.Z_MARGIN,
# #             yaw=target_yaw, target_size="APPROACH"
# #         ))
# #         time.sleep(self.WAIT_TIME)

# #         # [2] 최종 수직 접근 (yaw 회전 후에도 tool Z축은 수직 유지)
# #         self.call(self.cli_r, GetTargetPose.Request(z=self.Z_MARGIN, target_size="Z"))
# #         time.sleep(self.WAIT_TIME)

# #     def pick_target(self, color, layer_index=0, offset_studs_x=0.0, offset_studs_y=0.0):
# #         self.get_logger().info(f"\n--- PICK TARGET: [{color.upper()}] ---")
        
# #         p = self.find_target_with_retry(color)
# #         if not p: return False

# #         self.move_fast_from_pose(
# #             p,
# #             layer_index=layer_index,
# #             offset_studs_x=offset_studs_x,
# #             offset_studs_y=offset_studs_y,
# #         )

# #         self.call(self.cli_g, SetBool.Request(data=True))
# #         time.sleep(self.WAIT_TIME)
        
# #         self.call(self.cli_r, GetTargetPose.Request(z=-50.0, target_size="Z"))
# #         time.sleep(self.WAIT_TIME)
# #         return True

# #     def blind_insert(self, base_pose, layer_index, yaw_offset=0.0, release_gripper=True, offset_studs_x=0.0, offset_studs_y=0.0):
# #         self.get_logger().info(f"\n--- BLIND STACK (메모리 사용): Layer {layer_index} (Y Offset: {offset_studs_y}) ---")
# #         time.sleep(1.0)

# #         self.move_fast_from_pose(
# #             base_pose,
# #             layer_index=layer_index,
# #             yaw_offset=yaw_offset,
# #             offset_studs_x=offset_studs_x,
# #             offset_studs_y=offset_studs_y,
# #             pre_xy_lower=False,
# #         )

# #         if release_gripper:
# #             self.call(self.cli_g, SetBool.Request(data=False))
# #             time.sleep(self.WAIT_TIME)
            
# #         self.call(self.cli_r, GetTargetPose.Request(z=-50.0, target_size="Z"))
# #         time.sleep(self.WAIT_TIME)
# #         return True

# #     def visual_insert(
# #         self,
# #         target_color,
# #         layer_index,
# #         release_gripper=True,
# #         yaw_offset=0.0,
# #         # offset_studs_x=0.3125,
# #         # offset_studs_y=0.15625,
# #         offset_studs_x=0.0,
# #         offset_studs_y=0.0,
# #         pre_xy_lower=False,
# #     ):
# #         self.get_logger().info(f"\n--- VISUAL STACK: [{target_color.upper()}] (Layer +{layer_index}, Y Offset: {offset_studs_y}) ---")
# #         time.sleep(1.0)

# #         p = self.find_target_with_retry(target_color)
# #         if not p: return False
       
# #         self.last_perfect_pose = p

# #         self.move_fast_from_pose(
# #             p,
# #             layer_index=layer_index,
# #             yaw_offset=yaw_offset,
# #             offset_studs_x=offset_studs_x,
# #             offset_studs_y=offset_studs_y,
# #             pre_xy_lower=pre_xy_lower,
# #         )

# #         if release_gripper:
# #             self.call(self.cli_g, SetBool.Request(data=False))
# #             time.sleep(self.WAIT_TIME)
            
# #         self.call(self.cli_r, GetTargetPose.Request(z=-50.0, target_size="Z"))
# #         time.sleep(self.WAIT_TIME)
# #         return True

# #     def build_battery(self):
# #         self.get_logger().info("🔋 [배터리] 노란색(Pick) -> 파란색(Base)")
# #         if self.pick_target("2x2_yellow"):
# #             self.call(self.cli_h, Trigger.Request())
# #             self.visual_insert("2x2_blue", layer_index=1, pre_xy_lower=True)
# #             self.get_logger().info("✅ 배터리 조립 완료!")

# #     def build_magnet(self):
# #         self.get_logger().info("🧲 [자석] 파란색(Pick) -> 빨간색(Base)")
# #         if self.pick_target("2x2_blue"):
# #             self.call(self.cli_h, Trigger.Request())
# #             self.visual_insert("2x2_red", layer_index=1, pre_xy_lower=True)
# #             self.get_logger().info("✅ 자석 조립 완료!")

# #     def build_e_stop(self):
# #         self.get_logger().info("🛑 [비상정지] 빨간색(Pick) -> 노란색4x2(Base)")
# #         if self.pick_target("2x2_red"):
# #             self.call(self.cli_h, Trigger.Request())
# #             self.visual_insert("4x2_yellow", layer_index=1, yaw_offset=-90.0, pre_xy_lower=True)
# #             self.get_logger().info("✅ 비상정지 조립 완료!")

# #     def build_carrot(self):
# #         self.get_logger().info("🥕 [당근] 초록 (Pick) -> 노랑 (그리퍼 유지) -> 노란색(base)")
# #         if self.pick_target("2x2_green"):
# #             self.call(self.cli_h, Trigger.Request())
# #             if self.visual_insert("2x2_yellow", layer_index=0.9, pre_xy_lower=True,release_gripper=False):
# #                 self.call(self.cli_h, Trigger.Request())
# #                 if self.visual_insert("2x2_yellow", layer_index=1.8, pre_xy_lower=True):
# #                     self.get_logger().info("✅ 당근 완성!")

# #     def build_traffic_light(self):
# #         self.get_logger().info("🚦 [신호등] 노란색(Pick) -> 초록색(Base) -> 빨간색(Pick)")
# #         if self.pick_target("2x2_red"):
# #             self.call(self.cli_h, Trigger.Request())
# #             if self.visual_insert("2x2_yellow", layer_index=0.9, pre_xy_lower=True,release_gripper=False):
# #                 self.call(self.cli_h, Trigger.Request())
# #                 if self.visual_insert("2x2_green", layer_index=1.7, pre_xy_lower=True):
# #                     self.get_logger().info("✅ 신호등 완성!")

# #     def build_small_tree(self):
# #         self.get_logger().info("🌳 [작은 나무] 초록4x2(Pick) -> 노랑2x2(Base) -> 초록2x2(Pick)")
# #         if self.pick_target("2x2_green"):
# #             self.call(self.cli_h, Trigger.Request())
# #             if self.visual_insert("4x2_green", layer_index=0.8, pre_xy_lower=True, release_gripper=False):
# #                 self.call(self.cli_h, Trigger.Request())
# #                 if self.visual_insert("2x2_yellow", layer_index=1.8, pre_xy_lower=True):
# #                     self.get_logger().info("✅ 작은 나무 완성!")

# #     def build_hammer(self):
# #         self.get_logger().info("🔨 [망치] 빨강2x2(Pick) -> 빨강2x2(Base) -> 파랑4x2(Pick)")
# #         if self.pick_target("4x2_blue"):
# #             self.call(self.cli_h, Trigger.Request())
# #             if self.visual_insert("2x2_red", layer_index=0.9, pre_xy_lower=True, release_gripper=False):
# #                 self.call(self.cli_h, Trigger.Request())
# #                 if self.visual_insert("2x2_red", layer_index=1.8, yaw_offset=0.0, pre_xy_lower=True):
# #                     self.get_logger().info("✅ 망치 완성!")

# #     def build_big_carrot(self):
# #         self.get_logger().info("🥕🥕 [큰 당근] 노랑2x2(Pick) -> 노랑2x2(Base) -> 노랑4x2(Pick) -> 파랑2x2(Pick)")
# #         if self.pick_target("2x2_green"):
# #             self.call(self.cli_h, Trigger.Request())
# #             if self.visual_insert("4x2_yellow", layer_index=0.9, pre_xy_lower=True, release_gripper=False):
# #                 self.call(self.cli_h, Trigger.Request())
# #                 if self.visual_insert("2x2_yellow", layer_index=1.8, yaw_offset=0.0, pre_xy_lower=True, release_gripper=False):
# #                         self.call(self.cli_h, Trigger.Request())
# #                         if self.visual_insert("2x2_yellow", layer_index=2.7, pre_xy_lower=True, release_gripper=False):
# #                             self.get_logger().info("✅ 대왕 당근 완성!")

# #     # def build_burger(self):
# #     #     self.get_logger().info("🍔 [버거] 노랑4x2(Base) -> 빨강4x2(Offset Y -1) -> 빨강2x2(Offset Y +2) -> 노랑4x2(Top)")
# #     #     if self.pick_target("4x2_red"):
# #     #         self.call(self.cli_h, Trigger.Request())
# #     #         if self.visual_insert("4x2_yellow", layer_index=1, offset_studs_y=-1.0):
# #     #             saved_base_bun_pose = self.last_perfect_pose
# #     #             self.call(self.cli_h, Trigger.Request())
# #     #             if self.pick_target("2x2_red"):
# #     #                 self.call(self.cli_h, Trigger.Request())
# #     #                 if self.visual_insert("4x2_red", layer_index=0, yaw_offset=0.0, offset_studs_y=3.0):
# #     #                     self.call(self.cli_h, Trigger.Request())
# #     #                     if self.pick_target("4x2_yellow"):
# #     #                         self.call(self.cli_h, Trigger.Request())
# #     #                         if saved_base_bun_pose:
# #     #                             self.get_logger().info("🧠 [메모리 사용] 최초 바닥 빵의 좌표를 기억해서 정중앙에 덮습니다!")
# #     #                             self.blind_insert(saved_base_bun_pose, layer_index=2, offset_studs_y=1.0)
# #     #                             self.get_logger().info("✅ 버거 완성!")
# #     #                         else:
# #     #                             self.get_logger().warn("❌ 저장된 좌표가 없습니다. 조립 실패.")


# #     def build_burger(self):
# #         self.get_logger().info("🍔 [버거] 4x2_yellow(Pick) -> 4x2_red 결합 -> 2x2_red로 6x2 조립 -> 저장된 4x2_yellow에 최종 결합")

# #         self.get_logger().info("🧱 [Phase 1] 4x2_yellow 파지")
# #         if self.pick_target("4x2_yellow", offset_studs_y=0):
# #             self.call(self.cli_h, Trigger.Request())

# #             self.get_logger().info("🧱 [Phase 2] 바닥의 다른 4x2_yellow 위치 스캔 및 기억")
# #             p_yellow = self.find_target_with_retry("4x2_yellow")
# #             if not p_yellow:
# #                 self.get_logger().warn("❌ 바닥에 다른 4x2_yellow가 안 보입니다. 조립을 취소합니다.")
# #                 return
# #             saved_yellow_pose = p_yellow
# #             self.call(self.cli_h, Trigger.Request())

# #             self.get_logger().info("🧱 [Phase 3] 들고 있는 4x2_yellow -> 4x2_red 결합 (그리퍼 유지)")
# #             if self.visual_insert("4x2_red", layer_index=0.8, offset_studs_y=-1.0, release_gripper=False):
# #                 self.call(self.cli_h, Trigger.Request())
# #                 time.sleep(1.0) 

# #                 self.get_logger().info("🧱 [Phase 4] 바닥의 2x2_red 스캔 및 6x2 조립 (그리퍼 유지)")
# #                 p_2x2_base = self.find_target_with_retry("2x2_red")
# #                 if not p_2x2_base:
# #                     self.get_logger().warn("❌ 바닥에 2x2_red가 안 보입니다.")
# #                     return
# #                 saved_6x2_pose = p_2x2_base

# #                 if self.blind_insert(saved_6x2_pose, layer_index=0.8, offset_studs_y=2.0, release_gripper=False):
# #                     self.call(self.cli_h, Trigger.Request())

# #                     self.get_logger().info("🧱 [Phase 5] 덩어리를 처음 저장한 4x2_yellow에 최종 결합")
# #                     if self.blind_insert(saved_yellow_pose, layer_index=1.9, offset_studs_y=0.0):
# #                         self.get_logger().info("✅ 버거 완성!")


# #     def build_ice_cream(self):
# #         self.get_logger().info("🍦 [아이스크림] 모듈형 조립 전략")
# #         self.get_logger().info("[Phase 1] 하단 조립: 노랑4x2(Pick) -> 노랑2x2(Base)")
# #         if self.pick_target("4x2_yellow"):
# #             self.call(self.cli_h, Trigger.Request())
# #             if not self.visual_insert("2x2_yellow", layer_index=1):
# #                 self.get_logger().warn("❌ 하단 모듈 조립 실패")
# #                 return

# #         self.get_logger().info("[Phase 2-1] 파랑2x2(Pick) -> 빨강2x2 옆에 배치 (바닥)")
# #         self.call(self.cli_h, Trigger.Request())
# #         if self.pick_target("2x2_blue"):
# #             self.call(self.cli_h, Trigger.Request())
# #             if not self.visual_insert("2x2_red", layer_index=0, offset_studs_y=2.0):
# #                 self.get_logger().warn("❌ 파란색 블록 배치 실패")
# #                 return

# #         self.get_logger().info("[Phase 2-2] 노랑2x2(Pick) -> 파랑2x2(Base, offset_y=-1)로 결합")
# #         self.call(self.cli_h, Trigger.Request())
# #         if self.pick_target("2x2_green"):
# #             self.call(self.cli_h, Trigger.Request())
# #             if not self.visual_insert("2x2_blue", layer_index=1, offset_studs_y=-1.0, release_gripper=False):
# #                 self.get_logger().warn("❌ 상단 모듈 결합 실패")
# #                 return

# #         self.get_logger().info(" [Phase 3] 최종 결합: 상단 모듈 들어서 하단 모듈(노랑4x2) 위에 꽂기!")
# #         self.call(self.cli_h, Trigger.Request())
# #         if self.visual_insert("4x2_yellow", layer_index=2):
# #                 self.get_logger().info("✅🎉 5단 아이스크림 완벽하게 완성!")

# #     def build_big_tree(self):
# #         self.get_logger().info("🌲 [큰 나무] 6x2 베이스 조립 후 2x2_yellow 중앙에 최종 결합")
# #         self.get_logger().info("🧱 [초기화] 맨 처음 2x2_yellow 위치 스캔 및 기억")
# #         p_yellow = self.find_target_with_retry("2x2_yellow")
# #         if not p_yellow:
# #             self.get_logger().warn("❌ 바닥에 2x2_yellow가 안 보입니다. 조립을 취소합니다.")
# #             return
# #         saved_yellow_pose = p_yellow
# #         self.call(self.cli_h, Trigger.Request())

# #         self.get_logger().info("🧱 [Phase 1] 4x2_green(-1.85) 파지 -> 2x2_green(0.0) 결합 (그리퍼 유지)")
# #         if self.pick_target("4x2_green", offset_studs_y=0):
# #             self.call(self.cli_h, Trigger.Request())
# #             if self.visual_insert("4x2_green", layer_index=1, offset_studs_y=-1.0, release_gripper=False):
# #                 self.call(self.cli_h, Trigger.Request())
# #                 time.sleep(1.0) 

# #                 self.get_logger().info("🧱 [Phase 2] 바닥의 다른 4x2_green 스캔 및 6x2 조립 (그리퍼 해제)")
# #                 p_2x2_base = self.find_target_with_retry("2x2_green")
# #                 if not p_2x2_base:
# #                     self.get_logger().warn("❌ 바닥에 다른 2x2_green가 안 보입니다.")
# #                     return
# #                 saved_6x2_pose = p_2x2_base 
                
# #                 if self.blind_insert(saved_6x2_pose, layer_index=1, offset_studs_y=2.0, release_gripper=True):
# #                     self.call(self.cli_h, Trigger.Request())

# #                     self.get_logger().info("🧱 [Phase 3] 2x2_red 파지 -> 6x2 중심에 결합 (그리퍼 유지)")
# #                     if self.pick_target("2x2_green", offset_studs_y=-0.2):
# #                         self.call(self.cli_h, Trigger.Request())
# #                         if self.blind_insert(saved_6x2_pose, layer_index=2, offset_studs_y=0.0, release_gripper=False):
# #                             self.call(self.cli_h, Trigger.Request())

# #                             self.get_logger().info("🧱 [Phase 4] 덩어리를 2x2_yellow 중앙에 최종 결합")
# #                             if self.blind_insert(saved_yellow_pose, layer_index=2.5, offset_studs_y=0.0):
# #                                 self.get_logger().info("✅ 최종 조립 시퀀 완벽 종료!")

# #     def run(self):
# #         self.get_logger().info("🚀 STARTING VISUAL-STACK ASSEMBLY SEQUENCE (Keyboard Select Mode)")
# #         home_res = self.call(self.cli_h, Trigger.Request())
# #         if home_res is None or not home_res.success:
# #             self.get_logger().error("❌ 시작 HOME 이동 실패. 조립을 시작하지 않습니다.")
# #             return
# #         self.call(self.cli_g, SetBool.Request(data=False))
# #         time.sleep(1.0)
       
# #         actions = {
# #             "1": self.build_battery, "battery": self.build_battery, "배터리": self.build_battery,
# #             "2": self.build_magnet, "magnet": self.build_magnet, "자석": self.build_magnet,
# #             "3": self.build_e_stop, "estop": self.build_e_stop, "비상정지": self.build_e_stop,
# #             "4": self.build_carrot, "carrot": self.build_carrot, "당근": self.build_carrot,
# #             "5": self.build_traffic_light, "traffic": self.build_traffic_light, "신호등": self.build_traffic_light,
# #             "6": self.build_small_tree, "tree": self.build_small_tree, "작은나무": self.build_small_tree,
# #             "7": self.build_hammer, "hammer": self.build_hammer, "망치": self.build_hammer,
# #             "8": self.build_big_carrot, "bigcarrot": self.build_big_carrot, "큰당근": self.build_big_carrot,
# #             "9": self.build_burger, "burger": self.build_burger, "버거": self.build_burger,
# #             "10": self.build_ice_cream, "icecream": self.build_ice_cream, "아이스크림": self.build_ice_cream,
# #             "11": self.build_big_tree, "bigtree": self.build_big_tree, "큰나무": self.build_big_tree, "큰 나무": self.build_big_tree
# #         }

# #         print("\n=== Master Node Assembly Keyboard Select ===")
# #         print("1: 배터리 / 2: 자석 / 3: 비상정지 / 4: 당근 / 5: 신호등 / 6: 작은나무")
# #         print("7: 망치 / 8: 큰당근 / 9: 버거 / 10: 아이스크림 / 11: 큰나무")
# #         print("q: 종료")

# #         while rclpy.ok():
# #             user_input = input("\n조립할 항목을 선택하세요 [1~11/q]: ").strip().replace(" ", "").lower()
# #             if user_input in ("q", "quit", "exit", "종료"):
# #                 self.get_logger().info("조립 시퀀스를 종료합니다.")
# #                 break

# #             action = actions.get(user_input)
# #             if action is None:
# #                 print("잘못된 입력입니다. 1~11 또는 q 중에서 선택하세요.")
# #                 continue

# #             self.get_logger().info(f"▶️ 작업 시작: {user_input}")
# #             action()
            
# #             self.call(self.cli_h, Trigger.Request())
# #             time.sleep(1.0)
# #             self.get_logger().info("✅ 개별 조립 완료")

# #         self.call(self.cli_h, Trigger.Request())
# #         self.get_logger().info("🎉 ALL SEQUENCE DONE")

# # def main():
# #     rclpy.init()
# #     node = MasterNode()
# #     node.run()
# #     rclpy.shutdown()

# # if __name__ == '__main__':
# #     main()

# # import rclpy
# # from rclpy.node import Node
# # from srvs_pkg.srv import GetTargetPose
# # from std_srvs.srv import SetBool, Trigger
# # import time
# # import math

# # class MasterNode(Node):
# #     def __init__(self):
# #         super().__init__('master_node')
# #         self.cli_v = self.create_client(GetTargetPose, '/get_target_pose')
# #         self.cli_r = self.create_client(GetTargetPose, '/robot_move_step')
# #         self.cli_g = self.create_client(SetBool, '/control_gripper')
# #         self.cli_h = self.create_client(Trigger, '/robot_home')
       
# #         self.Z_OFF = -95.0
# #         self.Z_MARGIN = 20.0
# #         self.BLOCK_H = 20.0
# #         self.WAIT_TIME = 1.5
# #         self.PRE_XY_LOWER = 70.0
       
# #         self.STUD_PITCH = 0.016
# #         self.YAW_TUNE = 0.0

# #         self.CLASS_TO_TARGET_ID = {
# #             "2x2_red": "1",
# #             "2x2_green": "2",
# #             "2x2_blue": "3",
# #             "2x2_yellow": "4",
# #             "4x2_red": "5",
# #             "4x2_green": "6",
# #             "4x2_blue": "7",
# #             "4x2_yellow": "8",
# #             "2x4_red": "5",
# #             "2x4_green": "6",
# #             "2x4_blue": "7",
# #             "2x4_yellow": "8",
# #             "assembly": "999",
# #         }
       
# #         self.last_perfect_pose = None

# #     def call(self, cli, req):
# #         while not cli.wait_for_service(timeout_sec=1.0):
# #             self.get_logger().info(f'Waiting for {cli.srv_name}...')
# #         future = cli.call_async(req)
# #         rclpy.spin_until_future_complete(self, future)
# #         return future.result()

# #     def to_vision_target_id(self, target):
# #         target = str(target).strip()
# #         for prefix in ("count_", "far_"):
# #             if target.startswith(prefix):
# #                 target = target[len(prefix):]

# #         if target.isdigit():
# #             return target

# #         target_id = self.CLASS_TO_TARGET_ID.get(target)
# #         if target_id is None:
# #             self.get_logger().error(f"❌ vision_node.py ID 매핑 없음: {target}")
# #             return target
# #         return target_id

# #     def request_target_pose(self, target):
# #         return self.call(
# #             self.cli_v,
# #             GetTargetPose.Request(target_color=self.to_vision_target_id(target))
# #         )

# #     def find_target_with_retry(self, color):
# #         p = self.request_target_pose(color)
# #         if p.success:
# #             return p
# #         self.get_logger().error(f"❌ [{color}] 타겟 인식 실패")
# #         return None

# #     def normalize_yaw(self, yaw):
# #         while yaw > 90.0:
# #             yaw -= 180.0
# #         while yaw < -90.0:
# #             yaw += 180.0
# #         return yaw

# #     def is_2x2_pose(self, pose):
# #         return str(getattr(pose, "class_name", "")).startswith("2x2_")

# #     def fold_2x2_yaw(self, yaw):
# #         yaw = self.normalize_yaw(yaw)
# #         if yaw > 45.0:
# #             yaw -= 90.0
# #         elif yaw < -45.0:
# #             yaw += 90.0
# #         return yaw

# #     def pose_yaw_for_xy_offset(self, pose):
# #         if self.is_2x2_pose(pose):
# #             return self.fold_2x2_yaw(pose.yaw)
# #         return pose.yaw

# #     def calc_target_xy(self, pose, offset_studs_x=0.0, offset_studs_y=0.0):
# #         dx = offset_studs_x * self.STUD_PITCH
# #         dy = offset_studs_y * self.STUD_PITCH
# #         yaw_rad = math.radians(self.pose_yaw_for_xy_offset(pose))
# #         real_offset_x = dx * math.cos(yaw_rad) - dy * math.sin(yaw_rad)
# #         real_offset_y = dx * math.sin(yaw_rad) + dy * math.cos(yaw_rad)
# #         return pose.x + real_offset_x, pose.y + real_offset_y

# #     def move_fast_from_pose(
# #         self,
# #         pose,
# #         layer_index=0,
# #         yaw_offset=0.0,
# #         offset_studs_x=0.0,
# #         offset_studs_y=0.0,
# #         pre_xy_lower=False,  # (호환용) 더 이상 사용 안 함: Z 선하강/중간 재촬영 제거됨
# #     ):
# #         # AMR 탑재 로봇팔 load_node 방식:
# #         # HOME에서 1회 측정한 pose 하나로 YAW + XY + Z 접근을 한 모션에 합친다.
# #         # (Z 7cm 선하강 / 중간 재촬영 모두 제거)
# #         target_x, target_y = self.calc_target_xy(pose, offset_studs_x, offset_studs_y)
# #         if self.is_2x2_pose(pose):
# #             target_yaw = self.fold_2x2_yaw(pose.yaw + yaw_offset + self.YAW_TUNE)
# #             self.get_logger().info(
# #                 f"🔄 [2x2 YAW FOLD] {pose.class_name} vision_yaw={pose.yaw:.1f}도 -> wrist_yaw={target_yaw:.1f}도"
# #             )
# #         else:
# #             target_yaw = self.normalize_yaw(pose.yaw + yaw_offset + self.YAW_TUNE)

# #         z_move = (pose.z * 1000.0 + self.Z_OFF) - (self.BLOCK_H * layer_index)

# #         # [1] YAW + XY + Z 동시 이동 (물체 바로 위 Z_MARGIN 지점까지 대각선 접근)
# #         self.get_logger().info(
# #             f"➡️ [ONE-SHOT] x={target_x:.4f}m, y={target_y:.4f}m, "
# #             f"yaw={target_yaw:.1f}도, z_approach={z_move - self.Z_MARGIN:.1f}mm"
# #         )
# #         self.call(self.cli_r, GetTargetPose.Request(
# #             x=target_x, y=target_y, z=z_move - self.Z_MARGIN,
# #             yaw=target_yaw, target_size="APPROACH"
# #         ))
# #         time.sleep(self.WAIT_TIME)

# #         # [2] 최종 수직 접근 (yaw 회전 후에도 tool Z축은 수직 유지)
# #         self.call(self.cli_r, GetTargetPose.Request(z=self.Z_MARGIN, target_size="Z"))
# #         time.sleep(self.WAIT_TIME)

# #     def pick_target(self, color, layer_index=0, offset_studs_x=0.0, offset_studs_y=0.0):
# #         self.get_logger().info(f"\n--- PICK TARGET: [{color.upper()}] ---")
        
# #         p = self.find_target_with_retry(color)
# #         if not p: return False

# #         self.move_fast_from_pose(
# #             p,
# #             layer_index=layer_index,
# #             offset_studs_x=offset_studs_x,
# #             offset_studs_y=offset_studs_y,
# #         )

# #         self.call(self.cli_g, SetBool.Request(data=True))
# #         time.sleep(self.WAIT_TIME)
        
# #         self.call(self.cli_r, GetTargetPose.Request(z=-50.0, target_size="Z"))
# #         time.sleep(self.WAIT_TIME)
# #         return True

# #     def blind_insert(self, base_pose, layer_index, yaw_offset=0.0, release_gripper=True, offset_studs_x=0.0, offset_studs_y=0.0):
# #         self.get_logger().info(f"\n--- BLIND STACK (메모리 사용): Layer {layer_index} (Y Offset: {offset_studs_y}) ---")
# #         time.sleep(1.0)

# #         self.move_fast_from_pose(
# #             base_pose,
# #             layer_index=layer_index,
# #             yaw_offset=yaw_offset,
# #             offset_studs_x=offset_studs_x,
# #             offset_studs_y=offset_studs_y,
# #             pre_xy_lower=False,
# #         )

# #         if release_gripper:
# #             self.call(self.cli_g, SetBool.Request(data=False))
# #             time.sleep(self.WAIT_TIME)
            
# #         self.call(self.cli_r, GetTargetPose.Request(z=-50.0, target_size="Z"))
# #         time.sleep(self.WAIT_TIME)
# #         return True

# #     def visual_insert(
# #         self,
# #         target_color,
# #         layer_index,
# #         release_gripper=True,
# #         yaw_offset=0.0,
# #         # offset_studs_x=0.3125,
# #         # offset_studs_y=0.15625,
# #         offset_studs_x=0.0,
# #         offset_studs_y=0.0,
# #         pre_xy_lower=False,
# #     ):
# #         self.get_logger().info(f"\n--- VISUAL STACK: [{target_color.upper()}] (Layer +{layer_index}, Y Offset: {offset_studs_y}) ---")
# #         time.sleep(1.0)

# #         p = self.find_target_with_retry(target_color)
# #         if not p: return False
       
# #         self.last_perfect_pose = p

# #         self.move_fast_from_pose(
# #             p,
# #             layer_index=layer_index,
# #             yaw_offset=yaw_offset,
# #             offset_studs_x=offset_studs_x,
# #             offset_studs_y=offset_studs_y,
# #             pre_xy_lower=pre_xy_lower,
# #         )

# #         if release_gripper:
# #             self.call(self.cli_g, SetBool.Request(data=False))
# #             time.sleep(self.WAIT_TIME)
            
# #         self.call(self.cli_r, GetTargetPose.Request(z=-50.0, target_size="Z"))
# #         time.sleep(self.WAIT_TIME)
# #         return True

# #     def build_battery(self):
# #         self.get_logger().info("🔋 [배터리] 노란색(Pick) -> 파란색(Base)")
# #         if self.pick_target("2x2_yellow"):
# #             self.call(self.cli_h, Trigger.Request())
# #             self.visual_insert("2x2_blue", layer_index=1, pre_xy_lower=True)
# #             self.get_logger().info("✅ 배터리 조립 완료!")

# #     def build_magnet(self):
# #         self.get_logger().info("🧲 [자석] 파란색(Pick) -> 빨간색(Base)")
# #         if self.pick_target("2x2_blue"):
# #             self.call(self.cli_h, Trigger.Request())
# #             self.visual_insert("2x2_red", layer_index=1, pre_xy_lower=True)
# #             self.get_logger().info("✅ 자석 조립 완료!")

# #     def build_e_stop(self):
# #         self.get_logger().info("🛑 [비상정지] 빨간색(Pick) -> 노란색4x2(Base)")
# #         if self.pick_target("2x2_red"):
# #             self.call(self.cli_h, Trigger.Request())
# #             self.visual_insert("4x2_yellow", layer_index=1, yaw_offset=-90.0, pre_xy_lower=True)
# #             self.get_logger().info("✅ 비상정지 조립 완료!")

# #     def build_carrot(self):
# #         self.get_logger().info("🥕 [당근] 초록 (Pick) -> 노랑 (그리퍼 유지) -> 노란색(base)")
# #         if self.pick_target("2x2_green"):
# #             self.call(self.cli_h, Trigger.Request())
# #             if self.visual_insert("2x2_yellow", layer_index=0.9, pre_xy_lower=True,release_gripper=False):
# #                 self.call(self.cli_h, Trigger.Request())
# #                 if self.visual_insert("2x2_yellow", layer_index=1.8, pre_xy_lower=True):
# #                     self.get_logger().info("✅ 당근 완성!")

# #     def build_traffic_light(self):
# #         self.get_logger().info("🚦 [신호등] 노란색(Pick) -> 초록색(Base) -> 빨간색(Pick)")
# #         if self.pick_target("2x2_red"):
# #             self.call(self.cli_h, Trigger.Request())
# #             if self.visual_insert("2x2_yellow", layer_index=0.9, pre_xy_lower=True,release_gripper=False):
# #                 self.call(self.cli_h, Trigger.Request())
# #                 if self.visual_insert("2x2_green", layer_index=1.7, pre_xy_lower=True):
# #                     self.get_logger().info("✅ 신호등 완성!")

# #     def build_small_tree(self):
# #         self.get_logger().info("🌳 [작은 나무] 초록4x2(Pick) -> 노랑2x2(Base) -> 초록2x2(Pick)")
# #         if self.pick_target("2x2_green"):
# #             self.call(self.cli_h, Trigger.Request())
# #             if self.visual_insert("4x2_green", layer_index=0.8, pre_xy_lower=True, release_gripper=False):
# #                 self.call(self.cli_h, Trigger.Request())
# #                 if self.visual_insert("2x2_yellow", layer_index=1.8, pre_xy_lower=True):
# #                     self.get_logger().info("✅ 작은 나무 완성!")

# #     def build_hammer(self):
# #         self.get_logger().info("🔨 [망치] 빨강2x2(Pick) -> 빨강2x2(Base) -> 파랑4x2(Pick)")
# #         if self.pick_target("4x2_blue"):
# #             self.call(self.cli_h, Trigger.Request())
# #             if self.visual_insert("2x2_red", layer_index=0.9, pre_xy_lower=True, release_gripper=False):
# #                 self.call(self.cli_h, Trigger.Request())
# #                 if self.visual_insert("2x2_red", layer_index=1.8, yaw_offset=0.0, pre_xy_lower=True):
# #                     self.get_logger().info("✅ 망치 완성!")

# #     def build_big_carrot(self):
# #         self.get_logger().info("🥕🥕 [큰 당근] 노랑2x2(Pick) -> 노랑2x2(Base) -> 노랑4x2(Pick) -> 파랑2x2(Pick)")
# #         if self.pick_target("2x2_green"):
# #             self.call(self.cli_h, Trigger.Request())
# #             if self.visual_insert("4x2_yellow", layer_index=0.9, pre_xy_lower=True, release_gripper=False):
# #                 self.call(self.cli_h, Trigger.Request())
# #                 if self.visual_insert("2x2_yellow", layer_index=1.8, yaw_offset=0.0, pre_xy_lower=True, release_gripper=False):
# #                         self.call(self.cli_h, Trigger.Request())
# #                         if self.visual_insert("2x2_yellow", layer_index=2.7, pre_xy_lower=True, release_gripper=False):
# #                             self.get_logger().info("✅ 대왕 당근 완성!")

# #     # def build_burger(self):
# #     #     self.get_logger().info("🍔 [버거] 노랑4x2(Base) -> 빨강4x2(Offset Y -1) -> 빨강2x2(Offset Y +2) -> 노랑4x2(Top)")
# #     #     if self.pick_target("4x2_red"):
# #     #         self.call(self.cli_h, Trigger.Request())
# #     #         if self.visual_insert("4x2_yellow", layer_index=1, offset_studs_y=-1.0):
# #     #             saved_base_bun_pose = self.last_perfect_pose
# #     #             self.call(self.cli_h, Trigger.Request())
# #     #             if self.pick_target("2x2_red"):
# #     #                 self.call(self.cli_h, Trigger.Request())
# #     #                 if self.visual_insert("4x2_red", layer_index=0, yaw_offset=0.0, offset_studs_y=3.0):
# #     #                     self.call(self.cli_h, Trigger.Request())
# #     #                     if self.pick_target("4x2_yellow"):
# #     #                         self.call(self.cli_h, Trigger.Request())
# #     #                         if saved_base_bun_pose:
# #     #                             self.get_logger().info("🧠 [메모리 사용] 최초 바닥 빵의 좌표를 기억해서 정중앙에 덮습니다!")
# #     #                             self.blind_insert(saved_base_bun_pose, layer_index=2, offset_studs_y=1.0)
# #     #                             self.get_logger().info("✅ 버거 완성!")
# #     #                         else:
# #     #                             self.get_logger().warn("❌ 저장된 좌표가 없습니다. 조립 실패.")


# #     def build_burger(self):
# #         self.get_logger().info("🍔 [버거] 4x2_yellow(Pick) -> 4x2_red 결합 -> 2x2_red로 6x2 조립 -> 저장된 4x2_yellow에 최종 결합")

# #         self.get_logger().info("🧱 [Phase 1] 4x2_yellow 파지")
# #         if self.pick_target("4x2_yellow", offset_studs_y=0):
# #             self.call(self.cli_h, Trigger.Request())

# #             self.get_logger().info("🧱 [Phase 2] 바닥의 다른 4x2_yellow 위치 스캔 및 기억")
# #             p_yellow = self.find_target_with_retry("4x2_yellow")
# #             if not p_yellow:
# #                 self.get_logger().warn("❌ 바닥에 다른 4x2_yellow가 안 보입니다. 조립을 취소합니다.")
# #                 return
# #             saved_yellow_pose = p_yellow
# #             self.call(self.cli_h, Trigger.Request())

# #             self.get_logger().info("🧱 [Phase 3] 들고 있는 4x2_yellow -> 4x2_red 결합 (그리퍼 유지)")
# #             if self.visual_insert("4x2_red", layer_index=0.8, offset_studs_y=-1.0, release_gripper=False):
# #                 self.call(self.cli_h, Trigger.Request())
# #                 time.sleep(1.0) 

# #                 self.get_logger().info("🧱 [Phase 4] 바닥의 2x2_red 스캔 및 6x2 조립 (그리퍼 유지)")
# #                 p_2x2_base = self.find_target_with_retry("2x2_red")
# #                 if not p_2x2_base:
# #                     self.get_logger().warn("❌ 바닥에 2x2_red가 안 보입니다.")
# #                     return
# #                 saved_6x2_pose = p_2x2_base

# #                 if self.blind_insert(saved_6x2_pose, layer_index=0.8, offset_studs_y=2.0, release_gripper=False):
# #                     self.call(self.cli_h, Trigger.Request())

# #                     self.get_logger().info("🧱 [Phase 5] 덩어리를 처음 저장한 4x2_yellow에 최종 결합")
# #                     if self.blind_insert(saved_yellow_pose, layer_index=1.9, offset_studs_y=0.0):
# #                         self.get_logger().info("✅ 버거 완성!")


# #     def build_ice_cream(self):
# #         self.get_logger().info("🍦 [아이스크림] 모듈형 조립 전략")
# #         self.get_logger().info("[Phase 1] 하단 조립: 노랑4x2(Pick) -> 노랑2x2(Base)")
# #         if self.pick_target("4x2_yellow"):
# #             self.call(self.cli_h, Trigger.Request())
# #             if not self.visual_insert("2x2_yellow", layer_index=1):
# #                 self.get_logger().warn("❌ 하단 모듈 조립 실패")
# #                 return

# #         self.get_logger().info("[Phase 2-1] 파랑2x2(Pick) -> 빨강2x2 옆에 배치 (바닥)")
# #         self.call(self.cli_h, Trigger.Request())
# #         if self.pick_target("2x2_blue"):
# #             self.call(self.cli_h, Trigger.Request())
# #             if not self.visual_insert("2x2_red", layer_index=0, offset_studs_y=2.0):
# #                 self.get_logger().warn("❌ 파란색 블록 배치 실패")
# #                 return

# #         self.get_logger().info("[Phase 2-2] 노랑2x2(Pick) -> 파랑2x2(Base, offset_y=-1)로 결합")
# #         self.call(self.cli_h, Trigger.Request())
# #         if self.pick_target("2x2_green"):
# #             self.call(self.cli_h, Trigger.Request())
# #             if not self.visual_insert("2x2_blue", layer_index=1, offset_studs_y=-1.0, release_gripper=False):
# #                 self.get_logger().warn("❌ 상단 모듈 결합 실패")
# #                 return

# #         self.get_logger().info(" [Phase 3] 최종 결합: 상단 모듈 들어서 하단 모듈(노랑4x2) 위에 꽂기!")
# #         self.call(self.cli_h, Trigger.Request())
# #         if self.visual_insert("4x2_yellow", layer_index=2):
# #                 self.get_logger().info("✅🎉 5단 아이스크림 완벽하게 완성!")

# #     def build_studs_y(self):
# #         self.get_logger().info("🧱 [초기화] 맨 처음 2x2_yellow 위치 스캔 및 기억")
# #         p_yellow = self.find_target_with_retry("2x2_yellow")
# #         if not p_yellow:
# #             self.get_logger().warn("❌ 바닥에 2x2_yellow가 안 보입니다. 조립을 취소합니다.")
# #             return
# #         saved_yellow_pose = p_yellow
# #         self.call(self.cli_h, Trigger.Request())

# #         self.get_logger().info("🧱 [Phase 1] 4x2_green(-1.85) 파지 -> 2x2_green(0.0) 결합 (그리퍼 유지)")
# #         if self.pick_target("4x2_green", offset_studs_y=0):
# #             self.call(self.cli_h, Trigger.Request())
# #             if self.visual_insert("4x2_green", layer_index=1, offset_studs_y=-1.0, release_gripper=False):
# #                 self.call(self.cli_h, Trigger.Request())
# #                 time.sleep(1.0) 

# #                 self.get_logger().info("🧱 [Phase 2] 바닥의 다른 4x2_green 스캔 및 6x2 조립 (그리퍼 해제)")
# #                 p_2x2_base = self.find_target_with_retry("2x2_green")
# #                 if not p_2x2_base:
# #                     self.get_logger().warn("❌ 바닥에 다른 2x2_green가 안 보입니다.")
# #                     return
# #                 saved_6x2_pose = p_2x2_base 
                
# #                 if self.blind_insert(saved_6x2_pose, layer_index=1, offset_studs_y=2.0, release_gripper=True):
# #                     self.call(self.cli_h, Trigger.Request())

# #                     self.get_logger().info("🧱 [Phase 3] 2x2_red 파지 -> 6x2 중심에 결합 (그리퍼 유지)")
# #                     if self.pick_target("2x2_green", offset_studs_y=-0.2):
# #                         self.call(self.cli_h, Trigger.Request())
# #                         if self.blind_insert(saved_6x2_pose, layer_index=2, offset_studs_y=0.0, release_gripper=False):
# #                             self.call(self.cli_h, Trigger.Request())

# #                             self.get_logger().info("🧱 [Phase 4] 덩어리를 2x2_yellow 중앙에 최종 결합")
# #                             if self.blind_insert(saved_yellow_pose, layer_index=2.5, offset_studs_y=0.0):
# #                                 self.get_logger().info("✅ 최종 조립 시퀀 완벽 종료!")

# #     def run(self):
# #         self.get_logger().info("🚀 STARTING VISUAL-STACK ASSEMBLY SEQUENCE (Keyboard Select Mode)")
# #         home_res = self.call(self.cli_h, Trigger.Request())
# #         if home_res is None or not home_res.success:
# #             self.get_logger().error("❌ 시작 HOME 이동 실패. 조립을 시작하지 않습니다.")
# #             return
# #         self.call(self.cli_g, SetBool.Request(data=False))
# #         time.sleep(1.0)
       
# #         actions = {
# #             "1": self.build_battery, "battery": self.build_battery, "배터리": self.build_battery,
# #             "2": self.build_magnet, "magnet": self.build_magnet, "자석": self.build_magnet,
# #             "3": self.build_e_stop, "estop": self.build_e_stop, "비상정지": self.build_e_stop,
# #             "4": self.build_carrot, "carrot": self.build_carrot, "당근": self.build_carrot,
# #             "5": self.build_traffic_light, "traffic": self.build_traffic_light, "신호등": self.build_traffic_light,
# #             "6": self.build_small_tree, "tree": self.build_small_tree, "작은나무": self.build_small_tree,
# #             "7": self.build_hammer, "hammer": self.build_hammer, "망치": self.build_hammer,
# #             "8": self.build_big_carrot, "bigcarrot": self.build_big_carrot, "큰당근": self.build_big_carrot,
# #             "9": self.build_burger, "burger": self.build_burger, "버거": self.build_burger,
# #             "10": self.build_ice_cream, "icecream": self.build_ice_cream, "아이스크림": self.build_ice_cream,
# #             "11": self.build_studs_y, "studs_y": self.build_studs_y
# #         }

# #         print("\n=== Master Node Assembly Keyboard Select ===")
# #         print("1: 배터리 / 2: 자석 / 3: 비상정지 / 4: 당근 / 5: 신호등 / 6: 작은나무")
# #         print("7: 망치 / 8: 큰당근 / 9: 버거 / 10: 아이스크림 / 11: studs_y")
# #         print("q: 종료")

# #         while rclpy.ok():
# #             user_input = input("\n조립할 항목을 선택하세요 [1~11/q]: ").strip().replace(" ", "").lower()
# #             if user_input in ("q", "quit", "exit", "종료"):
# #                 self.get_logger().info("조립 시퀀스를 종료합니다.")
# #                 break

# #             action = actions.get(user_input)
# #             if action is None:
# #                 print("잘못된 입력입니다. 1~11 또는 q 중에서 선택하세요.")
# #                 continue

# #             self.get_logger().info(f"▶️ 작업 시작: {user_input}")
# #             action()
            
# #             self.call(self.cli_h, Trigger.Request())
# #             time.sleep(1.0)
# #             self.get_logger().info("✅ 개별 조립 완료")

# #         self.call(self.cli_h, Trigger.Request())
# #         self.get_logger().info("🎉 ALL SEQUENCE DONE")

# # def main():
# #     rclpy.init()
# #     node = MasterNode()
# #     node.run()
# #     rclpy.shutdown()

# # if __name__ == '__main__':
# #     main()

# # import rclpy
# # from rclpy.node import Node
# # from srvs_pkg.srv import GetTargetPose
# # from std_srvs.srv import SetBool, Trigger
# # import time
# # import math

# # class MasterNode(Node):
# #     def __init__(self):
# #         super().__init__('master_node')
# #         self.cli_v = self.create_client(GetTargetPose, '/get_target_pose')
# #         self.cli_r = self.create_client(GetTargetPose, '/robot1/robot_move_step')
# #         self.cli_g = self.create_client(SetBool, '/control_gripper')
# #         self.cli_h = self.create_client(Trigger, '/robot1/robot_home')
       
# #         self.Z_OFF = -95.0
# #         self.Z_MARGIN = 20.0
# #         self.BLOCK_H = 20.0
# #         self.WAIT_TIME = 1.5
# #         self.PRE_XY_LOWER = 70.0
       
# #         self.STUD_PITCH = 0.016
# #         self.YAW_TUNE = 0.0

# #         self.CLASS_TO_TARGET_ID = {
# #             "2x2_red": "1",
# #             "2x2_green": "2",
# #             "2x2_blue": "3",
# #             "2x2_yellow": "4",
# #             "4x2_red": "5",
# #             "4x2_green": "6",
# #             "4x2_blue": "7",
# #             "4x2_yellow": "8",
# #             "2x4_red": "5",
# #             "2x4_green": "6",
# #             "2x4_blue": "7",
# #             "2x4_yellow": "8",
# #             "assembly": "999",
# #         }
       
# #         self.last_perfect_pose = None

# #     def call(self, cli, req):
# #         while not cli.wait_for_service(timeout_sec=1.0):
# #             self.get_logger().info(f'Waiting for {cli.srv_name}...')
# #         future = cli.call_async(req)
# #         rclpy.spin_until_future_complete(self, future)
# #         return future.result()

# #     def to_vision_target_id(self, target):
# #         target = str(target).strip()
# #         for prefix in ("count_", "far_"):
# #             if target.startswith(prefix):
# #                 target = target[len(prefix):]

# #         if target.isdigit():
# #             return target

# #         target_id = self.CLASS_TO_TARGET_ID.get(target)
# #         if target_id is None:
# #             self.get_logger().error(f"❌ vision_node.py ID 매핑 없음: {target}")
# #             return target
# #         return target_id

# #     def request_target_pose(self, target):
# #         return self.call(
# #             self.cli_v,
# #             GetTargetPose.Request(target_color=self.to_vision_target_id(target))
# #         )

# #     def find_target_with_retry(self, color):
# #         p = self.request_target_pose(color)
# #         if p.success:
# #             return p
# #         self.get_logger().error(f"❌ [{color}] 타겟 인식 실패")
# #         return None

# #     def normalize_yaw(self, yaw):
# #         while yaw > 90.0:
# #             yaw -= 180.0
# #         while yaw < -90.0:
# #             yaw += 180.0
# #         return yaw

# #     def is_2x2_pose(self, pose):
# #         return str(getattr(pose, "class_name", "")).startswith("2x2_")

# #     def fold_2x2_yaw(self, yaw):
# #         yaw = self.normalize_yaw(yaw)
# #         if yaw > 45.0:
# #             yaw -= 90.0
# #         elif yaw < -45.0:
# #             yaw += 90.0
# #         return yaw

# #     def pose_yaw_for_xy_offset(self, pose):
# #         if self.is_2x2_pose(pose):
# #             return self.fold_2x2_yaw(pose.yaw)
# #         return pose.yaw

# #     def calc_target_xy(self, pose, offset_studs_x=0.0, offset_studs_y=0.0):
# #         dx = offset_studs_x * self.STUD_PITCH
# #         dy = offset_studs_y * self.STUD_PITCH
# #         yaw_rad = math.radians(self.pose_yaw_for_xy_offset(pose))
# #         real_offset_x = dx * math.cos(yaw_rad) - dy * math.sin(yaw_rad)
# #         real_offset_y = dx * math.sin(yaw_rad) + dy * math.cos(yaw_rad)
# #         return pose.x + real_offset_x, pose.y + real_offset_y

# #     def move_fast_from_pose(
# #         self,
# #         pose,
# #         layer_index=0,
# #         yaw_offset=0.0,
# #         offset_studs_x=0.0,
# #         offset_studs_y=0.0,
# #         pre_xy_lower=False,  # (호환용) 더 이상 사용 안 함: Z 선하강/중간 재촬영 제거됨
# #     ):
# #         # AMR 탑재 로봇팔 load_node 방식:
# #         # HOME에서 1회 측정한 pose 하나로 YAW + XY + Z 접근을 한 모션에 합친다.
# #         # (Z 7cm 선하강 / 중간 재촬영 모두 제거)
# #         target_x, target_y = self.calc_target_xy(pose, offset_studs_x, offset_studs_y)
# #         if self.is_2x2_pose(pose):
# #             target_yaw = self.fold_2x2_yaw(pose.yaw + yaw_offset + self.YAW_TUNE)
# #             self.get_logger().info(
# #                 f"🔄 [2x2 YAW FOLD] {pose.class_name} vision_yaw={pose.yaw:.1f}도 -> wrist_yaw={target_yaw:.1f}도"
# #             )
# #         else:
# #             target_yaw = self.normalize_yaw(pose.yaw + yaw_offset + self.YAW_TUNE)

# #         z_move = (pose.z * 1000.0 + self.Z_OFF) - (self.BLOCK_H * layer_index)

# #         # [1] YAW + XY + Z 동시 이동 (물체 바로 위 Z_MARGIN 지점까지 대각선 접근)
# #         self.get_logger().info(
# #             f"➡️ [ONE-SHOT] x={target_x:.4f}m, y={target_y:.4f}m, "
# #             f"yaw={target_yaw:.1f}도, z_approach={z_move - self.Z_MARGIN:.1f}mm"
# #         )
# #         self.call(self.cli_r, GetTargetPose.Request(
# #             x=target_x, y=target_y, z=z_move - self.Z_MARGIN,
# #             yaw=target_yaw, target_size="APPROACH"
# #         ))
# #         time.sleep(self.WAIT_TIME)

# #         # [2] 최종 수직 접근 (yaw 회전 후에도 tool Z축은 수직 유지)
# #         self.call(self.cli_r, GetTargetPose.Request(z=self.Z_MARGIN, target_size="Z"))
# #         time.sleep(self.WAIT_TIME)

# #     def pick_target(self, color, layer_index=0, offset_studs_x=0.0, offset_studs_y=0.0):
# #         self.get_logger().info(f"\n--- PICK TARGET: [{color.upper()}] ---")
        
# #         p = self.find_target_with_retry(color)
# #         if not p: return False

# #         self.move_fast_from_pose(
# #             p,
# #             layer_index=layer_index,
# #             offset_studs_x=offset_studs_x,
# #             offset_studs_y=offset_studs_y,
# #         )

# #         self.call(self.cli_g, SetBool.Request(data=True))
# #         time.sleep(self.WAIT_TIME)
        
# #         self.call(self.cli_r, GetTargetPose.Request(z=-50.0, target_size="Z"))
# #         time.sleep(self.WAIT_TIME)
# #         return True

# #     def blind_insert(self, base_pose, layer_index, yaw_offset=0.0, release_gripper=True, offset_studs_x=0.0, offset_studs_y=0.0):
# #         self.get_logger().info(f"\n--- BLIND STACK (메모리 사용): Layer {layer_index} (Y Offset: {offset_studs_y}) ---")
# #         time.sleep(1.0)

# #         self.move_fast_from_pose(
# #             base_pose,
# #             layer_index=layer_index,
# #             yaw_offset=yaw_offset,
# #             offset_studs_x=offset_studs_x,
# #             offset_studs_y=offset_studs_y,
# #             pre_xy_lower=False,
# #         )

# #         if release_gripper:
# #             self.call(self.cli_g, SetBool.Request(data=False))
# #             time.sleep(self.WAIT_TIME)
            
# #         self.call(self.cli_r, GetTargetPose.Request(z=-50.0, target_size="Z"))
# #         time.sleep(self.WAIT_TIME)
# #         return True

# #     def visual_insert(
# #         self,
# #         target_color,
# #         layer_index,
# #         release_gripper=True,
# #         yaw_offset=0.0,
# #         # offset_studs_x=0.3125,
# #         # offset_studs_y=0.15625,
# #         offset_studs_x=0.0,
# #         offset_studs_y=0.0,
# #         pre_xy_lower=False,
# #     ):
# #         self.get_logger().info(f"\n--- VISUAL STACK: [{target_color.upper()}] (Layer +{layer_index}, Y Offset: {offset_studs_y}) ---")
# #         time.sleep(1.0)

# #         p = self.find_target_with_retry(target_color)
# #         if not p: return False
       
# #         self.last_perfect_pose = p

# #         self.move_fast_from_pose(
# #             p,
# #             layer_index=layer_index,
# #             yaw_offset=yaw_offset,
# #             offset_studs_x=offset_studs_x,
# #             offset_studs_y=offset_studs_y,
# #             pre_xy_lower=pre_xy_lower,
# #         )

# #         if release_gripper:
# #             self.call(self.cli_g, SetBool.Request(data=False))
# #             time.sleep(self.WAIT_TIME)
            
# #         self.call(self.cli_r, GetTargetPose.Request(z=-50.0, target_size="Z"))
# #         time.sleep(self.WAIT_TIME)
# #         return True

# #     def build_battery(self):
# #         self.get_logger().info("🔋 [배터리] 노란색(Pick) -> 파란색(Base)")
# #         if self.pick_target("2x2_yellow"):
# #             self.call(self.cli_h, Trigger.Request())
# #             self.visual_insert("2x2_blue", layer_index=1, pre_xy_lower=True)
# #             self.get_logger().info("✅ 배터리 조립 완료!")

# #     def build_magnet(self):
# #         self.get_logger().info("🧲 [자석] 파란색(Pick) -> 빨간색(Base)")
# #         if self.pick_target("2x2_blue"):
# #             self.call(self.cli_h, Trigger.Request())
# #             self.visual_insert("2x2_red", layer_index=1, pre_xy_lower=True)
# #             self.get_logger().info("✅ 자석 조립 완료!")

# #     def build_e_stop(self):
# #         self.get_logger().info("🛑 [비상정지] 빨간색(Pick) -> 노란색4x2(Base)")
# #         if self.pick_target("2x2_red"):
# #             self.call(self.cli_h, Trigger.Request())
# #             self.visual_insert("4x2_yellow", layer_index=1, yaw_offset=-90.0, pre_xy_lower=True)
# #             self.get_logger().info("✅ 비상정지 조립 완료!")

# #     def build_carrot(self):
# #         self.get_logger().info("🥕 [당근] 초록 (Pick) -> 노랑 (그리퍼 유지) -> 노란색(base)")
# #         if self.pick_target("2x2_green"):
# #             self.call(self.cli_h, Trigger.Request())
# #             if self.visual_insert("2x2_yellow", layer_index=0.9, pre_xy_lower=True,release_gripper=False):
# #                 self.call(self.cli_h, Trigger.Request())
# #                 if self.visual_insert("2x2_yellow", layer_index=1.8, pre_xy_lower=True):
# #                     self.get_logger().info("✅ 당근 완성!")

# #     def build_traffic_light(self):
# #         self.get_logger().info("🚦 [신호등] 노란색(Pick) -> 초록색(Base) -> 빨간색(Pick)")
# #         if self.pick_target("2x2_red"):
# #             self.call(self.cli_h, Trigger.Request())
# #             if self.visual_insert("2x2_yellow", layer_index=0.9, pre_xy_lower=True,release_gripper=False):
# #                 self.call(self.cli_h, Trigger.Request())
# #                 if self.visual_insert("2x2_green", layer_index=1.7, pre_xy_lower=True):
# #                     self.get_logger().info("✅ 신호등 완성!")

# #     def build_small_tree(self):
# #         self.get_logger().info("🌳 [작은 나무] 초록4x2(Pick) -> 노랑2x2(Base) -> 초록2x2(Pick)")
# #         if self.pick_target("2x2_green"):
# #             self.call(self.cli_h, Trigger.Request())
# #             if self.visual_insert("4x2_green", layer_index=0.8, pre_xy_lower=True, release_gripper=False):
# #                 self.call(self.cli_h, Trigger.Request())
# #                 if self.visual_insert("2x2_yellow", layer_index=1.8, pre_xy_lower=True):
# #                     self.get_logger().info("✅ 작은 나무 완성!")

# #     def build_hammer(self):
# #         self.get_logger().info("🔨 [망치] 빨강2x2(Pick) -> 빨강2x2(Base) -> 파랑4x2(Pick)")
# #         if self.pick_target("4x2_blue"):
# #             self.call(self.cli_h, Trigger.Request())
# #             if self.visual_insert("2x2_red", layer_index=0.9, pre_xy_lower=True, release_gripper=False):
# #                 self.call(self.cli_h, Trigger.Request())
# #                 if self.visual_insert("2x2_red", layer_index=1.8, yaw_offset=0.0, pre_xy_lower=True):
# #                     self.get_logger().info("✅ 망치 완성!")

# #     def build_big_carrot(self):
# #         self.get_logger().info("🥕🥕 [큰 당근] 노랑2x2(Pick) -> 노랑2x2(Base) -> 노랑4x2(Pick) -> 파랑2x2(Pick)")
# #         if self.pick_target("2x2_green"):
# #             self.call(self.cli_h, Trigger.Request())
# #             if self.visual_insert("4x2_yellow", layer_index=0.9, pre_xy_lower=True, release_gripper=False):
# #                 self.call(self.cli_h, Trigger.Request())
# #                 if self.visual_insert("2x2_yellow", layer_index=1.8, yaw_offset=0.0, pre_xy_lower=True, release_gripper=False):
# #                         self.call(self.cli_h, Trigger.Request())
# #                         if self.visual_insert("2x2_yellow", layer_index=2.7, pre_xy_lower=True, release_gripper=False):
# #                             self.get_logger().info("✅ 대왕 당근 완성!")

# #     # def build_burger(self):
# #     #     self.get_logger().info("🍔 [버거] 노랑4x2(Base) -> 빨강4x2(Offset Y -1) -> 빨강2x2(Offset Y +2) -> 노랑4x2(Top)")
# #     #     if self.pick_target("4x2_red"):
# #     #         self.call(self.cli_h, Trigger.Request())
# #     #         if self.visual_insert("4x2_yellow", layer_index=1, offset_studs_y=-1.0):
# #     #             saved_base_bun_pose = self.last_perfect_pose
# #     #             self.call(self.cli_h, Trigger.Request())
# #     #             if self.pick_target("2x2_red"):
# #     #                 self.call(self.cli_h, Trigger.Request())
# #     #                 if self.visual_insert("4x2_red", layer_index=0, yaw_offset=0.0, offset_studs_y=3.0):
# #     #                     self.call(self.cli_h, Trigger.Request())
# #     #                     if self.pick_target("4x2_yellow"):
# #     #                         self.call(self.cli_h, Trigger.Request())
# #     #                         if saved_base_bun_pose:
# #     #                             self.get_logger().info("🧠 [메모리 사용] 최초 바닥 빵의 좌표를 기억해서 정중앙에 덮습니다!")
# #     #                             self.blind_insert(saved_base_bun_pose, layer_index=2, offset_studs_y=1.0)
# #     #                             self.get_logger().info("✅ 버거 완성!")
# #     #                         else:
# #     #                             self.get_logger().warn("❌ 저장된 좌표가 없습니다. 조립 실패.")


# #     def build_burger(self):
# #         self.get_logger().info("🍔 [버거] 4x2_yellow(Pick) -> 4x2_red 결합 -> 2x2_red로 6x2 조립 -> 저장된 4x2_yellow에 최종 결합")

# #         self.get_logger().info("🧱 [Phase 1] 4x2_yellow 파지")
# #         if self.pick_target("4x2_yellow", offset_studs_y=0):
# #             self.call(self.cli_h, Trigger.Request())

# #             self.get_logger().info("🧱 [Phase 2] 바닥의 다른 4x2_yellow 위치 스캔 및 기억")
# #             p_yellow = self.find_target_with_retry("4x2_yellow")
# #             if not p_yellow:
# #                 self.get_logger().warn("❌ 바닥에 다른 4x2_yellow가 안 보입니다. 조립을 취소합니다.")
# #                 return
# #             saved_yellow_pose = p_yellow
# #             self.call(self.cli_h, Trigger.Request())

# #             self.get_logger().info("🧱 [Phase 3] 들고 있는 4x2_yellow -> 4x2_red 결합 (그리퍼 유지)")
# #             if self.visual_insert("4x2_red", layer_index=0.8, offset_studs_y=-1.0, release_gripper=False):
# #                 self.call(self.cli_h, Trigger.Request())
# #                 time.sleep(1.0) 

# #                 self.get_logger().info("🧱 [Phase 4] 바닥의 2x2_red 스캔 및 6x2 조립 (그리퍼 유지)")
# #                 p_2x2_base = self.find_target_with_retry("2x2_red")
# #                 if not p_2x2_base:
# #                     self.get_logger().warn("❌ 바닥에 2x2_red가 안 보입니다.")
# #                     return
# #                 saved_6x2_pose = p_2x2_base

# #                 if self.blind_insert(saved_6x2_pose, layer_index=0.8, offset_studs_y=2.0, release_gripper=False):
# #                     self.call(self.cli_h, Trigger.Request())

# #                     self.get_logger().info("🧱 [Phase 5] 덩어리를 처음 저장한 4x2_yellow에 최종 결합")
# #                     if self.blind_insert(saved_yellow_pose, layer_index=1.9, offset_studs_y=0.0):
# #                         self.get_logger().info("✅ 버거 완성!")


# #     def build_ice_cream(self):
# #         self.get_logger().info("🍦 [아이스크림] 모듈형 조립 전략")
# #         self.get_logger().info("[Phase 1] 하단 조립: 노랑4x2(Pick) -> 노랑2x2(Base)")
# #         if self.pick_target("4x2_yellow"):
# #             self.call(self.cli_h, Trigger.Request())
# #             if not self.visual_insert("2x2_yellow", layer_index=1):
# #                 self.get_logger().warn("❌ 하단 모듈 조립 실패")
# #                 return

# #         self.get_logger().info("[Phase 2-1] 파랑2x2(Pick) -> 빨강2x2 옆에 배치 (바닥)")
# #         self.call(self.cli_h, Trigger.Request())
# #         if self.pick_target("2x2_blue"):
# #             self.call(self.cli_h, Trigger.Request())
# #             if not self.visual_insert("2x2_red", layer_index=0, offset_studs_y=2.0):
# #                 self.get_logger().warn("❌ 파란색 블록 배치 실패")
# #                 return

# #         self.get_logger().info("[Phase 2-2] 노랑2x2(Pick) -> 파랑2x2(Base, offset_y=-1)로 결합")
# #         self.call(self.cli_h, Trigger.Request())
# #         if self.pick_target("2x2_green"):
# #             self.call(self.cli_h, Trigger.Request())
# #             if not self.visual_insert("2x2_blue", layer_index=1, offset_studs_y=-1.0, release_gripper=False):
# #                 self.get_logger().warn("❌ 상단 모듈 결합 실패")
# #                 return

# #         self.get_logger().info(" [Phase 3] 최종 결합: 상단 모듈 들어서 하단 모듈(노랑4x2) 위에 꽂기!")
# #         self.call(self.cli_h, Trigger.Request())
# #         if self.visual_insert("4x2_yellow", layer_index=2):
# #                 self.get_logger().info("✅🎉 5단 아이스크림 완벽하게 완성!")

# #     def build_big_tree(self):
# #         self.get_logger().info("🌲 [큰 나무] 6x2 베이스 조립 후 2x2_yellow 중앙에 최종 결합")
# #         self.get_logger().info("🧱 [초기화] 맨 처음 2x2_yellow 위치 스캔 및 기억")
# #         p_yellow = self.find_target_with_retry("2x2_yellow")
# #         if not p_yellow:
# #             self.get_logger().warn("❌ 바닥에 2x2_yellow가 안 보입니다. 조립을 취소합니다.")
# #             return
# #         saved_yellow_pose = p_yellow
# #         self.call(self.cli_h, Trigger.Request())

# #         self.get_logger().info("🧱 [Phase 1] 4x2_green(-1.85) 파지 -> 2x2_green(0.0) 결합 (그리퍼 유지)")
# #         if self.pick_target("4x2_green", offset_studs_y=0):
# #             self.call(self.cli_h, Trigger.Request())
# #             if self.visual_insert("4x2_green", layer_index=1, offset_studs_y=-1.0, release_gripper=False):
# #                 self.call(self.cli_h, Trigger.Request())
# #                 time.sleep(1.0) 

# #                 self.get_logger().info("🧱 [Phase 2] 바닥의 다른 4x2_green 스캔 및 6x2 조립 (그리퍼 해제)")
# #                 p_2x2_base = self.find_target_with_retry("2x2_green")
# #                 if not p_2x2_base:
# #                     self.get_logger().warn("❌ 바닥에 다른 2x2_green가 안 보입니다.")
# #                     return
# #                 saved_6x2_pose = p_2x2_base 
                
# #                 if self.blind_insert(saved_6x2_pose, layer_index=1, offset_studs_y=2.0, release_gripper=True):
# #                     self.call(self.cli_h, Trigger.Request())

# #                     self.get_logger().info("🧱 [Phase 3] 2x2_red 파지 -> 6x2 중심에 결합 (그리퍼 유지)")
# #                     if self.pick_target("2x2_green", offset_studs_y=-0.2):
# #                         self.call(self.cli_h, Trigger.Request())
# #                         if self.blind_insert(saved_6x2_pose, layer_index=2, offset_studs_y=0.0, release_gripper=False):
# #                             self.call(self.cli_h, Trigger.Request())

# #                             self.get_logger().info("🧱 [Phase 4] 덩어리를 2x2_yellow 중앙에 최종 결합")
# #                             if self.blind_insert(saved_yellow_pose, layer_index=2.5, offset_studs_y=0.0):
# #                                 self.get_logger().info("✅ 최종 조립 시퀀 완벽 종료!")

# #     def run(self):
# #         self.get_logger().info("🚀 STARTING VISUAL-STACK ASSEMBLY SEQUENCE (Keyboard Select Mode)")
# #         home_res = self.call(self.cli_h, Trigger.Request())
# #         if home_res is None or not home_res.success:
# #             self.get_logger().error("❌ 시작 HOME 이동 실패. 조립을 시작하지 않습니다.")
# #             return
# #         self.call(self.cli_g, SetBool.Request(data=False))
# #         time.sleep(1.0)
       
# #         actions = {
# #             "1": self.build_battery, "battery": self.build_battery, "배터리": self.build_battery,
# #             "2": self.build_magnet, "magnet": self.build_magnet, "자석": self.build_magnet,
# #             "3": self.build_e_stop, "estop": self.build_e_stop, "비상정지": self.build_e_stop,
# #             "4": self.build_carrot, "carrot": self.build_carrot, "당근": self.build_carrot,
# #             "5": self.build_traffic_light, "traffic": self.build_traffic_light, "신호등": self.build_traffic_light,
# #             "6": self.build_small_tree, "tree": self.build_small_tree, "작은나무": self.build_small_tree,
# #             "7": self.build_hammer, "hammer": self.build_hammer, "망치": self.build_hammer,
# #             "8": self.build_big_carrot, "bigcarrot": self.build_big_carrot, "큰당근": self.build_big_carrot,
# #             "9": self.build_burger, "burger": self.build_burger, "버거": self.build_burger,
# #             "10": self.build_ice_cream, "icecream": self.build_ice_cream, "아이스크림": self.build_ice_cream,
# #             "11": self.build_big_tree, "bigtree": self.build_big_tree, "큰나무": self.build_big_tree, "큰 나무": self.build_big_tree
# #         }

# #         print("\n=== Master Node Assembly Keyboard Select ===")
# #         print("1: 배터리 / 2: 자석 / 3: 비상정지 / 4: 당근 / 5: 신호등 / 6: 작은나무")
# #         print("7: 망치 / 8: 큰당근 / 9: 버거 / 10: 아이스크림 / 11: 큰나무")
# #         print("q: 종료")

# #         while rclpy.ok():
# #             user_input = input("\n조립할 항목을 선택하세요 [1~11/q]: ").strip().replace(" ", "").lower()
# #             if user_input in ("q", "quit", "exit", "종료"):
# #                 self.get_logger().info("조립 시퀀스를 종료합니다.")
# #                 break

# #             action = actions.get(user_input)
# #             if action is None:
# #                 print("잘못된 입력입니다. 1~11 또는 q 중에서 선택하세요.")
# #                 continue

# #             self.get_logger().info(f"▶️ 작업 시작: {user_input}")
# #             action()
            
# #             self.call(self.cli_h, Trigger.Request())
# #             time.sleep(1.0)
# #             self.get_logger().info("✅ 개별 조립 완료")

# #         self.call(self.cli_h, Trigger.Request())
# #         self.get_logger().info("🎉 ALL SEQUENCE DONE")

# # def main():
# #     rclpy.init()
# #     node = MasterNode()
# #     node.run()
# #     rclpy.shutdown()

# # if __name__ == '__main__':
# #     main()

# # import rclpy
# # from rclpy.node import Node
# # from srvs_pkg.srv import GetTargetPose
# # from std_srvs.srv import SetBool, Trigger
# # import time
# # import math

# # class MasterNode(Node):
# #     def __init__(self):
# #         super().__init__('master_node')
# #         self.cli_v = self.create_client(GetTargetPose, '/get_target_pose')
# #         self.cli_r = self.create_client(GetTargetPose, '/robot_move_step')
# #         self.cli_g = self.create_client(SetBool, '/control_gripper')
# #         self.cli_h = self.create_client(Trigger, '/robot_home')
       
# #         self.Z_OFF = -95.0
# #         self.Z_MARGIN = 20.0
# #         self.BLOCK_H = 20.0
# #         self.WAIT_TIME = 1.5
# #         self.PRE_XY_LOWER = 70.0
       
# #         self.STUD_PITCH = 0.016
# #         self.YAW_TUNE = 0.0

# #         self.CLASS_TO_TARGET_ID = {
# #             "2x2_red": "1",
# #             "2x2_green": "2",
# #             "2x2_blue": "3",
# #             "2x2_yellow": "4",
# #             "4x2_red": "5",
# #             "4x2_green": "6",
# #             "4x2_blue": "7",
# #             "4x2_yellow": "8",
# #             "2x4_red": "5",
# #             "2x4_green": "6",
# #             "2x4_blue": "7",
# #             "2x4_yellow": "8",
# #             "assembly": "999",
# #         }
       
# #         self.last_perfect_pose = None

# #     def call(self, cli, req):
# #         while not cli.wait_for_service(timeout_sec=1.0):
# #             self.get_logger().info(f'Waiting for {cli.srv_name}...')
# #         future = cli.call_async(req)
# #         rclpy.spin_until_future_complete(self, future)
# #         return future.result()

# #     def to_vision_target_id(self, target):
# #         target = str(target).strip()
# #         for prefix in ("count_", "far_"):
# #             if target.startswith(prefix):
# #                 target = target[len(prefix):]

# #         if target.isdigit():
# #             return target

# #         target_id = self.CLASS_TO_TARGET_ID.get(target)
# #         if target_id is None:
# #             self.get_logger().error(f"❌ vision_node.py ID 매핑 없음: {target}")
# #             return target
# #         return target_id

# #     def request_target_pose(self, target):
# #         return self.call(
# #             self.cli_v,
# #             GetTargetPose.Request(target_color=self.to_vision_target_id(target))
# #         )

# #     def find_target_with_retry(self, color):
# #         p = self.request_target_pose(color)
# #         if p.success:
# #             return p
# #         self.get_logger().error(f"❌ [{color}] 타겟 인식 실패")
# #         return None

# #     def normalize_yaw(self, yaw):
# #         while yaw > 90.0:
# #             yaw -= 180.0
# #         while yaw < -90.0:
# #             yaw += 180.0
# #         return yaw

# #     def is_2x2_pose(self, pose):
# #         return str(getattr(pose, "class_name", "")).startswith("2x2_")

# #     def fold_2x2_yaw(self, yaw):
# #         yaw = self.normalize_yaw(yaw)
# #         if yaw > 45.0:
# #             yaw -= 90.0
# #         elif yaw < -45.0:
# #             yaw += 90.0
# #         return yaw

# #     def pose_yaw_for_xy_offset(self, pose):
# #         if self.is_2x2_pose(pose):
# #             return self.fold_2x2_yaw(pose.yaw)
# #         return pose.yaw

# #     def calc_target_xy(self, pose, offset_studs_x=0.0, offset_studs_y=0.0):
# #         dx = offset_studs_x * self.STUD_PITCH
# #         dy = offset_studs_y * self.STUD_PITCH
# #         yaw_rad = math.radians(self.pose_yaw_for_xy_offset(pose))
# #         real_offset_x = dx * math.cos(yaw_rad) - dy * math.sin(yaw_rad)
# #         real_offset_y = dx * math.sin(yaw_rad) + dy * math.cos(yaw_rad)
# #         return pose.x + real_offset_x, pose.y + real_offset_y

# #     def move_fast_from_pose(
# #         self,
# #         pose,
# #         layer_index=0,
# #         yaw_offset=0.0,
# #         offset_studs_x=0.0,
# #         offset_studs_y=0.0,
# #         pre_xy_lower=False,  # (호환용) 더 이상 사용 안 함: Z 선하강/중간 재촬영 제거됨
# #     ):
# #         # AMR 탑재 로봇팔 load_node 방식:
# #         # HOME에서 1회 측정한 pose 하나로 YAW + XY + Z 접근을 한 모션에 합친다.
# #         # (Z 7cm 선하강 / 중간 재촬영 모두 제거)
# #         target_x, target_y = self.calc_target_xy(pose, offset_studs_x, offset_studs_y)
# #         if self.is_2x2_pose(pose):
# #             target_yaw = self.fold_2x2_yaw(pose.yaw + yaw_offset + self.YAW_TUNE)
# #             self.get_logger().info(
# #                 f"🔄 [2x2 YAW FOLD] {pose.class_name} vision_yaw={pose.yaw:.1f}도 -> wrist_yaw={target_yaw:.1f}도"
# #             )
# #         else:
# #             target_yaw = self.normalize_yaw(pose.yaw + yaw_offset + self.YAW_TUNE)

# #         z_move = (pose.z * 1000.0 + self.Z_OFF) - (self.BLOCK_H * layer_index)

# #         # [1] YAW + XY + Z 동시 이동 (물체 바로 위 Z_MARGIN 지점까지 대각선 접근)
# #         self.get_logger().info(
# #             f"➡️ [ONE-SHOT] x={target_x:.4f}m, y={target_y:.4f}m, "
# #             f"yaw={target_yaw:.1f}도, z_approach={z_move - self.Z_MARGIN:.1f}mm"
# #         )
# #         self.call(self.cli_r, GetTargetPose.Request(
# #             x=target_x, y=target_y, z=z_move - self.Z_MARGIN,
# #             yaw=target_yaw, target_size="APPROACH"
# #         ))
# #         time.sleep(self.WAIT_TIME)

# #         # [2] 최종 수직 접근 (yaw 회전 후에도 tool Z축은 수직 유지)
# #         self.call(self.cli_r, GetTargetPose.Request(z=self.Z_MARGIN, target_size="Z"))
# #         time.sleep(self.WAIT_TIME)

# #     def pick_target(self, color, layer_index=0, offset_studs_x=0.0, offset_studs_y=0.0):
# #         self.get_logger().info(f"\n--- PICK TARGET: [{color.upper()}] ---")
        
# #         p = self.find_target_with_retry(color)
# #         if not p: return False

# #         self.move_fast_from_pose(
# #             p,
# #             layer_index=layer_index,
# #             offset_studs_x=offset_studs_x,
# #             offset_studs_y=offset_studs_y,
# #         )

# #         self.call(self.cli_g, SetBool.Request(data=True))
# #         time.sleep(self.WAIT_TIME)
        
# #         self.call(self.cli_r, GetTargetPose.Request(z=-50.0, target_size="Z"))
# #         time.sleep(self.WAIT_TIME)
# #         return True

# #     def blind_insert(self, base_pose, layer_index, yaw_offset=0.0, release_gripper=True, offset_studs_x=0.0, offset_studs_y=0.0):
# #         self.get_logger().info(f"\n--- BLIND STACK (메모리 사용): Layer {layer_index} (Y Offset: {offset_studs_y}) ---")
# #         time.sleep(1.0)

# #         self.move_fast_from_pose(
# #             base_pose,
# #             layer_index=layer_index,
# #             yaw_offset=yaw_offset,
# #             offset_studs_x=offset_studs_x,
# #             offset_studs_y=offset_studs_y,
# #             pre_xy_lower=False,
# #         )

# #         if release_gripper:
# #             self.call(self.cli_g, SetBool.Request(data=False))
# #             time.sleep(self.WAIT_TIME)
            
# #         self.call(self.cli_r, GetTargetPose.Request(z=-50.0, target_size="Z"))
# #         time.sleep(self.WAIT_TIME)
# #         return True

# #     def visual_insert(
# #         self,
# #         target_color,
# #         layer_index,
# #         release_gripper=True,
# #         yaw_offset=0.0,
# #         # offset_studs_x=0.3125,
# #         # offset_studs_y=0.15625,
# #         offset_studs_x=0.0,
# #         offset_studs_y=0.0,
# #         pre_xy_lower=False,
# #     ):
# #         self.get_logger().info(f"\n--- VISUAL STACK: [{target_color.upper()}] (Layer +{layer_index}, Y Offset: {offset_studs_y}) ---")
# #         time.sleep(1.0)

# #         p = self.find_target_with_retry(target_color)
# #         if not p: return False
       
# #         self.last_perfect_pose = p

# #         self.move_fast_from_pose(
# #             p,
# #             layer_index=layer_index,
# #             yaw_offset=yaw_offset,
# #             offset_studs_x=offset_studs_x,
# #             offset_studs_y=offset_studs_y,
# #             pre_xy_lower=pre_xy_lower,
# #         )

# #         if release_gripper:
# #             self.call(self.cli_g, SetBool.Request(data=False))
# #             time.sleep(self.WAIT_TIME)
            
# #         self.call(self.cli_r, GetTargetPose.Request(z=-50.0, target_size="Z"))
# #         time.sleep(self.WAIT_TIME)
# #         return True

# #     def build_battery(self):
# #         self.get_logger().info("🔋 [배터리] 노란색(Pick) -> 파란색(Base)")
# #         if self.pick_target("2x2_yellow"):
# #             self.call(self.cli_h, Trigger.Request())
# #             self.visual_insert("2x2_blue", layer_index=1, pre_xy_lower=True)
# #             self.get_logger().info("✅ 배터리 조립 완료!")

# #     def build_magnet(self):
# #         self.get_logger().info("🧲 [자석] 파란색(Pick) -> 빨간색(Base)")
# #         if self.pick_target("2x2_blue"):
# #             self.call(self.cli_h, Trigger.Request())
# #             self.visual_insert("2x2_red", layer_index=1, pre_xy_lower=True)
# #             self.get_logger().info("✅ 자석 조립 완료!")

# #     def build_e_stop(self):
# #         self.get_logger().info("🛑 [비상정지] 빨간색(Pick) -> 노란색4x2(Base)")
# #         if self.pick_target("2x2_red"):
# #             self.call(self.cli_h, Trigger.Request())
# #             self.visual_insert("4x2_yellow", layer_index=1, yaw_offset=-90.0, pre_xy_lower=True)
# #             self.get_logger().info("✅ 비상정지 조립 완료!")

# #     def build_carrot(self):
# #         self.get_logger().info("🥕 [당근] 초록 (Pick) -> 노랑 (그리퍼 유지) -> 노란색(base)")
# #         if self.pick_target("2x2_green"):
# #             self.call(self.cli_h, Trigger.Request())
# #             if self.visual_insert("2x2_yellow", layer_index=0.9, pre_xy_lower=True,release_gripper=False):
# #                 self.call(self.cli_h, Trigger.Request())
# #                 if self.visual_insert("2x2_yellow", layer_index=1.8, pre_xy_lower=True):
# #                     self.get_logger().info("✅ 당근 완성!")

# #     def build_traffic_light(self):
# #         self.get_logger().info("🚦 [신호등] 노란색(Pick) -> 초록색(Base) -> 빨간색(Pick)")
# #         if self.pick_target("2x2_red"):
# #             self.call(self.cli_h, Trigger.Request())
# #             if self.visual_insert("2x2_yellow", layer_index=0.9, pre_xy_lower=True,release_gripper=False):
# #                 self.call(self.cli_h, Trigger.Request())
# #                 if self.visual_insert("2x2_green", layer_index=1.7, pre_xy_lower=True):
# #                     self.get_logger().info("✅ 신호등 완성!")

# #     def build_small_tree(self):
# #         self.get_logger().info("🌳 [작은 나무] 초록4x2(Pick) -> 노랑2x2(Base) -> 초록2x2(Pick)")
# #         if self.pick_target("2x2_green"):
# #             self.call(self.cli_h, Trigger.Request())
# #             if self.visual_insert("4x2_green", layer_index=0.8, pre_xy_lower=True, release_gripper=False):
# #                 self.call(self.cli_h, Trigger.Request())
# #                 if self.visual_insert("2x2_yellow", layer_index=1.8, pre_xy_lower=True):
# #                     self.get_logger().info("✅ 작은 나무 완성!")

# #     def build_hammer(self):
# #         self.get_logger().info("🔨 [망치] 빨강2x2(Pick) -> 빨강2x2(Base) -> 파랑4x2(Pick)")
# #         if self.pick_target("4x2_blue"):
# #             self.call(self.cli_h, Trigger.Request())
# #             if self.visual_insert("2x2_red", layer_index=0.9, pre_xy_lower=True, release_gripper=False):
# #                 self.call(self.cli_h, Trigger.Request())
# #                 if self.visual_insert("2x2_red", layer_index=1.8, yaw_offset=0.0, pre_xy_lower=True):
# #                     self.get_logger().info("✅ 망치 완성!")

# #     def build_big_carrot(self):
# #         self.get_logger().info("🥕🥕 [큰 당근] 노랑2x2(Pick) -> 노랑2x2(Base) -> 노랑4x2(Pick) -> 파랑2x2(Pick)")
# #         if self.pick_target("2x2_green"):
# #             self.call(self.cli_h, Trigger.Request())
# #             if self.visual_insert("4x2_yellow", layer_index=0.9, pre_xy_lower=True, release_gripper=False):
# #                 self.call(self.cli_h, Trigger.Request())
# #                 if self.visual_insert("2x2_yellow", layer_index=1.8, yaw_offset=0.0, pre_xy_lower=True, release_gripper=False):
# #                         self.call(self.cli_h, Trigger.Request())
# #                         if self.visual_insert("2x2_yellow", layer_index=2.7, pre_xy_lower=True, release_gripper=False):
# #                             self.get_logger().info("✅ 대왕 당근 완성!")

# #     # def build_burger(self):
# #     #     self.get_logger().info("🍔 [버거] 노랑4x2(Base) -> 빨강4x2(Offset Y -1) -> 빨강2x2(Offset Y +2) -> 노랑4x2(Top)")
# #     #     if self.pick_target("4x2_red"):
# #     #         self.call(self.cli_h, Trigger.Request())
# #     #         if self.visual_insert("4x2_yellow", layer_index=1, offset_studs_y=-1.0):
# #     #             saved_base_bun_pose = self.last_perfect_pose
# #     #             self.call(self.cli_h, Trigger.Request())
# #     #             if self.pick_target("2x2_red"):
# #     #                 self.call(self.cli_h, Trigger.Request())
# #     #                 if self.visual_insert("4x2_red", layer_index=0, yaw_offset=0.0, offset_studs_y=3.0):
# #     #                     self.call(self.cli_h, Trigger.Request())
# #     #                     if self.pick_target("4x2_yellow"):
# #     #                         self.call(self.cli_h, Trigger.Request())
# #     #                         if saved_base_bun_pose:
# #     #                             self.get_logger().info("🧠 [메모리 사용] 최초 바닥 빵의 좌표를 기억해서 정중앙에 덮습니다!")
# #     #                             self.blind_insert(saved_base_bun_pose, layer_index=2, offset_studs_y=1.0)
# #     #                             self.get_logger().info("✅ 버거 완성!")
# #     #                         else:
# #     #                             self.get_logger().warn("❌ 저장된 좌표가 없습니다. 조립 실패.")


# #     def build_burger(self):
# #         self.get_logger().info("🍔 [버거] 4x2_yellow(Pick) -> 4x2_red 결합 -> 2x2_red로 6x2 조립 -> 저장된 4x2_yellow에 최종 결합")

# #         self.get_logger().info("🧱 [Phase 1] 4x2_yellow 파지")
# #         if self.pick_target("4x2_yellow", offset_studs_y=0):
# #             self.call(self.cli_h, Trigger.Request())

# #             self.get_logger().info("🧱 [Phase 2] 바닥의 다른 4x2_yellow 위치 스캔 및 기억")
# #             p_yellow = self.find_target_with_retry("4x2_yellow")
# #             if not p_yellow:
# #                 self.get_logger().warn("❌ 바닥에 다른 4x2_yellow가 안 보입니다. 조립을 취소합니다.")
# #                 return
# #             saved_yellow_pose = p_yellow
# #             self.call(self.cli_h, Trigger.Request())

# #             self.get_logger().info("🧱 [Phase 3] 들고 있는 4x2_yellow -> 4x2_red 결합 (그리퍼 유지)")
# #             if self.visual_insert("4x2_red", layer_index=0.8, offset_studs_y=-1.0, release_gripper=False):
# #                 self.call(self.cli_h, Trigger.Request())
# #                 time.sleep(1.0) 

# #                 self.get_logger().info("🧱 [Phase 4] 바닥의 2x2_red 스캔 및 6x2 조립 (그리퍼 유지)")
# #                 p_2x2_base = self.find_target_with_retry("2x2_red")
# #                 if not p_2x2_base:
# #                     self.get_logger().warn("❌ 바닥에 2x2_red가 안 보입니다.")
# #                     return
# #                 saved_6x2_pose = p_2x2_base

# #                 if self.blind_insert(saved_6x2_pose, layer_index=0.8, offset_studs_y=2.0, release_gripper=False):
# #                     self.call(self.cli_h, Trigger.Request())

# #                     self.get_logger().info("🧱 [Phase 5] 덩어리를 처음 저장한 4x2_yellow에 최종 결합")
# #                     if self.blind_insert(saved_yellow_pose, layer_index=1.9, offset_studs_y=0.0):
# #                         self.get_logger().info("✅ 버거 완성!")


# #     def build_ice_cream(self):
# #         self.get_logger().info("🍦 [아이스크림] 모듈형 조립 전략")
# #         self.get_logger().info("[Phase 1] 하단 조립: 노랑4x2(Pick) -> 노랑2x2(Base)")
# #         if self.pick_target("4x2_yellow"):
# #             self.call(self.cli_h, Trigger.Request())
# #             if not self.visual_insert("2x2_yellow", layer_index=1):
# #                 self.get_logger().warn("❌ 하단 모듈 조립 실패")
# #                 return

# #         self.get_logger().info("[Phase 2-1] 파랑2x2(Pick) -> 빨강2x2 옆에 배치 (바닥)")
# #         self.call(self.cli_h, Trigger.Request())
# #         if self.pick_target("2x2_blue"):
# #             self.call(self.cli_h, Trigger.Request())
# #             if not self.visual_insert("2x2_red", layer_index=0, offset_studs_y=2.0):
# #                 self.get_logger().warn("❌ 파란색 블록 배치 실패")
# #                 return

# #         self.get_logger().info("[Phase 2-2] 노랑2x2(Pick) -> 파랑2x2(Base, offset_y=-1)로 결합")
# #         self.call(self.cli_h, Trigger.Request())
# #         if self.pick_target("2x2_green"):
# #             self.call(self.cli_h, Trigger.Request())
# #             if not self.visual_insert("2x2_blue", layer_index=1, offset_studs_y=-1.0, release_gripper=False):
# #                 self.get_logger().warn("❌ 상단 모듈 결합 실패")
# #                 return

# #         self.get_logger().info(" [Phase 3] 최종 결합: 상단 모듈 들어서 하단 모듈(노랑4x2) 위에 꽂기!")
# #         self.call(self.cli_h, Trigger.Request())
# #         if self.visual_insert("4x2_yellow", layer_index=2):
# #                 self.get_logger().info("✅🎉 5단 아이스크림 완벽하게 완성!")

# #     def build_studs_y(self):
# #         self.get_logger().info("🧱 [초기화] 맨 처음 2x2_yellow 위치 스캔 및 기억")
# #         p_yellow = self.find_target_with_retry("2x2_yellow")
# #         if not p_yellow:
# #             self.get_logger().warn("❌ 바닥에 2x2_yellow가 안 보입니다. 조립을 취소합니다.")
# #             return
# #         saved_yellow_pose = p_yellow
# #         self.call(self.cli_h, Trigger.Request())

# #         self.get_logger().info("🧱 [Phase 1] 4x2_green(-1.85) 파지 -> 2x2_green(0.0) 결합 (그리퍼 유지)")
# #         if self.pick_target("4x2_green", offset_studs_y=0):
# #             self.call(self.cli_h, Trigger.Request())
# #             if self.visual_insert("4x2_green", layer_index=1, offset_studs_y=-1.0, release_gripper=False):
# #                 self.call(self.cli_h, Trigger.Request())
# #                 time.sleep(1.0) 

# #                 self.get_logger().info("🧱 [Phase 2] 바닥의 다른 4x2_green 스캔 및 6x2 조립 (그리퍼 해제)")
# #                 p_2x2_base = self.find_target_with_retry("2x2_green")
# #                 if not p_2x2_base:
# #                     self.get_logger().warn("❌ 바닥에 다른 2x2_green가 안 보입니다.")
# #                     return
# #                 saved_6x2_pose = p_2x2_base 
                
# #                 if self.blind_insert(saved_6x2_pose, layer_index=1, offset_studs_y=2.0, release_gripper=True):
# #                     self.call(self.cli_h, Trigger.Request())

# #                     self.get_logger().info("🧱 [Phase 3] 2x2_red 파지 -> 6x2 중심에 결합 (그리퍼 유지)")
# #                     if self.pick_target("2x2_green", offset_studs_y=-0.2):
# #                         self.call(self.cli_h, Trigger.Request())
# #                         if self.blind_insert(saved_6x2_pose, layer_index=2, offset_studs_y=0.0, release_gripper=False):
# #                             self.call(self.cli_h, Trigger.Request())

# #                             self.get_logger().info("🧱 [Phase 4] 덩어리를 2x2_yellow 중앙에 최종 결합")
# #                             if self.blind_insert(saved_yellow_pose, layer_index=2.5, offset_studs_y=0.0):
# #                                 self.get_logger().info("✅ 최종 조립 시퀀 완벽 종료!")

# #     def run(self):
# #         self.get_logger().info("🚀 STARTING VISUAL-STACK ASSEMBLY SEQUENCE (Keyboard Select Mode)")
# #         home_res = self.call(self.cli_h, Trigger.Request())
# #         if home_res is None or not home_res.success:
# #             self.get_logger().error("❌ 시작 HOME 이동 실패. 조립을 시작하지 않습니다.")
# #             return
# #         self.call(self.cli_g, SetBool.Request(data=False))
# #         time.sleep(1.0)
       
# #         actions = {
# #             "1": self.build_battery, "battery": self.build_battery, "배터리": self.build_battery,
# #             "2": self.build_magnet, "magnet": self.build_magnet, "자석": self.build_magnet,
# #             "3": self.build_e_stop, "estop": self.build_e_stop, "비상정지": self.build_e_stop,
# #             "4": self.build_carrot, "carrot": self.build_carrot, "당근": self.build_carrot,
# #             "5": self.build_traffic_light, "traffic": self.build_traffic_light, "신호등": self.build_traffic_light,
# #             "6": self.build_small_tree, "tree": self.build_small_tree, "작은나무": self.build_small_tree,
# #             "7": self.build_hammer, "hammer": self.build_hammer, "망치": self.build_hammer,
# #             "8": self.build_big_carrot, "bigcarrot": self.build_big_carrot, "큰당근": self.build_big_carrot,
# #             "9": self.build_burger, "burger": self.build_burger, "버거": self.build_burger,
# #             "10": self.build_ice_cream, "icecream": self.build_ice_cream, "아이스크림": self.build_ice_cream,
# #             "11": self.build_studs_y, "studs_y": self.build_studs_y
# #         }

# #         print("\n=== Master Node Assembly Keyboard Select ===")
# #         print("1: 배터리 / 2: 자석 / 3: 비상정지 / 4: 당근 / 5: 신호등 / 6: 작은나무")
# #         print("7: 망치 / 8: 큰당근 / 9: 버거 / 10: 아이스크림 / 11: studs_y")
# #         print("q: 종료")

# #         while rclpy.ok():
# #             user_input = input("\n조립할 항목을 선택하세요 [1~11/q]: ").strip().replace(" ", "").lower()
# #             if user_input in ("q", "quit", "exit", "종료"):
# #                 self.get_logger().info("조립 시퀀스를 종료합니다.")
# #                 break

# #             action = actions.get(user_input)
# #             if action is None:
# #                 print("잘못된 입력입니다. 1~11 또는 q 중에서 선택하세요.")
# #                 continue

# #             self.get_logger().info(f"▶️ 작업 시작: {user_input}")
# #             action()
            
# #             self.call(self.cli_h, Trigger.Request())
# #             time.sleep(1.0)
# #             self.get_logger().info("✅ 개별 조립 완료")

# #         self.call(self.cli_h, Trigger.Request())
# #         self.get_logger().info("🎉 ALL SEQUENCE DONE")

# # def main():
# #     rclpy.init()
# #     node = MasterNode()
# #     node.run()
# #     rclpy.shutdown()

# # if __name__ == '__main__':
# #     main()
# import rclpy
# from rclpy.node import Node
# from srvs_pkg.srv import GetTargetPose
# from std_srvs.srv import SetBool, Trigger
# import time
# import math

# class MasterNode(Node):
#     def __init__(self):
#         super().__init__('master_node')
#         self.cli_v = self.create_client(GetTargetPose, '/get_target_pose')
#         self.cli_r = self.create_client(GetTargetPose, '/robot1/robot_move_step')
#         self.cli_g = self.create_client(SetBool, '/control_gripper')
#         self.cli_h = self.create_client(Trigger, '/robot1/robot_home')
       
#         self.Z_OFF = -95.0
#         self.Z_MARGIN = 20.0
#         self.BLOCK_H = 20.0
#         self.WAIT_TIME = 1.5
#         self.PRE_XY_LOWER = 70.0
       
#         self.STUD_PITCH = 0.016
#         self.YAW_TUNE = 0.0

#         self.CLASS_TO_TARGET_ID = {
#             "2x2_red": "1",
#             "2x2_green": "2",
#             "2x2_blue": "3",
#             "2x2_yellow": "4",
#             "4x2_red": "5",
#             "4x2_green": "6",
#             "4x2_blue": "7",
#             "4x2_yellow": "8",
#             "2x4_red": "5",
#             "2x4_green": "6",
#             "2x4_blue": "7",
#             "2x4_yellow": "8",
#             "assembly": "999",
#         }
       
#         self.last_perfect_pose = None

#     def call(self, cli, req):
#         while not cli.wait_for_service(timeout_sec=1.0):
#             self.get_logger().info(f'Waiting for {cli.srv_name}...')
#         future = cli.call_async(req)
#         rclpy.spin_until_future_complete(self, future)
#         return future.result()

#     def to_vision_target_id(self, target):
#         target = str(target).strip()
#         for prefix in ("count_", "far_"):
#             if target.startswith(prefix):
#                 target = target[len(prefix):]

#         if target.isdigit():
#             return target

#         target_id = self.CLASS_TO_TARGET_ID.get(target)
#         if target_id is None:
#             self.get_logger().error(f"❌ vision_node.py ID 매핑 없음: {target}")
#             return target
#         return target_id

#     def request_target_pose(self, target):
#         return self.call(
#             self.cli_v,
#             GetTargetPose.Request(target_color=self.to_vision_target_id(target))
#         )

#     def find_target_with_retry(self, color):
#         p = self.request_target_pose(color)
#         if p.success:
#             return p
#         self.get_logger().error(f"❌ [{color}] 타겟 인식 실패")
#         return None

#     def normalize_yaw(self, yaw):
#         while yaw > 90.0:
#             yaw -= 180.0
#         while yaw < -90.0:
#             yaw += 180.0
#         return yaw

#     def is_2x2_pose(self, pose):
#         return str(getattr(pose, "class_name", "")).startswith("2x2_")

#     def fold_2x2_yaw(self, yaw):
#         yaw = self.normalize_yaw(yaw)
#         if yaw > 45.0:
#             yaw -= 90.0
#         elif yaw < -45.0:
#             yaw += 90.0
#         return yaw

#     def pose_yaw_for_xy_offset(self, pose):
#         if self.is_2x2_pose(pose):
#             return self.fold_2x2_yaw(pose.yaw)
#         return pose.yaw

#     def calc_target_xy(self, pose, offset_studs_x=0.0, offset_studs_y=0.0):
#         dx = offset_studs_x * self.STUD_PITCH
#         dy = offset_studs_y * self.STUD_PITCH
#         yaw_rad = math.radians(self.pose_yaw_for_xy_offset(pose))
#         real_offset_x = dx * math.cos(yaw_rad) - dy * math.sin(yaw_rad)
#         real_offset_y = dx * math.sin(yaw_rad) + dy * math.cos(yaw_rad)
#         return pose.x + real_offset_x, pose.y + real_offset_y

#     def move_fast_from_pose(
#         self,
#         pose,
#         layer_index=0,
#         yaw_offset=0.0,
#         offset_studs_x=0.0,
#         offset_studs_y=0.0,
#         pre_xy_lower=False,  # (호환용) 더 이상 사용 안 함: Z 선하강/중간 재촬영 제거됨
#     ):
#         # AMR 탑재 로봇팔 load_node 방식:
#         # HOME에서 1회 측정한 pose 하나로 YAW + XY + Z 접근을 한 모션에 합친다.
#         # (Z 7cm 선하강 / 중간 재촬영 모두 제거)
#         target_x, target_y = self.calc_target_xy(pose, offset_studs_x, offset_studs_y)
#         if self.is_2x2_pose(pose):
#             target_yaw = self.fold_2x2_yaw(pose.yaw + yaw_offset + self.YAW_TUNE)
#             self.get_logger().info(
#                 f"🔄 [2x2 YAW FOLD] {pose.class_name} vision_yaw={pose.yaw:.1f}도 -> wrist_yaw={target_yaw:.1f}도"
#             )
#         else:
#             target_yaw = self.normalize_yaw(pose.yaw + yaw_offset + self.YAW_TUNE)

#         z_move = (pose.z * 1000.0 + self.Z_OFF) - (self.BLOCK_H * layer_index)
#         z_approach = z_move - self.Z_MARGIN
#         approach_target_size = "APPROACH"

#         retry_class_name = str(getattr(pose, "class_name", "")).strip()
#         if retry_class_name:
#             scan_x, scan_y = self.calc_target_xy(
#                 pose,
#                 offset_studs_x=2.0303,
#                 offset_studs_y=3.2114,
#             )
#             scan_z = self.PRE_XY_LOWER
#             self.get_logger().info(
#                 f"➡️ [PRECISION] 재촬영 위치 APPROACH: "
#                 f"x={scan_x:.4f}m, y={scan_y:.4f}m, yaw=0.0도, z={scan_z:.1f}mm"
#             )
#             self.call(self.cli_r, GetTargetPose.Request(
#                 x=scan_x, y=scan_y, z=scan_z,
#                 yaw=0.0, target_size="APPROACH"
#             ))
#             time.sleep(self.WAIT_TIME)

#             refined_pose = self.find_target_with_retry(retry_class_name)
#             if refined_pose:
#                 target_x, target_y = self.calc_target_xy(
#                     refined_pose,
#                     offset_studs_x,
#                     offset_studs_y,
#                 )
#                 refined_z_move = (
#                     refined_pose.z * 1000.0 + self.Z_OFF
#                 ) - (self.BLOCK_H * layer_index)
#                 z_approach = refined_z_move - self.Z_MARGIN
#                 if self.is_2x2_pose(refined_pose):
#                     target_yaw = self.fold_2x2_yaw(refined_pose.yaw + yaw_offset + self.YAW_TUNE)
#                     self.get_logger().info(
#                         f"🔄 [PRECISION 재촬영][2x2] {retry_class_name}: "
#                         f"x={target_x:.4f}m, y={target_y:.4f}m, "
#                         f"vision_yaw={refined_pose.yaw:.1f}도 -> wrist_yaw={target_yaw:.1f}도"
#                     )
#                 else:
#                     target_yaw = self.normalize_yaw(refined_pose.yaw + yaw_offset + self.YAW_TUNE)
#                     self.get_logger().info(
#                         f"🔄 [PRECISION 재촬영] {retry_class_name}: "
#                         f"x={target_x:.4f}m, y={target_y:.4f}m, "
#                         f"vision_yaw={refined_pose.yaw:.1f}도 -> wrist_yaw={target_yaw:.1f}도"
#                     )
#             else:
#                 target_x -= scan_x
#                 target_y -= scan_y
#                 z_approach -= scan_z
#                 approach_target_size = "APPROACH_DELTA"
#                 self.get_logger().warn(
#                     f"⚠️ [PRECISION] 재촬영 실패. 처음 촬영 목표로 fallback 접근: "
#                     f"dx={target_x:.4f}m, dy={target_y:.4f}m, dz={z_approach:.1f}mm, yaw={target_yaw:.1f}도"
#                 )

#         # [1] YAW + XY + Z 동시 이동 (물체 바로 위 Z_MARGIN 지점까지 대각선 접근)
#         self.get_logger().info(
#             f"➡️ [ONE-SHOT] x={target_x:.4f}m, y={target_y:.4f}m, "
#             f"yaw={target_yaw:.1f}도, z_approach={z_approach:.1f}mm"
#         )
#         self.call(self.cli_r, GetTargetPose.Request(
#             x=target_x, y=target_y, z=z_approach,
#             yaw=target_yaw, target_size=approach_target_size
#         ))
#         time.sleep(self.WAIT_TIME)

#         # [2] 최종 수직 접근 (yaw 회전 후에도 tool Z축은 수직 유지)
#         self.call(self.cli_r, GetTargetPose.Request(z=self.Z_MARGIN, target_size="Z"))
#         time.sleep(self.WAIT_TIME)
#         return True

#     def pick_target(self, color, layer_index=0, offset_studs_x=0.0, offset_studs_y=0.0):
#         self.get_logger().info(f"\n--- PICK TARGET: [{color.upper()}] ---")
        
#         p = self.find_target_with_retry(color)
#         if not p: return False

#         self.move_fast_from_pose(
#             p,
#             layer_index=layer_index,
#             offset_studs_x=offset_studs_x,
#             offset_studs_y=offset_studs_y,
#         )

#         self.call(self.cli_g, SetBool.Request(data=True))
#         time.sleep(self.WAIT_TIME)
        
#         self.call(self.cli_r, GetTargetPose.Request(z=-50.0, target_size="Z"))
#         time.sleep(self.WAIT_TIME)
#         return True

#     def blind_insert(self, base_pose, layer_index, yaw_offset=0.0, release_gripper=True, offset_studs_x=0.0, offset_studs_y=0.0):
#         self.get_logger().info(f"\n--- BLIND STACK (메모리 사용): Layer {layer_index} (Y Offset: {offset_studs_y}) ---")
#         time.sleep(1.0)

#         self.move_fast_from_pose(
#             base_pose,
#             layer_index=layer_index,
#             yaw_offset=yaw_offset,
#             offset_studs_x=offset_studs_x,
#             offset_studs_y=offset_studs_y,
#             pre_xy_lower=False,
#         )

#         if release_gripper:
#             self.call(self.cli_g, SetBool.Request(data=False))
#             time.sleep(self.WAIT_TIME)
            
#         self.call(self.cli_r, GetTargetPose.Request(z=-50.0, target_size="Z"))
#         time.sleep(self.WAIT_TIME)
#         return True

#     def visual_insert(
#         self,
#         target_color,
#         layer_index,
#         release_gripper=True,
#         yaw_offset=0.0,
#         # offset_studs_x=0.3125,
#         # offset_studs_y=0.15625,
#         offset_studs_x=0.0,
#         offset_studs_y=0.0,
#         pre_xy_lower=False,
#     ):
#         self.get_logger().info(f"\n--- VISUAL STACK: [{target_color.upper()}] (Layer +{layer_index}, Y Offset: {offset_studs_y}) ---")
#         time.sleep(1.0)

#         p = self.find_target_with_retry(target_color)
#         if not p: return False
       
#         self.last_perfect_pose = p

#         self.move_fast_from_pose(
#             p,
#             layer_index=layer_index,
#             yaw_offset=yaw_offset,
#             offset_studs_x=offset_studs_x,
#             offset_studs_y=offset_studs_y,
#             pre_xy_lower=pre_xy_lower,
#         )

#         if release_gripper:
#             self.call(self.cli_g, SetBool.Request(data=False))
#             time.sleep(self.WAIT_TIME)
            
#         self.call(self.cli_r, GetTargetPose.Request(z=-50.0, target_size="Z"))
#         time.sleep(self.WAIT_TIME)
#         return True

#     def build_battery(self):
#         self.get_logger().info("🔋 [배터리] 노란색(Pick) -> 파란색(Base)")
#         if self.pick_target("2x2_yellow"):
#             self.call(self.cli_h, Trigger.Request())
#             self.visual_insert("2x2_blue", layer_index=1, pre_xy_lower=True)
#             self.get_logger().info("✅ 배터리 조립 완료!")

#     def build_magnet(self):
#         self.get_logger().info("🧲 [자석] 파란색(Pick) -> 빨간색(Base)")
#         if self.pick_target("2x2_blue"):
#             self.call(self.cli_h, Trigger.Request())
#             self.visual_insert("2x2_red", layer_index=1, pre_xy_lower=True)
#             self.get_logger().info("✅ 자석 조립 완료!")

#     def build_e_stop(self):
#         self.get_logger().info("🛑 [비상정지] 빨간색(Pick) -> 노란색4x2(Base)")
#         if self.pick_target("2x2_red"):
#             self.call(self.cli_h, Trigger.Request())
#             self.visual_insert("4x2_yellow", layer_index=1, yaw_offset=-90.0, pre_xy_lower=True)
#             self.get_logger().info("✅ 비상정지 조립 완료!")

#     def build_carrot(self):
#         self.get_logger().info("🥕 [당근] 초록 (Pick) -> 노랑 (그리퍼 유지) -> 노란색(base)")
#         if self.pick_target("2x2_green"):
#             self.call(self.cli_h, Trigger.Request())
#             if self.visual_insert("2x2_yellow", layer_index=0.9, pre_xy_lower=True,release_gripper=False):
#                 self.call(self.cli_h, Trigger.Request())
#                 if self.visual_insert("2x2_yellow", layer_index=1.8, pre_xy_lower=True):
#                     self.get_logger().info("✅ 당근 완성!")

#     def build_traffic_light(self):
#         self.get_logger().info("🚦 [신호등] 노란색(Pick) -> 초록색(Base) -> 빨간색(Pick)")
#         if self.pick_target("2x2_red"):
#             self.call(self.cli_h, Trigger.Request())
#             if self.visual_insert("2x2_yellow", layer_index=0.9, pre_xy_lower=True,release_gripper=False):
#                 self.call(self.cli_h, Trigger.Request())
#                 if self.visual_insert("2x2_green", layer_index=1.7, pre_xy_lower=True):
#                     self.get_logger().info("✅ 신호등 완성!")

#     def build_small_tree(self):
#         self.get_logger().info("🌳 [작은 나무] 초록4x2(Pick) -> 노랑2x2(Base) -> 초록2x2(Pick)")
#         if self.pick_target("2x2_green"):
#             self.call(self.cli_h, Trigger.Request())
#             if self.visual_insert("4x2_green", layer_index=0.8, pre_xy_lower=True, release_gripper=False):
#                 self.call(self.cli_h, Trigger.Request())
#                 if self.visual_insert("2x2_yellow", layer_index=1.8, pre_xy_lower=True):
#                     self.get_logger().info("✅ 작은 나무 완성!")

#     def build_hammer(self):
#         self.get_logger().info("🔨 [망치] 빨강2x2(Pick) -> 빨강2x2(Base) -> 파랑4x2(Pick)")
#         if self.pick_target("4x2_blue"):
#             self.call(self.cli_h, Trigger.Request())
#             if self.visual_insert("2x2_red", layer_index=0.9, pre_xy_lower=True, release_gripper=False):
#                 self.call(self.cli_h, Trigger.Request())
#                 if self.visual_insert("2x2_red", layer_index=1.8, yaw_offset=0.0, pre_xy_lower=True):
#                     self.get_logger().info("✅ 망치 완성!")

#     def build_big_carrot(self):
#         self.get_logger().info("🥕🥕 [큰 당근] 노랑2x2(Pick) -> 노랑2x2(Base) -> 노랑4x2(Pick) -> 파랑2x2(Pick)")
#         if self.pick_target("2x2_green"):
#             self.call(self.cli_h, Trigger.Request())
#             if self.visual_insert("4x2_yellow", layer_index=0.7, pre_xy_lower=True, release_gripper=False):
#                 self.call(self.cli_h, Trigger.Request())
#                 if self.visual_insert("2x2_yellow", layer_index=1.7, yaw_offset=0.0, pre_xy_lower=True, release_gripper=False):
#                         self.call(self.cli_h, Trigger.Request())
#                         if self.visual_insert("2x2_yellow", layer_index=2.7, pre_xy_lower=True, release_gripper=False):
#                             self.get_logger().info("✅ 대왕 당근 완성!")

#     # def build_burger(self):
#     #     self.get_logger().info("🍔 [버거] 노랑4x2(Base) -> 빨강4x2(Offset Y -1) -> 빨강2x2(Offset Y +2) -> 노랑4x2(Top)")
#     #     if self.pick_target("4x2_red"):
#     #         self.call(self.cli_h, Trigger.Request())
#     #         if self.visual_insert("4x2_yellow", layer_index=1, offset_studs_y=-1.0):
#     #             saved_base_bun_pose = self.last_perfect_pose
#     #             self.call(self.cli_h, Trigger.Request())
#     #             if self.pick_target("2x2_red"):
#     #                 self.call(self.cli_h, Trigger.Request())
#     #                 if self.visual_insert("4x2_red", layer_index=0, yaw_offset=0.0, offset_studs_y=3.0):
#     #                     self.call(self.cli_h, Trigger.Request())
#     #                     if self.pick_target("4x2_yellow"):
#     #                         self.call(self.cli_h, Trigger.Request())
#     #                         if saved_base_bun_pose:
#     #                             self.get_logger().info("🧠 [메모리 사용] 최초 바닥 빵의 좌표를 기억해서 정중앙에 덮습니다!")
#     #                             self.blind_insert(saved_base_bun_pose, layer_index=2, offset_studs_y=1.0)
#     #                             self.get_logger().info("✅ 버거 완성!")
#     #                         else:
#     #                             self.get_logger().warn("❌ 저장된 좌표가 없습니다. 조립 실패.")


#     def build_burger(self):
#         self.get_logger().info("🍔 [버거] 4x2_yellow(Pick) -> 4x2_red 결합 -> 2x2_red로 6x2 조립 -> 저장된 4x2_yellow에 최종 결합")

#         self.get_logger().info("🧱 [Phase 1] 4x2_yellow 파지")
#         if self.pick_target("4x2_yellow", offset_studs_y=0):
#             self.call(self.cli_h, Trigger.Request())

#             self.get_logger().info("🧱 [Phase 2] 바닥의 다른 4x2_yellow 위치 스캔 및 기억")
#             p_yellow = self.find_target_with_retry("4x2_yellow")
#             if not p_yellow:
#                 self.get_logger().warn("❌ 바닥에 다른 4x2_yellow가 안 보입니다. 조립을 취소합니다.")
#                 return
#             saved_yellow_pose = p_yellow
#             self.call(self.cli_h, Trigger.Request())

#             self.get_logger().info("🧱 [Phase 3] 들고 있는 4x2_yellow -> 4x2_red 결합 (그리퍼 유지)")
#             if self.visual_insert("4x2_red", layer_index=0.8, offset_studs_y=-1.0, release_gripper=False):
#                 self.call(self.cli_h, Trigger.Request())
#                 time.sleep(1.0) 

#                 self.get_logger().info("🧱 [Phase 4] 바닥의 2x2_red 스캔 및 6x2 조립 (그리퍼 유지)")
#                 p_2x2_base = self.find_target_with_retry("2x2_red")
#                 if not p_2x2_base:
#                     self.get_logger().warn("❌ 바닥에 2x2_red가 안 보입니다.")
#                     return
#                 saved_6x2_pose = p_2x2_base

#                 if self.blind_insert(saved_6x2_pose, layer_index=0.8, offset_studs_y=2.0, release_gripper=False):
#                     self.call(self.cli_h, Trigger.Request())

#                     self.get_logger().info("🧱 [Phase 5] 덩어리를 처음 저장한 4x2_yellow에 최종 결합")
#                     if self.blind_insert(saved_yellow_pose, layer_index=1.9, offset_studs_y=0.0):
#                         self.get_logger().info("✅ 버거 완성!")


#     def build_ice_cream(self):
#         self.get_logger().info("🍦 [아이스크림] 모듈형 조립 전략")
#         self.get_logger().info("[Phase 1] 하단 조립: 노랑4x2(Pick) -> 노랑2x2(Base)")
#         if self.pick_target("4x2_yellow"):
#             self.call(self.cli_h, Trigger.Request())
#             if not self.visual_insert("2x2_yellow", layer_index=1):
#                 self.get_logger().warn("❌ 하단 모듈 조립 실패")
#                 return

#         self.get_logger().info("[Phase 2-1] 파랑2x2(Pick) -> 빨강2x2 옆에 배치 (바닥)")
#         self.call(self.cli_h, Trigger.Request())
#         if self.pick_target("2x2_blue"):
#             self.call(self.cli_h, Trigger.Request())
#             if not self.visual_insert("2x2_red", layer_index=0, offset_studs_y=2.0):
#                 self.get_logger().warn("❌ 파란색 블록 배치 실패")
#                 return

#         self.get_logger().info("[Phase 2-2] 노랑2x2(Pick) -> 파랑2x2(Base, offset_y=-1)로 결합")
#         self.call(self.cli_h, Trigger.Request())
#         if self.pick_target("2x2_green"):
#             self.call(self.cli_h, Trigger.Request())
#             if not self.visual_insert("2x2_blue", layer_index=1, offset_studs_y=-1.0, release_gripper=False):
#                 self.get_logger().warn("❌ 상단 모듈 결합 실패")
#                 return

#         self.get_logger().info(" [Phase 3] 최종 결합: 상단 모듈 들어서 하단 모듈(노랑4x2) 위에 꽂기!")
#         self.call(self.cli_h, Trigger.Request())
#         if self.visual_insert("4x2_yellow", layer_index=2):
#                 self.get_logger().info("✅🎉 5단 아이스크림 완벽하게 완성!")

#     def build_big_tree(self):
#         self.get_logger().info("🧱 [초기화] 맨 처음 2x2_yellow 위치 스캔 및 기억")
#         p_yellow = self.find_target_with_retry("2x2_yellow")
#         if not p_yellow:
#             self.get_logger().warn("❌ 바닥에 2x2_yellow가 안 보입니다. 조립을 취소합니다.")
#             return
#         saved_yellow_pose = p_yellow
#         self.call(self.cli_h, Trigger.Request())

#         self.get_logger().info("🧱 [Phase 1] 4x2_green(-1.85) 파지 -> 2x2_green(0.0) 결합 (그리퍼 유지)")
#         if self.pick_target("4x2_green", offset_studs_y=0):
#             self.call(self.cli_h, Trigger.Request())
#             if self.visual_insert("4x2_green", layer_index=1, offset_studs_y=-1.0, release_gripper=False):
#                 self.call(self.cli_h, Trigger.Request())
#                 time.sleep(1.0) 

#                 self.get_logger().info("🧱 [Phase 2] 바닥의 다른 4x2_green 스캔 및 6x2 조립 (그리퍼 해제)")
#                 p_2x2_base = self.find_target_with_retry("2x2_green")
#                 if not p_2x2_base:
#                     self.get_logger().warn("❌ 바닥에 다른 2x2_green가 안 보입니다.")
#                     return
#                 saved_6x2_pose = p_2x2_base 
                
#                 if self.blind_insert(saved_6x2_pose, layer_index=1, offset_studs_y=2.0, release_gripper=True):
#                     self.call(self.cli_h, Trigger.Request())

#                     self.get_logger().info("🧱 [Phase 3] 2x2_red 파지 -> 6x2 중심에 결합 (그리퍼 유지)")
#                     if self.pick_target("2x2_green", offset_studs_y=-0.2):
#                         self.call(self.cli_h, Trigger.Request())
#                         if self.blind_insert(saved_6x2_pose, layer_index=2, offset_studs_y=0.0, release_gripper=False):
#                             self.call(self.cli_h, Trigger.Request())

#                             self.get_logger().info("🧱 [Phase 4] 덩어리를 2x2_yellow 중앙에 최종 결합")
#                             if self.blind_insert(saved_yellow_pose, layer_index=2.5, offset_studs_y=0.0):
#                                 self.get_logger().info("✅ 최종 조립 시퀀 완벽 종료!")

#     def run(self):
#         self.get_logger().info("🚀 STARTING VISUAL-STACK ASSEMBLY SEQUENCE (Keyboard Select Mode)")
#         home_res = self.call(self.cli_h, Trigger.Request())
#         if home_res is None or not home_res.success:
#             self.get_logger().error("❌ 시작 HOME 이동 실패. 조립을 시작하지 않습니다.")
#             return
#         self.call(self.cli_g, SetBool.Request(data=False))
#         time.sleep(1.0)
       
#         actions = {
#             "1": self.build_battery, "battery": self.build_battery, "배터리": self.build_battery,
#             "2": self.build_magnet, "magnet": self.build_magnet, "자석": self.build_magnet,
#             "3": self.build_e_stop, "estop": self.build_e_stop, "비상정지": self.build_e_stop,
#             "4": self.build_carrot, "carrot": self.build_carrot, "당근": self.build_carrot,
#             "5": self.build_traffic_light, "traffic": self.build_traffic_light, "신호등": self.build_traffic_light,
#             "6": self.build_small_tree, "tree": self.build_small_tree, "작은나무": self.build_small_tree,
#             "7": self.build_hammer, "hammer": self.build_hammer, "망치": self.build_hammer,
#             "8": self.build_big_carrot, "bigcarrot": self.build_big_carrot, "큰당근": self.build_big_carrot,
#             "9": self.build_burger, "burger": self.build_burger, "버거": self.build_burger,
#             "10": self.build_ice_cream, "icecream": self.build_ice_cream, "아이스크림": self.build_ice_cream,
#             "11": self.build_big_tree, "big_tree": self.build_big_tree
#         }

#         print("\n=== Master Node Assembly Keyboard Select ===")
#         print("1: 배터리 / 2: 자석 / 3: 비상정지 / 4: 당근 / 5: 신호등 / 6: 작은나무")
#         print("7: 망치 / 8: 큰당근 / 9: 버거 / 10: 아이스크림 / 11: big_tree")
#         print("q: 종료")

#         while rclpy.ok():
#             user_input = input("\n조립할 항목을 선택하세요 [1~11/q]: ").strip().replace(" ", "").lower()
#             if user_input in ("q", "quit", "exit", "종료"):
#                 self.get_logger().info("조립 시퀀스를 종료합니다.")
#                 break

#             action = actions.get(user_input)
#             if action is None:
#                 print("잘못된 입력입니다. 1~11 또는 q 중에서 선택하세요.")
#                 continue

#             self.get_logger().info(f"▶️ 작업 시작: {user_input}")
#             action()
            
#             self.call(self.cli_h, Trigger.Request())
#             time.sleep(1.0)
#             self.get_logger().info("✅ 개별 조립 완료")

#         self.call(self.cli_h, Trigger.Request())
#         self.get_logger().info("🎉 ALL SEQUENCE DONE")

# def main():
#     rclpy.init()
#     node = MasterNode()
#     node.run()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

import rclpy
from rclpy.node import Node
from srvs_pkg.srv import GetTargetPose
from std_srvs.srv import SetBool, Trigger
import time
import math

class MasterNode(Node):
    def __init__(self):
        super().__init__('master_node')
        self.cli_v = self.create_client(GetTargetPose, '/get_target_pose')
        self.cli_r = self.create_client(GetTargetPose, '/robot1/robot_move_step')
        self.cli_g = self.create_client(SetBool, '/control_gripper')
        self.cli_h = self.create_client(Trigger, '/robot1/robot_home')
       
        self.Z_OFF = -95.0
        self.Z_MARGIN = 20.0
        self.BLOCK_H = 20.0
        self.WAIT_TIME = 1.5
        self.PRE_XY_LOWER = 70.0
       
        self.STUD_PITCH = 0.016
        self.YAW_TUNE = 0.0

        self.CLASS_TO_TARGET_ID = {
            "2x2_red": "1",
            "2x2_green": "2",
            "2x2_blue": "3",
            "2x2_yellow": "4",
            "4x2_red": "5",
            "4x2_green": "6",
            "4x2_blue": "7",
            "4x2_yellow": "8",
            "2x4_red": "5",
            "2x4_green": "6",
            "2x4_blue": "7",
            "2x4_yellow": "8",
            "assembly": "999",
        }
       
        self.last_perfect_pose = None

    def call(self, cli, req):
        while not cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'Waiting for {cli.srv_name}...')
        future = cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def to_vision_target_id(self, target):
        target = str(target).strip()
        for prefix in ("count_", "far_"):
            if target.startswith(prefix):
                target = target[len(prefix):]

        if target.isdigit():
            return target

        target_id = self.CLASS_TO_TARGET_ID.get(target)
        if target_id is None:
            self.get_logger().error(f"❌ vision_node.py ID 매핑 없음: {target}")
            return target
        return target_id

    def request_target_pose(self, target):
        return self.call(
            self.cli_v,
            GetTargetPose.Request(target_color=self.to_vision_target_id(target))
        )

    def find_target_with_retry(self, color):
        p = self.request_target_pose(color)
        if p.success:
            return p
        self.get_logger().error(f"❌ [{color}] 타겟 인식 실패")
        return None

    def normalize_yaw(self, yaw):
        while yaw > 90.0:
            yaw -= 180.0
        while yaw < -90.0:
            yaw += 180.0
        return yaw

    def is_2x2_pose(self, pose):
        return str(getattr(pose, "class_name", "")).startswith("2x2_")

    def fold_2x2_yaw(self, yaw):
        yaw = self.normalize_yaw(yaw)
        if yaw > 45.0:
            yaw -= 90.0
        elif yaw < -45.0:
            yaw += 90.0
        return yaw

    def pose_yaw_for_xy_offset(self, pose):
        if self.is_2x2_pose(pose):
            return self.fold_2x2_yaw(pose.yaw)
        return pose.yaw

    def calc_target_xy(self, pose, offset_studs_x=0.0, offset_studs_y=0.0):
        dx = offset_studs_x * self.STUD_PITCH
        dy = offset_studs_y * self.STUD_PITCH
        yaw_rad = math.radians(self.pose_yaw_for_xy_offset(pose))
        real_offset_x = dx * math.cos(yaw_rad) - dy * math.sin(yaw_rad)
        real_offset_y = dx * math.sin(yaw_rad) + dy * math.cos(yaw_rad)
        return pose.x + real_offset_x, pose.y + real_offset_y

    def move_fast_from_pose(
        self,
        pose,
        layer_index=0,
        yaw_offset=0.0,
        offset_studs_x=0.0,
        offset_studs_y=0.0,
        pre_xy_lower=False,  # (호환용) 더 이상 사용 안 함: Z 선하강/중간 재촬영 제거됨
    ):
        # AMR 탑재 로봇팔 load_node 방식:
        # HOME에서 1회 측정한 pose 하나로 YAW + XY + Z 접근을 한 모션에 합친다.
        # (Z 7cm 선하강 / 중간 재촬영 모두 제거)
        target_x, target_y = self.calc_target_xy(pose, offset_studs_x, offset_studs_y)
        if self.is_2x2_pose(pose):
            target_yaw = self.fold_2x2_yaw(pose.yaw + yaw_offset + self.YAW_TUNE)
            self.get_logger().info(
                f"🔄 [2x2 YAW FOLD] {pose.class_name} vision_yaw={pose.yaw:.1f}도 -> wrist_yaw={target_yaw:.1f}도"
            )
        else:
            target_yaw = self.normalize_yaw(pose.yaw + yaw_offset + self.YAW_TUNE)

        z_move = (pose.z * 1000.0 + self.Z_OFF) - (self.BLOCK_H * layer_index)
        z_approach = z_move - self.Z_MARGIN
        approach_target_size = "APPROACH"

        retry_class_name = str(getattr(pose, "class_name", "")).strip()
        if retry_class_name:
            scan_x, scan_y = self.calc_target_xy(
                pose,
                offset_studs_x=2.0303,
                offset_studs_y=3.2114,
            )
            scan_z = self.PRE_XY_LOWER
            self.get_logger().info(
                f"➡️ [PRECISION] 재촬영 위치 APPROACH: "
                f"x={scan_x:.4f}m, y={scan_y:.4f}m, yaw=0.0도, z={scan_z:.1f}mm"
            )
            self.call(self.cli_r, GetTargetPose.Request(
                x=scan_x, y=scan_y, z=scan_z,
                yaw=0.0, target_size="APPROACH"
            ))
            time.sleep(self.WAIT_TIME)

            refined_pose = self.find_target_with_retry("888")
            if refined_pose:
                target_x, target_y = self.calc_target_xy(
                    refined_pose,
                    offset_studs_x,
                    offset_studs_y,
                )
                refined_z_move = (
                    refined_pose.z * 1000.0 + self.Z_OFF
                ) - (self.BLOCK_H * layer_index)
                z_approach = refined_z_move - self.Z_MARGIN
                if self.is_2x2_pose(refined_pose):
                    target_yaw = self.fold_2x2_yaw(refined_pose.yaw + yaw_offset + self.YAW_TUNE)
                    self.get_logger().info(
                        f"🔄 [PRECISION 재촬영][2x2] {retry_class_name}: "
                        f"x={target_x:.4f}m, y={target_y:.4f}m, "
                        f"vision_yaw={refined_pose.yaw:.1f}도 -> wrist_yaw={target_yaw:.1f}도"
                    )
                else:
                    target_yaw = self.normalize_yaw(refined_pose.yaw + yaw_offset + self.YAW_TUNE)
                    self.get_logger().info(
                        f"🔄 [PRECISION 재촬영] {retry_class_name}: "
                        f"x={target_x:.4f}m, y={target_y:.4f}m, "
                        f"vision_yaw={refined_pose.yaw:.1f}도 -> wrist_yaw={target_yaw:.1f}도"
                    )
            else:
                target_x -= scan_x
                target_y -= scan_y
                z_approach -= scan_z
                approach_target_size = "APPROACH_DELTA"
                self.get_logger().warn(
                    f"⚠️ [PRECISION] 재촬영 실패. 처음 촬영 목표로 fallback 접근: "
                    f"dx={target_x:.4f}m, dy={target_y:.4f}m, dz={z_approach:.1f}mm, yaw={target_yaw:.1f}도"
                )

        # [1] YAW + XY + Z 동시 이동 (물체 바로 위 Z_MARGIN 지점까지 대각선 접근)
        self.get_logger().info(
            f"➡️ [ONE-SHOT] x={target_x:.4f}m, y={target_y:.4f}m, "
            f"yaw={target_yaw:.1f}도, z_approach={z_approach:.1f}mm"
        )
        self.call(self.cli_r, GetTargetPose.Request(
            x=target_x, y=target_y, z=z_approach,
            yaw=target_yaw, target_size=approach_target_size
        ))
        time.sleep(self.WAIT_TIME)

        # [2] 최종 수직 접근 (yaw 회전 후에도 tool Z축은 수직 유지)
        self.call(self.cli_r, GetTargetPose.Request(z=self.Z_MARGIN, target_size="Z"))
        time.sleep(self.WAIT_TIME)
        return True

    def pick_target(self, color, layer_index=0, offset_studs_x=0.0, offset_studs_y=0.0):
        self.get_logger().info(f"\n--- PICK TARGET: [{color.upper()}] ---")
        
        p = self.find_target_with_retry(color)
        if not p: return False

        self.move_fast_from_pose(
            p,
            layer_index=layer_index,
            offset_studs_x=offset_studs_x,
            offset_studs_y=offset_studs_y,
        )

        self.call(self.cli_g, SetBool.Request(data=True))
        time.sleep(self.WAIT_TIME)
        
        self.call(self.cli_r, GetTargetPose.Request(z=-50.0, target_size="Z"))
        time.sleep(self.WAIT_TIME)
        return True

    def blind_insert(self, base_pose, layer_index, yaw_offset=0.0, release_gripper=True, offset_studs_x=0.0, offset_studs_y=0.0):
        self.get_logger().info(f"\n--- BLIND STACK (메모리 사용): Layer {layer_index} (Y Offset: {offset_studs_y}) ---")
        time.sleep(1.0)

        self.move_fast_from_pose(
            base_pose,
            layer_index=layer_index,
            yaw_offset=yaw_offset,
            offset_studs_x=offset_studs_x,
            offset_studs_y=offset_studs_y,
            pre_xy_lower=False,
        )

        if release_gripper:
            self.call(self.cli_g, SetBool.Request(data=False))
            time.sleep(self.WAIT_TIME)
            
        self.call(self.cli_r, GetTargetPose.Request(z=-50.0, target_size="Z"))
        time.sleep(self.WAIT_TIME)
        return True

    def visual_insert(
        self,
        target_color,
        layer_index,
        release_gripper=True,
        yaw_offset=0.0,
        # offset_studs_x=0.3125,
        # offset_studs_y=0.15625,
        offset_studs_x=0.0,
        offset_studs_y=0.0,
        pre_xy_lower=False,
    ):
        self.get_logger().info(f"\n--- VISUAL STACK: [{target_color.upper()}] (Layer +{layer_index}, Y Offset: {offset_studs_y}) ---")
        time.sleep(1.0)

        p = self.find_target_with_retry(target_color)
        if not p: return False
       
        self.last_perfect_pose = p

        self.move_fast_from_pose(
            p,
            layer_index=layer_index,
            yaw_offset=yaw_offset,
            offset_studs_x=offset_studs_x,
            offset_studs_y=offset_studs_y,
            pre_xy_lower=pre_xy_lower,
        )

        if release_gripper:
            self.call(self.cli_g, SetBool.Request(data=False))
            time.sleep(self.WAIT_TIME)
            
        self.call(self.cli_r, GetTargetPose.Request(z=-50.0, target_size="Z"))
        time.sleep(self.WAIT_TIME)
        return True

    def build_battery(self):
        self.get_logger().info("🔋 [배터리] 노란색(Pick) -> 파란색(Base)")
        if self.pick_target("2x2_yellow"):
            self.call(self.cli_h, Trigger.Request())
            self.visual_insert("2x2_blue", layer_index=1, pre_xy_lower=True)
            self.get_logger().info("✅ 배터리 조립 완료!")

    def build_magnet(self):
        self.get_logger().info("🧲 [자석] 파란색(Pick) -> 빨간색(Base)")
        if self.pick_target("2x2_blue"):
            self.call(self.cli_h, Trigger.Request())
            self.visual_insert("2x2_red", layer_index=1, pre_xy_lower=True)
            self.get_logger().info("✅ 자석 조립 완료!")

    def build_e_stop(self):
        self.get_logger().info("🛑 [비상정지] 빨간색(Pick) -> 노란색4x2(Base)")
        if self.pick_target("2x2_red"):
            self.call(self.cli_h, Trigger.Request())
            self.visual_insert("4x2_yellow", layer_index=1, yaw_offset=-90.0, pre_xy_lower=True)
            self.get_logger().info("✅ 비상정지 조립 완료!")

    def build_carrot(self):
        self.get_logger().info("🥕 [당근] 초록 (Pick) -> 노랑 (그리퍼 유지) -> 노란색(base)")
        if self.pick_target("2x2_green"):
            self.call(self.cli_h, Trigger.Request())
            if self.visual_insert("2x2_yellow", layer_index=0.9, pre_xy_lower=True,release_gripper=False):
                self.call(self.cli_h, Trigger.Request())
                if self.visual_insert("2x2_yellow", layer_index=1.8, pre_xy_lower=True):
                    self.get_logger().info("✅ 당근 완성!")

    def build_traffic_light(self):
        self.get_logger().info("🚦 [신호등] 노란색(Pick) -> 초록색(Base) -> 빨간색(Pick)")
        if self.pick_target("2x2_red"):
            self.call(self.cli_h, Trigger.Request())
            if self.visual_insert("2x2_yellow", layer_index=0.9, pre_xy_lower=True,release_gripper=False):
                self.call(self.cli_h, Trigger.Request())
                if self.visual_insert("2x2_green", layer_index=1.7, pre_xy_lower=True):
                    self.get_logger().info("✅ 신호등 완성!")

    def build_small_tree(self):
        self.get_logger().info("🌳 [작은 나무] 초록4x2(Pick) -> 노랑2x2(Base) -> 초록2x2(Pick)")
        if self.pick_target("2x2_green"):
            self.call(self.cli_h, Trigger.Request())
            if self.visual_insert("4x2_green", layer_index=0.8, pre_xy_lower=True, release_gripper=False):
                self.call(self.cli_h, Trigger.Request())
                if self.visual_insert("2x2_yellow", layer_index=1.8, pre_xy_lower=True):
                    self.get_logger().info("✅ 작은 나무 완성!")

    def build_hammer(self):
        self.get_logger().info("🔨 [망치] 빨강2x2(Pick) -> 빨강2x2(Base) -> 파랑4x2(Pick)")
        if self.pick_target("4x2_blue"):
            self.call(self.cli_h, Trigger.Request())
            if self.visual_insert("2x2_red", layer_index=0.9, pre_xy_lower=True, release_gripper=False):
                self.call(self.cli_h, Trigger.Request())
                if self.visual_insert("2x2_red", layer_index=1.8, yaw_offset=0.0, pre_xy_lower=True):
                    self.get_logger().info("✅ 망치 완성!")

    def build_big_carrot(self):
        self.get_logger().info("🥕🥕 [큰 당근] 노랑2x2(Pick) -> 노랑2x2(Base) -> 노랑4x2(Pick) -> 파랑2x2(Pick)")
        if self.pick_target("2x2_green"):
            self.call(self.cli_h, Trigger.Request())
            if self.visual_insert("4x2_yellow", layer_index=0.7, pre_xy_lower=True, release_gripper=False):
                self.call(self.cli_h, Trigger.Request())
                if self.visual_insert("2x2_yellow", layer_index=1.7, yaw_offset=0.0, pre_xy_lower=True, release_gripper=False):
                        self.call(self.cli_h, Trigger.Request())
                        if self.visual_insert("2x2_yellow", layer_index=2.7, pre_xy_lower=True, release_gripper=False):
                            self.get_logger().info("✅ 대왕 당근 완성!")

    # def build_burger(self):
    #     self.get_logger().info("🍔 [버거] 노랑4x2(Base) -> 빨강4x2(Offset Y -1) -> 빨강2x2(Offset Y +2) -> 노랑4x2(Top)")
    #     if self.pick_target("4x2_red"):
    #         self.call(self.cli_h, Trigger.Request())
    #         if self.visual_insert("4x2_yellow", layer_index=1, offset_studs_y=-1.0):
    #             saved_base_bun_pose = self.last_perfect_pose
    #             self.call(self.cli_h, Trigger.Request())
    #             if self.pick_target("2x2_red"):
    #                 self.call(self.cli_h, Trigger.Request())
    #                 if self.visual_insert("4x2_red", layer_index=0, yaw_offset=0.0, offset_studs_y=3.0):
    #                     self.call(self.cli_h, Trigger.Request())
    #                     if self.pick_target("4x2_yellow"):
    #                         self.call(self.cli_h, Trigger.Request())
    #                         if saved_base_bun_pose:
    #                             self.get_logger().info("🧠 [메모리 사용] 최초 바닥 빵의 좌표를 기억해서 정중앙에 덮습니다!")
    #                             self.blind_insert(saved_base_bun_pose, layer_index=2, offset_studs_y=1.0)
    #                             self.get_logger().info("✅ 버거 완성!")
    #                         else:
    #                             self.get_logger().warn("❌ 저장된 좌표가 없습니다. 조립 실패.")


    def build_burger(self):
        self.get_logger().info("🍔 [버거] 4x2_yellow(Pick) -> 4x2_red 결합 -> 2x2_red로 6x2 조립 -> 저장된 4x2_yellow에 최종 결합")

        self.get_logger().info("🧱 [Phase 1] 4x2_yellow 파지")
        if self.pick_target("4x2_yellow", offset_studs_y=0):
            self.call(self.cli_h, Trigger.Request())

            self.get_logger().info("🧱 [Phase 2] 바닥의 다른 4x2_yellow 위치 스캔 및 기억")
            p_yellow = self.find_target_with_retry("4x2_yellow")
            if not p_yellow:
                self.get_logger().warn("❌ 바닥에 다른 4x2_yellow가 안 보입니다. 조립을 취소합니다.")
                return
            saved_yellow_pose = p_yellow
            self.call(self.cli_h, Trigger.Request())

            self.get_logger().info("🧱 [Phase 3] 들고 있는 4x2_yellow -> 4x2_red 결합 (그리퍼 유지)")
            if self.visual_insert("4x2_red", layer_index=0.8, offset_studs_y=-1.0, release_gripper=False):
                self.call(self.cli_h, Trigger.Request())
                time.sleep(1.0) 

                self.get_logger().info("🧱 [Phase 4] 바닥의 2x2_red 스캔 및 6x2 조립 (그리퍼 유지)")
                p_2x2_base = self.find_target_with_retry("2x2_red")
                if not p_2x2_base:
                    self.get_logger().warn("❌ 바닥에 2x2_red가 안 보입니다.")
                    return
                saved_6x2_pose = p_2x2_base

                if self.blind_insert(saved_6x2_pose, layer_index=0.8, offset_studs_y=2.0, release_gripper=False):
                    self.call(self.cli_h, Trigger.Request())

                    self.get_logger().info("🧱 [Phase 5] 덩어리를 처음 저장한 4x2_yellow에 최종 결합")
                    if self.blind_insert(saved_yellow_pose, layer_index=1.9, offset_studs_y=0.0):
                        self.get_logger().info("✅ 버거 완성!")


    def build_ice_cream(self):
        self.get_logger().info("🍦 [아이스크림] 모듈형 조립 전략")
        self.get_logger().info("[Phase 1] 하단 조립: 노랑4x2(Pick) -> 노랑2x2(Base)")
        if self.pick_target("4x2_yellow"):
            self.call(self.cli_h, Trigger.Request())
            if not self.visual_insert("2x2_yellow", layer_index=1):
                self.get_logger().warn("❌ 하단 모듈 조립 실패")
                return

        self.get_logger().info("[Phase 2-1] 파랑2x2(Pick) -> 빨강2x2 옆에 배치 (바닥)")
        self.call(self.cli_h, Trigger.Request())
        if self.pick_target("2x2_blue"):
            self.call(self.cli_h, Trigger.Request())
            if not self.visual_insert("2x2_red", layer_index=0, offset_studs_y=2.0):
                self.get_logger().warn("❌ 파란색 블록 배치 실패")
                return

        self.get_logger().info("[Phase 2-2] 노랑2x2(Pick) -> 파랑2x2(Base, offset_y=-1)로 결합")
        self.call(self.cli_h, Trigger.Request())
        if self.pick_target("2x2_green"):
            self.call(self.cli_h, Trigger.Request())
            if not self.visual_insert("2x2_blue", layer_index=1, offset_studs_y=-1.0, release_gripper=False):
                self.get_logger().warn("❌ 상단 모듈 결합 실패")
                return

        self.get_logger().info(" [Phase 3] 최종 결합: 상단 모듈 들어서 하단 모듈(노랑4x2) 위에 꽂기!")
        self.call(self.cli_h, Trigger.Request())
        if self.visual_insert("4x2_yellow", layer_index=2):
                self.get_logger().info("✅🎉 5단 아이스크림 완벽하게 완성!")

    def build_big_tree(self):
        self.get_logger().info("🧱 [초기화] 맨 처음 2x2_yellow 위치 스캔 및 기억")
        p_yellow = self.find_target_with_retry("2x2_yellow")
        if not p_yellow:
            self.get_logger().warn("❌ 바닥에 2x2_yellow가 안 보입니다. 조립을 취소합니다.")
            return
        saved_yellow_pose = p_yellow
        self.call(self.cli_h, Trigger.Request())

        self.get_logger().info("🧱 [Phase 1] 4x2_green(-1.85) 파지 -> 2x2_green(0.0) 결합 (그리퍼 유지)")
        if self.pick_target("4x2_green", offset_studs_y=0):
            self.call(self.cli_h, Trigger.Request())
            if self.visual_insert("4x2_green", layer_index=1, offset_studs_y=-1.0, release_gripper=False):
                self.call(self.cli_h, Trigger.Request())
                time.sleep(1.0) 

                self.get_logger().info("🧱 [Phase 2] 바닥의 다른 4x2_green 스캔 및 6x2 조립 (그리퍼 해제)")
                p_2x2_base = self.find_target_with_retry("2x2_green")
                if not p_2x2_base:
                    self.get_logger().warn("❌ 바닥에 다른 2x2_green가 안 보입니다.")
                    return
                saved_6x2_pose = p_2x2_base 
                
                if self.blind_insert(saved_6x2_pose, layer_index=1, offset_studs_y=2.0, release_gripper=True):
                    self.call(self.cli_h, Trigger.Request())

                    self.get_logger().info("🧱 [Phase 3] 2x2_red 파지 -> 6x2 중심에 결합 (그리퍼 유지)")
                    if self.pick_target("2x2_green", offset_studs_y=-0.2):
                        self.call(self.cli_h, Trigger.Request())
                        if self.blind_insert(saved_6x2_pose, layer_index=2, offset_studs_y=0.0, release_gripper=False):
                            self.call(self.cli_h, Trigger.Request())

                            self.get_logger().info("🧱 [Phase 4] 덩어리를 2x2_yellow 중앙에 최종 결합")
                            if self.blind_insert(saved_yellow_pose, layer_index=2.5, offset_studs_y=0.0):
                                self.get_logger().info("✅ 최종 조립 시퀀 완벽 종료!")

    def run(self):
        self.get_logger().info("🚀 STARTING VISUAL-STACK ASSEMBLY SEQUENCE (Keyboard Select Mode)")
        home_res = self.call(self.cli_h, Trigger.Request())
        if home_res is None or not home_res.success:
            self.get_logger().error("❌ 시작 HOME 이동 실패. 조립을 시작하지 않습니다.")
            return
        self.call(self.cli_g, SetBool.Request(data=False))
        time.sleep(1.0)
       
        actions = {
            "1": self.build_battery, "battery": self.build_battery, "배터리": self.build_battery,
            "2": self.build_magnet, "magnet": self.build_magnet, "자석": self.build_magnet,
            "3": self.build_e_stop, "estop": self.build_e_stop, "비상정지": self.build_e_stop,
            "4": self.build_carrot, "carrot": self.build_carrot, "당근": self.build_carrot,
            "5": self.build_traffic_light, "traffic": self.build_traffic_light, "신호등": self.build_traffic_light,
            "6": self.build_small_tree, "tree": self.build_small_tree, "작은나무": self.build_small_tree,
            "7": self.build_hammer, "hammer": self.build_hammer, "망치": self.build_hammer,
            "8": self.build_big_carrot, "bigcarrot": self.build_big_carrot, "큰당근": self.build_big_carrot,
            "9": self.build_burger, "burger": self.build_burger, "버거": self.build_burger,
            "10": self.build_ice_cream, "icecream": self.build_ice_cream, "아이스크림": self.build_ice_cream,
            "11": self.build_big_tree, "big_tree": self.build_big_tree
        }

        print("\n=== Master Node Assembly Keyboard Select ===")
        print("1: 배터리 / 2: 자석 / 3: 비상정지 / 4: 당근 / 5: 신호등 / 6: 작은나무")
        print("7: 망치 / 8: 큰당근 / 9: 버거 / 10: 아이스크림 / 11: big_tree")
        print("q: 종료")

        while rclpy.ok():
            user_input = input("\n조립할 항목을 선택하세요 [1~11/q]: ").strip().replace(" ", "").lower()
            if user_input in ("q", "quit", "exit", "종료"):
                self.get_logger().info("조립 시퀀스를 종료합니다.")
                break

            action = actions.get(user_input)
            if action is None:
                print("잘못된 입력입니다. 1~11 또는 q 중에서 선택하세요.")
                continue

            self.get_logger().info(f"▶️ 작업 시작: {user_input}")
            action()
            
            self.call(self.cli_h, Trigger.Request())
            time.sleep(1.0)
            self.get_logger().info("✅ 개별 조립 완료")

        self.call(self.cli_h, Trigger.Request())
        self.get_logger().info("🎉 ALL SEQUENCE DONE")

def main():
    rclpy.init()
    node = MasterNode()
    node.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()