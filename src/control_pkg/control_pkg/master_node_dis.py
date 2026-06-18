# import time
# ### 최최최신
# import rclpy
# from rclpy.node import Node
# from srvs_pkg.srv import GetTargetPose
# from std_srvs.srv import SetBool, Trigger


# class BatteryDualDisassembly(Node):
#     def __init__(self, node_name="master_node_dis11"):
#         super().__init__(node_name)

#         self.cli_v1 = self.create_client(GetTargetPose, "/get_target_pose")
#         self.cli_r1 = self.create_client(GetTargetPose, "/robot1/robot_move_step")
#         self.cli_r2 = self.create_client(GetTargetPose, "/robot2/robot_move_step")
#         self.cli_h1 = self.create_client(Trigger, "/robot1/robot_home")
#         self.cli_h2 = self.create_client(Trigger, "/robot2/robot_home")

#         self.robot1_gripper_service = self.declare_parameter(
#             "robot1_gripper_service",
#             "/control_gripper",
#         ).value
#         self.robot2_gripper_service = self.declare_parameter(
#             "robot2_gripper_service",
#             "/robot2/control_gripper",
#         ).value
#         self.cli_g1 = self.create_client(SetBool, self.robot1_gripper_service)
#         self.cli_g2 = self.create_client(SetBool, self.robot2_gripper_service)

#         self.wait_time = float(self.declare_parameter("wait_time", 0.7).value)
#         self.grip_wait_time = float(self.declare_parameter("grip_wait_time", 1.3).value)

#         self.z_off = float(self.declare_parameter("robot1_z_off", -105.0).value)
#         self.z_margin = float(self.declare_parameter("robot1_z_margin", 20.0).value)
#         self.robot1_pre_xy_lower_mm = float(self.declare_parameter("robot1_pre_xy_lower_mm", 70.0).value)
#         self.robot1_initial_lift_mm = float(self.declare_parameter("robot1_initial_lift_mm", -20.0).value)
#         self.robot1_pull_up_mm = float(self.declare_parameter("robot1_pull_up_mm", -30.0).value)

#         self.robot1_cam_x_off = -53.0
#         self.robot1_cam_y_off = 32.0

#         self.class_to_target_id = {
#             "2x2_red": "1",
#             "2x2_green": "2",
#             "2x2_blue": "3",
#             "2x2_yellow": "4",
#             "4x2_red": "5",
#             "4x2_green": "6",
#             "4x2_blue": "7",
#             "4x2_yellow": "8",
#             "assembly": "999",
#             "Magnet": "13",
#             "Battery": "34",
#             "estop": "81",
#             "traffic light": "241",
#             "carrot": "442",
#             "small tree": "462",
#             "hammer": "711",
#             "big carrot": "4482",
#             "burger": "8518",
#             "bigtree": "46262",
#             "icecream": "48132",
#         }

#         self.get_logger().info(
#             "Dual disassembly ready (keyboard selectable sequence). "
#             f"g1={self.robot1_gripper_service}, g2={self.robot2_gripper_service}"
#         )

#     def call(self, cli, req):
#         while not cli.wait_for_service(timeout_sec=1.0):
#             self.get_logger().info(f"Waiting for {cli.srv_name}...")
#         future = cli.call_async(req)
#         rclpy.spin_until_future_complete(self, future)
#         return future.result()

#     def sleep(self):
#         time.sleep(self.wait_time)

#     def normalize_yaw(self, yaw):
#         while yaw > 90.0:
#             yaw -= 180.0
#         while yaw < -90.0:
#             yaw += 180.0
#         return yaw

#     def pick_wrist_yaw(self, yaw):
#         while yaw > 0.0:
#             yaw -= 180.0
#         while yaw < -180.0:
#             yaw += 180.0
#         return yaw

#     def set_gripper(self, cli, closed):
#         res = self.call(cli, SetBool.Request(data=closed))
#         time.sleep(self.grip_wait_time)
#         return res.success

#     def move_z(self, cli, dz_mm):
#         req = GetTargetPose.Request()
#         req.target_size = "Z"
#         req.z = dz_mm
#         return self.call(cli, req).success

#     def find_target_with_retry(self, color, retries=3):
#         for i in range(retries):
#             p = self.request_vision_pose(color)
#             if p:
#                 return p
#             time.sleep(0.3) 
#         return None

#     def normalize_target_name(self, target):
#         target = str(target)
#         aliases = {
#             "2x4_red": "4x2_red",
#             "2x4_green": "4x2_green",
#             "2x4_blue": "4x2_blue",
#             "2x4_yellow": "4x2_yellow",
#         }
#         return aliases.get(target, target)

#     def request_vision_pose(self, target):
#         target_name = self.normalize_target_name(target)
#         request_target = self.class_to_target_id.get(target_name, target_name)
#         req = GetTargetPose.Request()
#         req.target_color = request_target

#         self.get_logger().info(f"비전 요청: target={target_name}, target_color={request_target}")
#         p = self.call(self.cli_v1, req)

#         if p is None:
#             self.get_logger().error(f"비전 응답 없음: {target_name}({request_target})")
#             return None
#         if not p.success:
#             self.get_logger().warn(f"비전 탐색 실패: {target_name}({request_target})")
#             return None

#         self.get_logger().info(
#             f"비전 응답: request={target_name}({request_target}), class={p.class_name}, "
#             f"x={p.x * 1000.0:.1f}mm, y={p.y * 1000.0:.1f}mm, "
#             f"z={p.z * 1000.0:.1f}mm, yaw={p.yaw:.1f}deg"
#         )
#         return p

#     def target_fallbacks(self, target):
#         if target.startswith("2x2_"):
#             return [target, target.replace("2x2_", "4x2_", 1)]
#         if target.startswith("4x2_"):
#             return [target, target.replace("4x2_", "2x2_", 1)]
#         if target.startswith("2x4_"):
#             return [target, target.replace("2x4_", "2x2_", 1)]
#         return [target]

#     def find_target_candidate_with_retry(self, target, retries=3, use_base_nearest=False):
#         candidates = self.target_fallbacks(target)
#         for _ in range(retries):
#             for candidate in candidates:
#                 p = self.request_vision_pose(candidate)
#                 if p:
#                     if candidate != target:
#                         self.get_logger().warn(f"{target} 대신 대체 타겟 {candidate}로 진행")
#                     return p, candidate
#                 time.sleep(0.3)
#         return None, None

#     def move_robot1_separation_pose(self):
#         req = GetTargetPose.Request()
#         req.target_size = "SEPARATION"
#         return self.call(self.cli_r1, req).success

#     def move_robot2_separation_pose(self):
#         req = GetTargetPose.Request()
#         req.target_size = "SEPARATION"
#         return self.call(self.cli_r2, req).success

#     def move_robot1_drop_pose(self):
#         req = GetTargetPose.Request()
#         req.target_size = "DROP"
#         return self.call(self.cli_r1, req).success

#     def move_robot1_center_pose(self):
#         req = GetTargetPose.Request()
#         req.target_size = "CENTER"
#         return self.call(self.cli_r1, req).success

#     def move_robot1_drop2_pose(self):
#         req = GetTargetPose.Request()
#         req.target_size = "DROP2"
#         return self.call(self.cli_r1, req).success

#     def move_robot1_drop3_pose(self):
#         req = GetTargetPose.Request()
#         req.target_size = "DROP3"
#         return self.call(self.cli_r1, req).success

#     def move_robot1_drop4_pose(self):
#         req = GetTargetPose.Request()
#         req.target_size = "DROP4"
#         return self.call(self.cli_r1, req).success

#     def move_robot1_drop5_pose(self):
#         req = GetTargetPose.Request()
#         req.target_size = "DROP5"
#         return self.call(self.cli_r1, req).success

#     def move_robot2_drop_pose(self):
#         req = GetTargetPose.Request()
#         req.target_size = "DROP"
#         return self.call(self.cli_r2, req).success

#     def move_robot2_drop2_pose(self):
#         req = GetTargetPose.Request()
#         req.target_size = "DROP2"
#         return self.call(self.cli_r2, req).success

#     def move_robot1_end_pose(self):
#         req = GetTargetPose.Request()
#         req.target_size = "END"
#         return self.call(self.cli_r1, req).success

#     def move_robot2_end_pose(self):
#         req = GetTargetPose.Request()
#         req.target_size = "END"
#         return self.call(self.cli_r2, req).success

#     def move_both_home_pose(self):
#         self.get_logger().info("노드 시작/시퀀스 시작: robot1/robot2 HOME 조인트로 이동")
#         self.call(self.cli_h1, Trigger.Request())
#         self.call(self.cli_h2, Trigger.Request())
#         self.sleep()
#         return True

#     def move_both_end_pose(self):
#         self.get_logger().info("시퀀스 종료: robot1/robot2 END 조인트로 이동")
#         ok1 = self.move_robot1_end_pose()
#         ok2 = self.move_robot2_end_pose()
#         self.sleep()
#         return ok1 and ok2

#     def robot1_top_pick(self, target, top_label, expected_layer=None, yaw_offset=0.0, z_extra_mm=0.0):
#         self.get_logger().info(
#             f"1) robot1: 비전 3단계 스캔 시작 [{target}] "
#             f"(Yaw Offset: {yaw_offset}도, Z Extra: {z_extra_mm:.1f}mm)"
#         )
#         use_base_nearest = expected_layer == 1

#         p, selected_target = self.find_target_candidate_with_retry(target, use_base_nearest=use_base_nearest)
#         if not p:
#             self.get_logger().error("robot1: 1차 스캔(Yaw) 실패")
#             return False
            
#         req_yaw = GetTargetPose.Request()
#         req_yaw.target_size = "YAW"
#         req_yaw.yaw = self.pick_wrist_yaw(p.yaw + yaw_offset)
#         self.call(self.cli_r1, req_yaw)
#         self.sleep()

#         if abs(self.robot1_pre_xy_lower_mm) > 0.1:
#             self.get_logger().info(f"robot1: XY 이동 전 Z {self.robot1_pre_xy_lower_mm:.1f}mm 선하강")
#             self.move_z(self.cli_r1, self.robot1_pre_xy_lower_mm)
#             self.sleep()

#         p, selected_target = self.find_target_candidate_with_retry(target, use_base_nearest=use_base_nearest)
#         if not p:
#             self.get_logger().error("robot1: 2차 스캔(XY) 실패")
#             return False
#         req_xy = GetTargetPose.Request()
#         req_xy.target_size = "XY"
#         req_xy.x = p.x
#         req_xy.y = p.y
#         self.call(self.cli_r1, req_xy)
#         self.sleep()

#         p, selected_target = self.find_target_candidate_with_retry(target, use_base_nearest=use_base_nearest)
#         if not p:
#             self.get_logger().error("robot1: 3차 스캔(Z) 실패")
#             return False
        
#         z_move = (p.z * 1000.0) + self.z_off + z_extra_mm
#         self.move_z(self.cli_r1, z_move - self.z_margin)
#         self.sleep()
#         self.move_z(self.cli_r1, self.z_margin)
#         self.sleep()

#         self.set_gripper(self.cli_g1, True)
#         self.sleep()
        
#         self.move_z(self.cli_r1, self.robot1_initial_lift_mm)
#         self.sleep()

#         self.get_logger().info("robot1: 물체 분리 자세 이동")
#         self.move_robot1_separation_pose()
#         self.sleep()
#         return True

#     def robot2_side_hold(self, bottom_label):
#         self.get_logger().info(f"2) robot2: 지정된 분리 조인트로 이동하여 하단({bottom_label}) 고정")
#         if not self.move_robot2_separation_pose():
#             self.get_logger().error("robot2: 고정 자세 이동 실패")
#             return False
#         self.sleep()

#         self.set_gripper(self.cli_g2, True)
#         return True

#     def robot1_pull_up(self, top_label):
#         self.get_logger().info(f"3) robot1: 상단({top_label}) 블럭 3cm 추가 상승하여 강제 분리")
#         self.move_z(self.cli_r1, self.robot1_pull_up_mm)
#         self.sleep()
#         return True

#     def robot2_return_home_holding(self, bottom_label):
#         self.get_logger().info(f"4) robot2: 하단({bottom_label}) 블럭 잡은 상태로 홈 위치 복귀")
#         self.call(self.cli_h2, Trigger.Request())
#         self.sleep()
#         return True

#     def robot2_release_and_home(self, bottom_label):
#         self.get_logger().info(f"6) robot2: 하단({bottom_label}) 고정 해제 후 홈 위치 복귀")
#         self.set_gripper(self.cli_g2, False)
#         self.sleep()
#         self.call(self.cli_h2, Trigger.Request())
#         self.sleep()
#         return True

#     def robot1_place_top_at_center_and_home(self, top_label):
#         self.get_logger().info(f"5) robot1: 상단({top_label}) 블럭 CENTER 조인트에 임시 배치 후 홈 복귀")
#         if not self.move_robot1_center_pose():
#             self.get_logger().error("robot1: CENTER 자세 이동 실패")
#             return False
#         self.sleep()

#         self.set_gripper(self.cli_g1, False)
#         self.sleep()

#         self.call(self.cli_h1, Trigger.Request())
#         self.sleep()
#         return True

#     def robot1_pick_layer_place_center_robot2_drop(
#         self,
#         target,
#         label,
#         expected_layer,
#         bottom_label,
#         robot2_drop_slot,
#         yaw_offset=0.0,
#         z_extra_mm=0.0,
#     ):
#         self.get_logger().info(f"{expected_layer}층 분해 시작: {label} / robot1 CENTER 임시배치, robot2 {robot2_drop_slot} 드롭")
#         if not self.robot1_top_pick(
#             target, label, expected_layer=expected_layer,
#             yaw_offset=yaw_offset, z_extra_mm=z_extra_mm
#         ):
#             self.get_logger().error(f"{expected_layer}층 {label} 분해 실패")
#             return False
#         if not self.robot2_side_hold(bottom_label):
#             return False
#         if not self.robot1_pull_up(label):
#             self.robot2_drop_bottom_and_home(bottom_label, drop_slot=robot2_drop_slot)
#             return False
#         if not self.robot2_return_home_holding(bottom_label):
#             return False
#         if not self.robot1_place_top_at_center_and_home(label):
#             self.robot2_drop_bottom_and_home(bottom_label, drop_slot=robot2_drop_slot)
#             return False
#         if not self.robot2_drop_bottom_and_home(bottom_label, drop_slot=robot2_drop_slot):
#             return False
#         return True

#     def robot1_drop_top_and_home(self, top_label, drop_slot="DROP"):
#         self.get_logger().info(f"5) robot1: 상단({top_label}) 블럭 {drop_slot} 조인트로 이동하여 내려놓고 홈 복귀")
#         if drop_slot == "DROP2":
#             moved = self.move_robot1_drop2_pose()
#         elif drop_slot == "DROP3":
#             moved = self.move_robot1_drop3_pose()
#         elif drop_slot == "DROP4":
#             moved = self.move_robot1_drop4_pose()
#         elif drop_slot == "DROP5":
#             moved = self.move_robot1_drop5_pose()
#         else:
#             moved = self.move_robot1_drop_pose()

#         if not moved:
#             self.get_logger().error(f"robot1: {drop_slot} 자세 이동 실패")
#             return False
#         self.sleep()
        
#         self.set_gripper(self.cli_g1, False)
#         self.sleep()
        
#         self.call(self.cli_h1, Trigger.Request())
#         self.sleep()
#         return True

#     def robot2_drop_bottom_and_home(self, bottom_label, drop_slot="DROP"):
#         self.get_logger().info(f"6) robot2: 하단({bottom_label}) 블럭 {drop_slot} 조인트로 이동하여 내려놓고 홈 복귀")
#         if drop_slot == "DROP2":
#             moved = self.move_robot2_drop2_pose()
#         else:
#             moved = self.move_robot2_drop_pose()

#         if not moved:
#             self.get_logger().error(f"robot2: {drop_slot} 자세 이동 실패")
#             return False
#         self.sleep()
        
#         self.set_gripper(self.cli_g2, False)
#         self.sleep()

#         self.call(self.cli_h2, Trigger.Request())
#         self.sleep()
#         return True

#     def run_two_layer_once(self, name, layers):
#         top_layer = layers[0]
#         bottom_layer = layers[1]
#         top_target = top_layer[1]
#         top_label = top_layer[2]
#         yaw_offset = top_layer[3] if len(top_layer) > 3 else 0.0
#         bottom_label = bottom_layer[2]

#         self.get_logger().info(f"{name} 협조 분해 시작: 2층 {top_label} / 1층 {bottom_label}")
#         self.move_both_home_pose()
#         self.set_gripper(self.cli_g1, False)
#         self.set_gripper(self.cli_g2, False)

#         if not self.robot1_top_pick(top_target, top_label, expected_layer=2, yaw_offset=yaw_offset): return False
#         if not self.robot2_side_hold(bottom_label): return False
#         if not self.robot1_pull_up(top_label): return False
#         if not self.robot2_return_home_holding(bottom_label): return False
#         if not self.robot1_drop_top_and_home(top_label): return False
#         if not self.robot2_drop_bottom_and_home(bottom_label): return False

#         self.get_logger().info(f"🎉 {name} 협조 분해 완벽 종료")
#         self.move_both_end_pose()
#         return True

#     def run_battery_once(self):
#         return self.run_two_layer_once("배터리", [
#             (2, "2x2_yellow", "2x2 노랑", 0.0),
#             (1, "2x2_blue", "2x2 파랑", 0.0),
#         ])

#     def run_magnet_once(self):
#         return self.run_two_layer_once("자석", [
#             (2, "2x2_blue", "2x2 파랑", 0.0),
#             (1, "2x2_red", "2x2 빨강", 0.0),
#         ])

#     def run_estop_once(self):
#         return self.run_two_layer_once("E-stop", [
#             (2, "2x2_red", "2x2 빨강", 90.0),
#             (1, "2x4_yellow", "2x4 노랑", 0.0),
#         ])

#     def robot1_pick_layer_and_drop(
#         self,
#         target,
#         label,
#         expected_layer,
#         drop_slot,
#         bottom_label=None,
#         yaw_offset=0.0,
#         robot2_drop_slot=None,
#         z_extra_mm=0.0,
#     ):
#         self.get_logger().info(f"{expected_layer}층 분해 시작: {label}")
#         if not self.robot1_top_pick(
#             target, label, expected_layer=expected_layer,
#             yaw_offset=yaw_offset, z_extra_mm=z_extra_mm
#         ):
#             self.get_logger().error(f"{expected_layer}층 {label} 분해 실패")
#             return False

#         if bottom_label is not None:
#             robot2_drop_slot = robot2_drop_slot or ("DROP2" if expected_layer in (2, 3) else "DROP")
#             if not self.robot2_side_hold(bottom_label):
#                 return False
#             if not self.robot1_pull_up(label):
#                 self.robot2_drop_bottom_and_home(bottom_label, drop_slot=robot2_drop_slot)
#                 return False

#             if not self.robot2_return_home_holding(bottom_label):
#                 return False

#             if not self.robot1_drop_top_and_home(label, drop_slot=drop_slot):
#                 self.robot2_drop_bottom_and_home(bottom_label, drop_slot=robot2_drop_slot)
#                 return False

#             if not self.robot2_drop_bottom_and_home(bottom_label, drop_slot=robot2_drop_slot):
#                 return False
#             return True

#         return self.robot1_drop_top_and_home(label, drop_slot=drop_slot)

#     def robot1_pick_layer_drop_top_keep_robot2(
#         self,
#         target,
#         label,
#         expected_layer,
#         drop_slot,
#         bottom_label,
#         yaw_offset=0.0,
#         z_extra_mm=0.0,
#     ):
#         self.get_logger().info(f"{expected_layer}층 분해 시작: {label} / robot2는 {bottom_label} 잡은 상태 유지")
#         if not self.robot1_top_pick(
#             target, label, expected_layer=expected_layer,
#             yaw_offset=yaw_offset, z_extra_mm=z_extra_mm
#         ):
#             self.get_logger().error(f"{expected_layer}층 {label} 분해 실패")
#             return False

#         if not self.robot2_side_hold(bottom_label):
#             return False
#         if not self.robot1_pull_up(label):
#             return False
#         if not self.robot2_return_home_holding(bottom_label):
#             return False
#         if not self.robot1_drop_top_and_home(label, drop_slot=drop_slot):
#             return False
#         return True

#     def robot1_pick_layer_drop_top_release_robot2(
#         self,
#         target,
#         label,
#         expected_layer,
#         drop_slot,
#         bottom_label,
#         yaw_offset=0.0,
#         z_extra_mm=0.0,
#     ):
#         self.get_logger().info(f"{expected_layer}층 분해 시작: {label} / robot2는 하단 고정 후 제자리 해제")
#         if not self.robot1_top_pick(
#             target, label, expected_layer=expected_layer,
#             yaw_offset=yaw_offset, z_extra_mm=z_extra_mm
#         ):
#             self.get_logger().error(f"{expected_layer}층 {label} 분해 실패")
#             return False

#         if not self.robot2_side_hold(bottom_label):
#             return False
#         if not self.robot1_pull_up(label):
#             self.robot2_release_and_home(bottom_label)
#             return False
#         if not self.robot1_drop_top_and_home(label, drop_slot=drop_slot):
#             self.robot2_release_and_home(bottom_label)
#             return False
#         if not self.robot2_release_and_home(bottom_label):
#             return False
#         return True

#     def robot1_pick_layer_drop_top_then_robot2_drop(
#         self,
#         target,
#         label,
#         expected_layer,
#         drop_slot,
#         bottom_label,
#         robot2_drop_slot,
#         yaw_offset=0.0,
#         z_extra_mm=0.0,
#     ):
#         self.get_logger().info(
#             f"{expected_layer}층 분해 시작: {label} / "
#             f"robot2 HOME 복귀 후 robot1 {drop_slot}, robot2 {robot2_drop_slot}"
#         )
#         if not self.robot1_top_pick(
#             target, label, expected_layer=expected_layer,
#             yaw_offset=yaw_offset, z_extra_mm=z_extra_mm
#         ):
#             self.get_logger().error(f"{expected_layer}층 {label} 분해 실패")
#             return False

#         if not self.robot2_side_hold(bottom_label):
#             return False
#         if not self.robot1_pull_up(label):
#             self.robot2_drop_bottom_and_home(bottom_label, drop_slot=robot2_drop_slot)
#             return False
#         if not self.robot2_return_home_holding(bottom_label):
#             return False
#         if not self.robot1_drop_top_and_home(label, drop_slot=drop_slot):
#             self.robot2_drop_bottom_and_home(bottom_label, drop_slot=robot2_drop_slot)
#             return False
#         if not self.robot2_drop_bottom_and_home(bottom_label, drop_slot=robot2_drop_slot):
#             return False
#         return True

#     def run_three_layer_once(self, name, layers):
#         self.get_logger().info(f"{name} 3층 분해 시작")
#         self.move_both_home_pose()
#         self.set_gripper(self.cli_g1, False)
#         self.set_gripper(self.cli_g2, False)

#         drop_slots = {3: "DROP", 2: "DROP2", 1: "DROP3"}

#         for index, layer_info in enumerate(layers):
#             layer = layer_info[0]
#             target = layer_info[1]
#             label = layer_info[2]
#             yaw_offset = layer_info[3] if len(layer_info) > 3 else 0.0
#             drop_slot = layer_info[4] if len(layer_info) > 4 else drop_slots.get(layer, "DROP")
            
#             bottom_label = layers[index + 1][2] if index + 1 < len(layers) else None
            
#             if not self.robot1_pick_layer_and_drop(
#                 target, label, layer, drop_slot,
#                 bottom_label=bottom_label, yaw_offset=yaw_offset
#             ):
#                 return False

#         self.get_logger().info(f"🎉 {name} 3층 분해 완료")
#         self.move_both_end_pose()
#         return True

#     def run_four_layer_once(self, name, layers):
#         self.get_logger().info(f"{name} 4층 분해 시작")
#         self.move_both_home_pose()
#         self.set_gripper(self.cli_g1, False)
#         self.set_gripper(self.cli_g2, False)

#         drop_slots = {4: "DROP", 3: "DROP2", 2: "DROP3", 1: "DROP4"}

#         for index, layer_info in enumerate(layers):
#             layer = layer_info[0]
#             target = layer_info[1]
#             label = layer_info[2]
#             yaw_offset = layer_info[3] if len(layer_info) > 3 else 0.0
#             drop_slot = layer_info[4] if len(layer_info) > 4 else drop_slots.get(layer, "DROP")
            
#             bottom_label = layers[index + 1][2] if index + 1 < len(layers) else None
            
#             if not self.robot1_pick_layer_and_drop(
#                 target, label, layer, drop_slot,
#                 bottom_label=bottom_label, yaw_offset=yaw_offset
#             ):
#                 return False

#         self.get_logger().info(f"🎉 {name} 4층 분해 완료")
#         self.move_both_end_pose()
#         return True

#     def run_carrot_once(self):
#         return self.run_three_layer_once("당근", [
#             (3, "2x2_green", "2x2 초록"), (2, "2x2_yellow", "2x2 노랑"), (1, "2x2_yellow", "2x2 노랑")
#         ])

#     def run_small_tree_once(self):
#         return self.run_three_layer_once("작은 나무", [
#             (3, "2x2_green", "2x2 초록"), (2, "4x2_green", "4x2 초록"), (1, "2x2_yellow", "2x2 노랑")
#         ])

#     def run_traffic_light_once(self):
#         return self.run_three_layer_once("신호등", [
#             (3, "2x2_red", "2x2 빨강"), (2, "2x2_yellow", "2x2 노랑"), (1, "2x2_green", "2x2 초록")
#         ])

#     def run_hammer_once(self):
#         return self.run_three_layer_once("망치", [
#             (3, "4x2_blue", "4x2 파랑", 0.0),
#             (2, "2x2_red", "2x2 빨강", 0.0),
#             (1, "2x2_red", "2x2 빨강", 0.0)
#         ])

#     def run_big_carrot_once(self):
#         return self.run_four_layer_once("큰 당근", [
#             (4, "2x2_green", "2x2 초록", 0.0), 
#             (3, "4x2_yellow", "4x2 노랑", 0.0),
#             (2, "2x2_yellow", "2x2 노랑", 0.0),
#             (1, "2x2_yellow", "2x2 노랑", 0.0),
#         ])

#     # ==========================================
#     # 💥 신규 추가: 버거, 아이스크림, 큰 나무 해체 시퀀스
#     # ==========================================
#     def run_burger_once(self):
#         return self.run_four_layer_once("버거", [
#             (4, "4x2_yellow", "4x2 노랑", 0.0),
#             (3, "2x2_red", "2x2 빨강", 0.0),
#             (2, "4x2_red", "4x2 빨강", 0.0),
#             (1, "4x2_yellow", "4x2 노랑", 0.0),
#         ])

#     def run_ice_cream_once(self):
#         return self.run_four_layer_once("아이스크림", [
#             (4, "2x2_green", "2x2 초록", 0.0, "DROP"),
#             (3, "2x2_blue", "2x2 파랑", 0.0, "DROP2"),
#             (3, "2x2_red", "2x2 빨강", 0.0, "DROP3"),
#             (2, "4x2_yellow", "4x2 노랑", 0.0, "DROP4"),
#             (1, "2x2_yellow", "2x2 노랑", 0.0, "DROP5"),
#         ])

#     def run_big_tree_once(self):
#         self.get_logger().info("큰 나무 특수 분해 시작: 4층 -> 1층 노랑 이송 -> 남은 3개")
#         self.move_both_home_pose()
#         self.set_gripper(self.cli_g1, False)
#         self.set_gripper(self.cli_g2, False)

#         if not self.robot1_pick_layer_and_drop(
#             "2x2_green", "4층 2x2 초록", 4, "DROP",
#             bottom_label="4x2 초록", yaw_offset=0.0
#         ):
#             return False

#         if not self.robot1_pick_layer_place_center_robot2_drop(
#             "4x2_green", "1층 분리용 2층 초록", 2,
#             bottom_label="2x2 노랑", yaw_offset=0.0,
#             robot2_drop_slot="DROP2", z_extra_mm=16.0
#         ):
#             return False

#         if not self.robot1_pick_layer_and_drop(
#             "2x2_yellow", "1층 2x2 노랑", 1, "DROP2",
#             yaw_offset=0.0
#         ):
#             return False

#         if not self.robot1_pick_layer_drop_top_then_robot2_drop(
#             "4x2_green", "남은 3층 4x2 초록", 3, "DROP3",
#             bottom_label="2층 초록", robot2_drop_slot="DROP2", yaw_offset=0.0
#         ):
#             return False

#         if not self.robot1_pick_layer_and_drop(
#             "4x2_green", "남은 2층 4x2 초록", 2, "DROP4",
#             yaw_offset=0.0
#         ):
#             return False

#         if not self.robot1_pick_layer_and_drop(
#             "2x2_green", "남은 2층 2x2 초록", 2, "DROP5",
#             yaw_offset=0.0
#         ):
#             return False

#         self.get_logger().info("🎉 큰 나무 특수 분해 완료")
#         self.move_both_end_pose()
#         return True

#     def run(self):
#         self.move_both_home_pose()

#         print("\n=== Master Node Dis10 Keyboard Select ===")
#         print("1: 배터리  (2층 2x2 노랑 / 1층 2x2 파랑)")
#         print("2: 자석    (2층 2x2 파랑 / 1층 2x2 빨강)")
#         print("3: E-stop  (2층 2x2 빨강 / 1층 2x4 노랑)")
#         print("4: 당근    (3층 2x2 초록 / 2층 2x2 노랑 / 1층 2x2 노랑)")
#         print("5: 작은 나무 (3층 2x2 초록 / 2층 4x2 초록 / 1층 2x2 노랑)")
#         print("6: 신호등  (3층 2x2 빨강 / 2층 2x2 노랑 / 1층 2x2 초록)")
#         print("7: 망치    (3층 4x2 파랑 / 2층 2x2 빨강 / 1층 2x2 빨강)")
#         print("8: 큰 당근 (4층 2x2 초록 / 3층 4x2 노랑 / 2층 2x2 노랑 / 1층 2x2 노랑)")
#         print("9: 버거    (4층 4x2 노랑 / 3층 2x2 빨강 / 2층 4x2 빨강 / 1층 4x2 노랑)")
#         print("10: 아이스크림 (4층 2x2 초록 / 3층 2x2 파랑,빨강 / 2층 4x2 노랑 / 1층 2x2 노랑)")
#         print("11: 큰 나무 (4층 2x2 초록 / 3층 4x2 초록 / 2층 4x2,2x2 초록 / 1층 2x2 노랑)")
#         print("q: 종료")

#         actions = {
#             "1": self.run_battery_once, "battery": self.run_battery_once, "배터리": self.run_battery_once,
#             "2": self.run_magnet_once, "magnet": self.run_magnet_once, "자석": self.run_magnet_once,
#             "3": self.run_estop_once, "estop": self.run_estop_once, "e-stop": self.run_estop_once, "비상정지": self.run_estop_once,
#             "4": self.run_carrot_once, "carrot": self.run_carrot_once, "당근": self.run_carrot_once,
#             "5": self.run_small_tree_once, "tree": self.run_small_tree_once, "small_tree": self.run_small_tree_once, "작은나무": self.run_small_tree_once,
#             "6": self.run_traffic_light_once, "traffic": self.run_traffic_light_once, "traffic_light": self.run_traffic_light_once, "신호등": self.run_traffic_light_once,
#             "7": self.run_hammer_once, "hammer": self.run_hammer_once, "망치": self.run_hammer_once,
#             "8": self.run_big_carrot_once, "bigcarrot": self.run_big_carrot_once, "큰당근": self.run_big_carrot_once, "큰 당근": self.run_big_carrot_once,
#             "9": self.run_burger_once, "burger": self.run_burger_once, "버거": self.run_burger_once,
#             "10": self.run_ice_cream_once, "icecream": self.run_ice_cream_once, "아이스크림": self.run_ice_cream_once,
#             "11": self.run_big_tree_once, "bigtree": self.run_big_tree_once, "큰나무": self.run_big_tree_once, "큰 나무": self.run_big_tree_once,
#         }

#         while rclpy.ok():
#             user_input = input("\n선택하세요 [1~11/q]: ").strip().replace(" ", "").lower()
#             if user_input in ("q", "quit", "exit", "종료"):
#                 self.get_logger().info("키보드 선택 모션을 종료합니다.")
#                 break

#             action = actions.get(user_input)
#             if action is None:
#                 print("잘못된 입력입니다. 1~11 또는 q 중에서 선택하세요.")
#                 continue

#             action()


# def main():
#     rclpy.init()
#     node = BatteryDualDisassembly()
#     try:
#         node.run()
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == "__main__":
#     main()

import time
### 최최최신
import rclpy
from rclpy.node import Node
from srvs_pkg.srv import GetTargetPose
from std_srvs.srv import SetBool, Trigger


class BatteryDualDisassembly(Node):
    def __init__(self, node_name="master_node_dis11"):
        super().__init__(node_name)

        self.cli_v1 = self.create_client(GetTargetPose, "/get_target_pose")
        self.cli_r1 = self.create_client(GetTargetPose, "/robot1/robot_move_step")
        self.cli_r2 = self.create_client(GetTargetPose, "/robot2/robot_move_step")
        self.cli_h1 = self.create_client(Trigger, "/robot1/robot_home")
        self.cli_h2 = self.create_client(Trigger, "/robot2/robot_home")

        self.robot1_gripper_service = self.declare_parameter(
            "robot1_gripper_service",
            "/control_gripper",
        ).value
        self.robot2_gripper_service = self.declare_parameter(
            "robot2_gripper_service",
            "/robot2/control_gripper",
        ).value
        self.cli_g1 = self.create_client(SetBool, self.robot1_gripper_service)
        self.cli_g2 = self.create_client(SetBool, self.robot2_gripper_service)

        self.wait_time = float(self.declare_parameter("wait_time", 0.7).value)
        self.grip_wait_time = float(self.declare_parameter("grip_wait_time", 1.3).value)

        self.z_off = float(self.declare_parameter("robot1_z_off", -105.0).value)
        self.z_margin = float(self.declare_parameter("robot1_z_margin", 20.0).value)
        self.robot1_pre_xy_lower_mm = float(self.declare_parameter("robot1_pre_xy_lower_mm", 70.0).value)
        self.robot1_initial_lift_mm = float(self.declare_parameter("robot1_initial_lift_mm", -20.0).value)
        self.robot1_pull_up_mm = float(self.declare_parameter("robot1_pull_up_mm", -30.0).value)

        self.robot1_cam_x_off = -53.0
        self.robot1_cam_y_off = 32.0

        self.class_to_target_id = {
            "2x2_red": "1",
            "2x2_green": "2",
            "2x2_blue": "3",
            "2x2_yellow": "4",
            "4x2_red": "5",
            "4x2_green": "6",
            "4x2_blue": "7",
            "4x2_yellow": "8",
            "assembly": "999",
            "Magnet": "13",
            "Battery": "34",
            "estop": "81",
            "traffic light": "241",
            "carrot": "442",
            "small tree": "462",
            "hammer": "711",
            "big carrot": "4482",
            "burger": "8518",
            "bigtree": "46262",
            "icecream": "48132",
        }

        self.get_logger().info(
            "Dual disassembly ready (keyboard selectable sequence). "
            f"g1={self.robot1_gripper_service}, g2={self.robot2_gripper_service}"
        )

    def call(self, cli, req):
        while not cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f"Waiting for {cli.srv_name}...")
        future = cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def sleep(self):
        time.sleep(self.wait_time)

    def normalize_yaw(self, yaw):
        while yaw > 90.0:
            yaw -= 180.0
        while yaw < -90.0:
            yaw += 180.0
        return yaw

    def pick_wrist_yaw(self, yaw):
        while yaw > 0.0:
            yaw -= 180.0
        while yaw < -180.0:
            yaw += 180.0
        return yaw

    def set_gripper(self, cli, closed):
        res = self.call(cli, SetBool.Request(data=closed))
        time.sleep(self.grip_wait_time)
        return res.success

    def move_z(self, cli, dz_mm):
        req = GetTargetPose.Request()
        req.target_size = "Z"
        req.z = dz_mm
        return self.call(cli, req).success

    def find_target_with_retry(self, color, retries=3):
        for i in range(retries):
            p = self.request_vision_pose(color)
            if p:
                return p
            time.sleep(0.3) 
        return None

    def normalize_target_name(self, target):
        target = str(target)
        aliases = {
            "2x4_red": "4x2_red",
            "2x4_green": "4x2_green",
            "2x4_blue": "4x2_blue",
            "2x4_yellow": "4x2_yellow",
        }
        return aliases.get(target, target)

    def request_vision_pose(self, target):
        target_name = self.normalize_target_name(target)
        request_target = self.class_to_target_id.get(target_name, target_name)
        req = GetTargetPose.Request()
        req.target_color = request_target

        self.get_logger().info(f"비전 요청: target={target_name}, target_color={request_target}")
        p = self.call(self.cli_v1, req)

        if p is None:
            self.get_logger().error(f"비전 응답 없음: {target_name}({request_target})")
            return None
        if not p.success:
            self.get_logger().warn(f"비전 탐색 실패: {target_name}({request_target})")
            return None

        self.get_logger().info(
            f"비전 응답: request={target_name}({request_target}), class={p.class_name}, "
            f"x={p.x * 1000.0:.1f}mm, y={p.y * 1000.0:.1f}mm, "
            f"z={p.z * 1000.0:.1f}mm, yaw={p.yaw:.1f}deg"
        )
        return p

    def target_fallbacks(self, target):
        if target.startswith("2x2_"):
            return [target, target.replace("2x2_", "4x2_", 1)]
        if target.startswith("4x2_"):
            return [target, target.replace("4x2_", "2x2_", 1)]
        if target.startswith("2x4_"):
            return [target, target.replace("2x4_", "2x2_", 1)]
        return [target]

    def find_target_candidate_with_retry(self, target, retries=3, use_base_nearest=False):
        candidates = self.target_fallbacks(target)
        for _ in range(retries):
            for candidate in candidates:
                p = self.request_vision_pose(candidate)
                if p:
                    if candidate != target:
                        self.get_logger().warn(f"{target} 대신 대체 타겟 {candidate}로 진행")
                    return p, candidate
                time.sleep(0.3)
        return None, None

    def move_robot1_separation_pose(self):
        req = GetTargetPose.Request()
        req.target_size = "SEPARATION"
        return self.call(self.cli_r1, req).success

    def move_robot2_separation_pose(self):
        req = GetTargetPose.Request()
        req.target_size = "SEPARATION"
        return self.call(self.cli_r2, req).success

    def move_robot1_drop_pose(self):
        req = GetTargetPose.Request()
        req.target_size = "DROP"
        return self.call(self.cli_r1, req).success

    def move_robot1_center_pose(self):
        req = GetTargetPose.Request()
        req.target_size = "CENTER"
        return self.call(self.cli_r1, req).success

    def move_robot1_drop2_pose(self):
        req = GetTargetPose.Request()
        req.target_size = "DROP2"
        return self.call(self.cli_r1, req).success

    def move_robot1_drop3_pose(self):
        req = GetTargetPose.Request()
        req.target_size = "DROP3"
        return self.call(self.cli_r1, req).success

    def move_robot1_drop4_pose(self):
        req = GetTargetPose.Request()
        req.target_size = "DROP4"
        return self.call(self.cli_r1, req).success

    def move_robot1_drop5_pose(self):
        req = GetTargetPose.Request()
        req.target_size = "DROP5"
        return self.call(self.cli_r1, req).success

    def move_robot2_drop_pose(self):
        req = GetTargetPose.Request()
        req.target_size = "DROP"
        return self.call(self.cli_r2, req).success

    def move_robot2_drop2_pose(self):
        req = GetTargetPose.Request()
        req.target_size = "DROP2"
        return self.call(self.cli_r2, req).success

    def move_robot1_end_pose(self):
        req = GetTargetPose.Request()
        req.target_size = "END"
        return self.call(self.cli_r1, req).success

    def move_robot2_end_pose(self):
        req = GetTargetPose.Request()
        req.target_size = "END"
        return self.call(self.cli_r2, req).success

    def move_both_home_pose(self):
        self.get_logger().info("노드 시작/시퀀스 시작: robot1/robot2 HOME 조인트로 이동")
        self.call(self.cli_h1, Trigger.Request())
        self.call(self.cli_h2, Trigger.Request())
        self.sleep()
        return True

    def move_both_end_pose(self):
        self.get_logger().info("시퀀스 종료: robot1/robot2 END 조인트로 이동")
        ok1 = self.move_robot1_end_pose()
        ok2 = self.move_robot2_end_pose()
        self.sleep()
        return ok1 and ok2

    def robot1_top_pick(self, target, top_label, expected_layer=None, yaw_offset=0.0, z_extra_mm=0.0):
        self.get_logger().info(
            f"1) robot1: 비전 1회 스캔 후 원샷 접근 시작 [{target}] "
            f"(Yaw Offset: {yaw_offset}도, Z Extra: {z_extra_mm:.1f}mm)"
        )
        use_base_nearest = expected_layer == 1

        # HOME에서 1회만 스캔 (재촬영 제거)
        p, selected_target = self.find_target_candidate_with_retry(target, use_base_nearest=use_base_nearest)
        if not p:
            self.get_logger().error("robot1: 비전 스캔 실패")
            return False

        # 손목 yaw (기존 분해 노드 규약 유지)
        target_yaw = self.pick_wrist_yaw(p.yaw + yaw_offset)
        # 접근 z (블록 위 z_margin 지점)
        z_move = (p.z * 1000.0) + self.z_off + z_extra_mm

        # [1] YAW + XY + Z 한 모션 접근 (APPROACH): 물체 바로 위 z_margin 지점까지
        self.get_logger().info(
            f"➡️ [ONE-SHOT] x={p.x:.4f}m, y={p.y:.4f}m, "
            f"yaw={target_yaw:.1f}도, z_approach={z_move - self.z_margin:.1f}mm"
        )
        req_app = GetTargetPose.Request()
        req_app.target_size = "APPROACH"
        req_app.x = p.x
        req_app.y = p.y
        req_app.z = z_move - self.z_margin
        req_app.yaw = target_yaw
        self.call(self.cli_r1, req_app)
        self.sleep()

        # [2] 최종 수직 하강
        self.move_z(self.cli_r1, self.z_margin)
        self.sleep()

        self.set_gripper(self.cli_g1, True)
        self.sleep()
        
        self.move_z(self.cli_r1, self.robot1_initial_lift_mm)
        self.sleep()

        self.get_logger().info("robot1: 물체 분리 자세 이동")
        self.move_robot1_separation_pose()
        self.sleep()
        return True

    def robot2_side_hold(self, bottom_label):
        self.get_logger().info(f"2) robot2: 지정된 분리 조인트로 이동하여 하단({bottom_label}) 고정")
        if not self.move_robot2_separation_pose():
            self.get_logger().error("robot2: 고정 자세 이동 실패")
            return False
        self.sleep()

        self.set_gripper(self.cli_g2, True)
        return True

    def robot1_pull_up(self, top_label):
        self.get_logger().info(f"3) robot1: 상단({top_label}) 블럭 3cm 추가 상승하여 강제 분리")
        self.move_z(self.cli_r1, self.robot1_pull_up_mm)
        self.sleep()
        return True

    def robot2_return_home_holding(self, bottom_label):
        self.get_logger().info(f"4) robot2: 하단({bottom_label}) 블럭 잡은 상태로 홈 위치 복귀")
        self.call(self.cli_h2, Trigger.Request())
        self.sleep()
        return True

    def robot2_release_and_home(self, bottom_label):
        self.get_logger().info(f"6) robot2: 하단({bottom_label}) 고정 해제 후 홈 위치 복귀")
        self.set_gripper(self.cli_g2, False)
        self.sleep()
        self.call(self.cli_h2, Trigger.Request())
        self.sleep()
        return True

    def robot1_place_top_at_center_and_home(self, top_label):
        self.get_logger().info(f"5) robot1: 상단({top_label}) 블럭 CENTER 조인트에 임시 배치 후 홈 복귀")
        if not self.move_robot1_center_pose():
            self.get_logger().error("robot1: CENTER 자세 이동 실패")
            return False
        self.sleep()

        self.set_gripper(self.cli_g1, False)
        self.sleep()

        self.call(self.cli_h1, Trigger.Request())
        self.sleep()
        return True

    def robot1_pick_layer_place_center_robot2_drop(
        self,
        target,
        label,
        expected_layer,
        bottom_label,
        robot2_drop_slot,
        yaw_offset=0.0,
        z_extra_mm=0.0,
    ):
        self.get_logger().info(f"{expected_layer}층 분해 시작: {label} / robot1 CENTER 임시배치, robot2 {robot2_drop_slot} 드롭")
        if not self.robot1_top_pick(
            target, label, expected_layer=expected_layer,
            yaw_offset=yaw_offset, z_extra_mm=z_extra_mm
        ):
            self.get_logger().error(f"{expected_layer}층 {label} 분해 실패")
            return False
        if not self.robot2_side_hold(bottom_label):
            return False
        if not self.robot1_pull_up(label):
            self.robot2_drop_bottom_and_home(bottom_label, drop_slot=robot2_drop_slot)
            return False
        if not self.robot2_return_home_holding(bottom_label):
            return False
        if not self.robot1_place_top_at_center_and_home(label):
            self.robot2_drop_bottom_and_home(bottom_label, drop_slot=robot2_drop_slot)
            return False
        if not self.robot2_drop_bottom_and_home(bottom_label, drop_slot=robot2_drop_slot):
            return False
        return True

    def robot1_drop_top_and_home(self, top_label, drop_slot="DROP"):
        self.get_logger().info(f"5) robot1: 상단({top_label}) 블럭 {drop_slot} 조인트로 이동하여 내려놓고 홈 복귀")
        if drop_slot == "DROP2":
            moved = self.move_robot1_drop2_pose()
        elif drop_slot == "DROP3":
            moved = self.move_robot1_drop3_pose()
        elif drop_slot == "DROP4":
            moved = self.move_robot1_drop4_pose()
        elif drop_slot == "DROP5":
            moved = self.move_robot1_drop5_pose()
        else:
            moved = self.move_robot1_drop_pose()

        if not moved:
            self.get_logger().error(f"robot1: {drop_slot} 자세 이동 실패")
            return False
        self.sleep()
        
        self.set_gripper(self.cli_g1, False)
        self.sleep()
        
        self.call(self.cli_h1, Trigger.Request())
        self.sleep()
        return True

    def robot2_drop_bottom_and_home(self, bottom_label, drop_slot="DROP"):
        self.get_logger().info(f"6) robot2: 하단({bottom_label}) 블럭 {drop_slot} 조인트로 이동하여 내려놓고 홈 복귀")
        if drop_slot == "DROP2":
            moved = self.move_robot2_drop2_pose()
        else:
            moved = self.move_robot2_drop_pose()

        if not moved:
            self.get_logger().error(f"robot2: {drop_slot} 자세 이동 실패")
            return False
        self.sleep()
        
        self.set_gripper(self.cli_g2, False)
        self.sleep()

        self.call(self.cli_h2, Trigger.Request())
        self.sleep()
        return True

    def run_two_layer_once(self, name, layers):
        top_layer = layers[0]
        bottom_layer = layers[1]
        top_target = top_layer[1]
        top_label = top_layer[2]
        yaw_offset = top_layer[3] if len(top_layer) > 3 else 0.0
        bottom_label = bottom_layer[2]

        self.get_logger().info(f"{name} 협조 분해 시작: 2층 {top_label} / 1층 {bottom_label}")
        self.move_both_home_pose()
        self.set_gripper(self.cli_g1, False)
        self.set_gripper(self.cli_g2, False)

        if not self.robot1_top_pick(top_target, top_label, expected_layer=2, yaw_offset=yaw_offset): return False
        if not self.robot2_side_hold(bottom_label): return False
        if not self.robot1_pull_up(top_label): return False
        if not self.robot2_return_home_holding(bottom_label): return False
        if not self.robot1_drop_top_and_home(top_label): return False
        if not self.robot2_drop_bottom_and_home(bottom_label): return False

        self.get_logger().info(f"🎉 {name} 협조 분해 완벽 종료")
        self.move_both_end_pose()
        return True

    def run_battery_once(self):
        return self.run_two_layer_once("배터리", [
            (2, "2x2_yellow", "2x2 노랑", 0.0),
            (1, "2x2_blue", "2x2 파랑", 0.0),
        ])

    def run_magnet_once(self):
        return self.run_two_layer_once("자석", [
            (2, "2x2_blue", "2x2 파랑", 0.0),
            (1, "2x2_red", "2x2 빨강", 0.0),
        ])

    def run_estop_once(self):
        return self.run_two_layer_once("E-stop", [
            (2, "2x2_red", "2x2 빨강", 90.0),
            (1, "2x4_yellow", "2x4 노랑", 0.0),
        ])

    def robot1_pick_layer_and_drop(
        self,
        target,
        label,
        expected_layer,
        drop_slot,
        bottom_label=None,
        yaw_offset=0.0,
        robot2_drop_slot=None,
        z_extra_mm=0.0,
    ):
        self.get_logger().info(f"{expected_layer}층 분해 시작: {label}")
        if not self.robot1_top_pick(
            target, label, expected_layer=expected_layer,
            yaw_offset=yaw_offset, z_extra_mm=z_extra_mm
        ):
            self.get_logger().error(f"{expected_layer}층 {label} 분해 실패")
            return False

        if bottom_label is not None:
            robot2_drop_slot = robot2_drop_slot or ("DROP2" if expected_layer in (2, 3) else "DROP")
            if not self.robot2_side_hold(bottom_label):
                return False
            if not self.robot1_pull_up(label):
                self.robot2_drop_bottom_and_home(bottom_label, drop_slot=robot2_drop_slot)
                return False

            if not self.robot2_return_home_holding(bottom_label):
                return False

            if not self.robot1_drop_top_and_home(label, drop_slot=drop_slot):
                self.robot2_drop_bottom_and_home(bottom_label, drop_slot=robot2_drop_slot)
                return False

            if not self.robot2_drop_bottom_and_home(bottom_label, drop_slot=robot2_drop_slot):
                return False
            return True

        return self.robot1_drop_top_and_home(label, drop_slot=drop_slot)

    def robot1_pick_layer_drop_top_keep_robot2(
        self,
        target,
        label,
        expected_layer,
        drop_slot,
        bottom_label,
        yaw_offset=0.0,
        z_extra_mm=0.0,
    ):
        self.get_logger().info(f"{expected_layer}층 분해 시작: {label} / robot2는 {bottom_label} 잡은 상태 유지")
        if not self.robot1_top_pick(
            target, label, expected_layer=expected_layer,
            yaw_offset=yaw_offset, z_extra_mm=z_extra_mm
        ):
            self.get_logger().error(f"{expected_layer}층 {label} 분해 실패")
            return False

        if not self.robot2_side_hold(bottom_label):
            return False
        if not self.robot1_pull_up(label):
            return False
        if not self.robot2_return_home_holding(bottom_label):
            return False
        if not self.robot1_drop_top_and_home(label, drop_slot=drop_slot):
            return False
        return True

    def robot1_pick_layer_drop_top_release_robot2(
        self,
        target,
        label,
        expected_layer,
        drop_slot,
        bottom_label,
        yaw_offset=0.0,
        z_extra_mm=0.0,
    ):
        self.get_logger().info(f"{expected_layer}층 분해 시작: {label} / robot2는 하단 고정 후 제자리 해제")
        if not self.robot1_top_pick(
            target, label, expected_layer=expected_layer,
            yaw_offset=yaw_offset, z_extra_mm=z_extra_mm
        ):
            self.get_logger().error(f"{expected_layer}층 {label} 분해 실패")
            return False

        if not self.robot2_side_hold(bottom_label):
            return False
        if not self.robot1_pull_up(label):
            self.robot2_release_and_home(bottom_label)
            return False
        if not self.robot1_drop_top_and_home(label, drop_slot=drop_slot):
            self.robot2_release_and_home(bottom_label)
            return False
        if not self.robot2_release_and_home(bottom_label):
            return False
        return True

    def robot1_pick_layer_drop_top_then_robot2_drop(
        self,
        target,
        label,
        expected_layer,
        drop_slot,
        bottom_label,
        robot2_drop_slot,
        yaw_offset=0.0,
        z_extra_mm=0.0,
    ):
        self.get_logger().info(
            f"{expected_layer}층 분해 시작: {label} / "
            f"robot2 HOME 복귀 후 robot1 {drop_slot}, robot2 {robot2_drop_slot}"
        )
        if not self.robot1_top_pick(
            target, label, expected_layer=expected_layer,
            yaw_offset=yaw_offset, z_extra_mm=z_extra_mm
        ):
            self.get_logger().error(f"{expected_layer}층 {label} 분해 실패")
            return False

        if not self.robot2_side_hold(bottom_label):
            return False
        if not self.robot1_pull_up(label):
            self.robot2_drop_bottom_and_home(bottom_label, drop_slot=robot2_drop_slot)
            return False
        if not self.robot2_return_home_holding(bottom_label):
            return False
        if not self.robot1_drop_top_and_home(label, drop_slot=drop_slot):
            self.robot2_drop_bottom_and_home(bottom_label, drop_slot=robot2_drop_slot)
            return False
        if not self.robot2_drop_bottom_and_home(bottom_label, drop_slot=robot2_drop_slot):
            return False
        return True

    def run_three_layer_once(self, name, layers):
        self.get_logger().info(f"{name} 3층 분해 시작")
        self.move_both_home_pose()
        self.set_gripper(self.cli_g1, False)
        self.set_gripper(self.cli_g2, False)

        drop_slots = {3: "DROP", 2: "DROP2", 1: "DROP3"}

        for index, layer_info in enumerate(layers):
            layer = layer_info[0]
            target = layer_info[1]
            label = layer_info[2]
            yaw_offset = layer_info[3] if len(layer_info) > 3 else 0.0
            drop_slot = layer_info[4] if len(layer_info) > 4 else drop_slots.get(layer, "DROP")
            
            bottom_label = layers[index + 1][2] if index + 1 < len(layers) else None
            
            if not self.robot1_pick_layer_and_drop(
                target, label, layer, drop_slot,
                bottom_label=bottom_label, yaw_offset=yaw_offset
            ):
                return False

        self.get_logger().info(f"🎉 {name} 3층 분해 완료")
        self.move_both_end_pose()
        return True

    def run_four_layer_once(self, name, layers):
        self.get_logger().info(f"{name} 4층 분해 시작")
        self.move_both_home_pose()
        self.set_gripper(self.cli_g1, False)
        self.set_gripper(self.cli_g2, False)

        drop_slots = {4: "DROP", 3: "DROP2", 2: "DROP3", 1: "DROP4"}

        for index, layer_info in enumerate(layers):
            layer = layer_info[0]
            target = layer_info[1]
            label = layer_info[2]
            yaw_offset = layer_info[3] if len(layer_info) > 3 else 0.0
            drop_slot = layer_info[4] if len(layer_info) > 4 else drop_slots.get(layer, "DROP")
            
            bottom_label = layers[index + 1][2] if index + 1 < len(layers) else None
            
            if not self.robot1_pick_layer_and_drop(
                target, label, layer, drop_slot,
                bottom_label=bottom_label, yaw_offset=yaw_offset
            ):
                return False

        self.get_logger().info(f"🎉 {name} 4층 분해 완료")
        self.move_both_end_pose()
        return True

    def run_carrot_once(self):
        return self.run_three_layer_once("당근", [
            (3, "2x2_green", "2x2 초록"), (2, "2x2_yellow", "2x2 노랑"), (1, "2x2_yellow", "2x2 노랑")
        ])

    def run_small_tree_once(self):
        return self.run_three_layer_once("작은 나무", [
            (3, "2x2_green", "2x2 초록"), (2, "4x2_green", "4x2 초록"), (1, "2x2_yellow", "2x2 노랑")
        ])

    def run_traffic_light_once(self):
        return self.run_three_layer_once("신호등", [
            (3, "2x2_red", "2x2 빨강"), (2, "2x2_yellow", "2x2 노랑"), (1, "2x2_green", "2x2 초록")
        ])

    def run_hammer_once(self):
        return self.run_three_layer_once("망치", [
            (3, "4x2_blue", "4x2 파랑", 0.0),
            (2, "2x2_red", "2x2 빨강", 0.0),
            (1, "2x2_red", "2x2 빨강", 0.0)
        ])

    def run_big_carrot_once(self):
        return self.run_four_layer_once("큰 당근", [
            (4, "2x2_green", "2x2 초록", 0.0), 
            (3, "4x2_yellow", "4x2 노랑", 0.0),
            (2, "2x2_yellow", "2x2 노랑", 0.0),
            (1, "2x2_yellow", "2x2 노랑", 0.0),
        ])

    # ==========================================
    # 💥 신규 추가: 버거, 아이스크림, 큰 나무 해체 시퀀스
    # ==========================================
    def run_burger_once(self):
        return self.run_four_layer_once("버거", [
            (4, "4x2_yellow", "4x2 노랑", 0.0),
            (3, "2x2_red", "2x2 빨강", 0.0),
            (2, "4x2_red", "4x2 빨강", 0.0),
            (1, "4x2_yellow", "4x2 노랑", 0.0),
        ])

    def run_ice_cream_once(self):
        return self.run_four_layer_once("아이스크림", [
            (4, "2x2_green", "2x2 초록", 0.0, "DROP"),
            (3, "2x2_blue", "2x2 파랑", 0.0, "DROP2"),
            (3, "2x2_red", "2x2 빨강", 0.0, "DROP3"),
            (2, "4x2_yellow", "4x2 노랑", 0.0, "DROP4"),
            (1, "2x2_yellow", "2x2 노랑", 0.0, "DROP5"),
        ])

    def run_big_tree_once(self):
        self.get_logger().info("큰 나무 특수 분해 시작: 4층 -> 1층 노랑 이송 -> 남은 3개")
        self.move_both_home_pose()
        self.set_gripper(self.cli_g1, False)
        self.set_gripper(self.cli_g2, False)

        if not self.robot1_pick_layer_and_drop(
            "2x2_green", "4층 2x2 초록", 4, "DROP",
            bottom_label="4x2 초록", yaw_offset=0.0
        ):
            return False

        if not self.robot1_pick_layer_place_center_robot2_drop(
            "4x2_green", "1층 분리용 2층 초록", 2,
            bottom_label="2x2 노랑", yaw_offset=0.0,
            robot2_drop_slot="DROP2", z_extra_mm=16.0
        ):
            return False

        if not self.robot1_pick_layer_and_drop(
            "2x2_yellow", "1층 2x2 노랑", 1, "DROP2",
            yaw_offset=0.0
        ):
            return False

        if not self.robot1_pick_layer_drop_top_then_robot2_drop(
            "4x2_green", "남은 3층 4x2 초록", 3, "DROP3",
            bottom_label="2층 초록", robot2_drop_slot="DROP2", yaw_offset=0.0
        ):
            return False

        if not self.robot1_pick_layer_and_drop(
            "4x2_green", "남은 2층 4x2 초록", 2, "DROP4",
            yaw_offset=0.0
        ):
            return False

        if not self.robot1_pick_layer_and_drop(
            "2x2_green", "남은 2층 2x2 초록", 2, "DROP5",
            yaw_offset=0.0
        ):
            return False

        self.get_logger().info("🎉 큰 나무 특수 분해 완료")
        self.move_both_end_pose()
        return True

    def run(self):
        self.move_both_home_pose()

        print("\n=== Master Node Dis10 Keyboard Select ===")
        print("1: 배터리  (2층 2x2 노랑 / 1층 2x2 파랑)")
        print("2: 자석    (2층 2x2 파랑 / 1층 2x2 빨강)")
        print("3: E-stop  (2층 2x2 빨강 / 1층 2x4 노랑)")
        print("4: 당근    (3층 2x2 초록 / 2층 2x2 노랑 / 1층 2x2 노랑)")
        print("5: 작은 나무 (3층 2x2 초록 / 2층 4x2 초록 / 1층 2x2 노랑)")
        print("6: 신호등  (3층 2x2 빨강 / 2층 2x2 노랑 / 1층 2x2 초록)")
        print("7: 망치    (3층 4x2 파랑 / 2층 2x2 빨강 / 1층 2x2 빨강)")
        print("8: 큰 당근 (4층 2x2 초록 / 3층 4x2 노랑 / 2층 2x2 노랑 / 1층 2x2 노랑)")
        print("9: 버거    (4층 4x2 노랑 / 3층 2x2 빨강 / 2층 4x2 빨강 / 1층 4x2 노랑)")
        print("10: 아이스크림 (4층 2x2 초록 / 3층 2x2 파랑,빨강 / 2층 4x2 노랑 / 1층 2x2 노랑)")
        print("11: 큰 나무 (4층 2x2 초록 / 3층 4x2 초록 / 2층 4x2,2x2 초록 / 1층 2x2 노랑)")
        print("q: 종료")

        actions = {
            "1": self.run_battery_once, "battery": self.run_battery_once, "배터리": self.run_battery_once,
            "2": self.run_magnet_once, "magnet": self.run_magnet_once, "자석": self.run_magnet_once,
            "3": self.run_estop_once, "estop": self.run_estop_once, "e-stop": self.run_estop_once, "비상정지": self.run_estop_once,
            "4": self.run_carrot_once, "carrot": self.run_carrot_once, "당근": self.run_carrot_once,
            "5": self.run_small_tree_once, "tree": self.run_small_tree_once, "small_tree": self.run_small_tree_once, "작은나무": self.run_small_tree_once,
            "6": self.run_traffic_light_once, "traffic": self.run_traffic_light_once, "traffic_light": self.run_traffic_light_once, "신호등": self.run_traffic_light_once,
            "7": self.run_hammer_once, "hammer": self.run_hammer_once, "망치": self.run_hammer_once,
            "8": self.run_big_carrot_once, "bigcarrot": self.run_big_carrot_once, "큰당근": self.run_big_carrot_once, "큰 당근": self.run_big_carrot_once,
            "9": self.run_burger_once, "burger": self.run_burger_once, "버거": self.run_burger_once,
            "10": self.run_ice_cream_once, "icecream": self.run_ice_cream_once, "아이스크림": self.run_ice_cream_once,
            "11": self.run_big_tree_once, "bigtree": self.run_big_tree_once, "큰나무": self.run_big_tree_once, "큰 나무": self.run_big_tree_once,
        }

        while rclpy.ok():
            user_input = input("\n선택하세요 [1~11/q]: ").strip().replace(" ", "").lower()
            if user_input in ("q", "quit", "exit", "종료"):
                self.get_logger().info("키보드 선택 모션을 종료합니다.")
                break

            action = actions.get(user_input)
            if action is None:
                print("잘못된 입력입니다. 1~11 또는 q 중에서 선택하세요.")
                continue

            action()


def main():
    rclpy.init()
    node = BatteryDualDisassembly()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()