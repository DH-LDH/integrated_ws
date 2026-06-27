# import rclpy
# from rclpy.node import Node
# from srvs_pkg.srv import GetTargetPose
# from std_srvs.srv import Trigger
# import rbpodo as rb
# import numpy as np
# ###최신

# ROBOT_CONFIGS = {
#     "robot1": {
#         "ip": "10.0.2.7",
#         "cam_x_off": -51.383,
#         "cam_y_off": 32.485,
#         "home_joint": [-90.0, 6.67, 35.34, 0.0, 138.0, 0.0],
#         "end_joint": [-90.0, -65.0, 110.0, 0.0, 140.0, 0.0],
#         "center_joint": [-108.2, -10.14, 104.67, 0.0, 85.48, -18.2],
#         "separation_joint": [-88.26, 3.5, 48.84, 0.0, 126.44, -90.0],
#         "drop_joint": [-90.0, 11.04, 83.29, 0.0, 85.67, 0.0],
#         "drop_joint2": [-102.53, 13.33, 80.6, 0.0, 86.07,-12.53],
#         "drop_joint3": [-113.13, 20.29, 71.96, 0.0, 87.75,-20.84],
#         "drop_joint4": [-79.61, 11.64, 82.56, 0.0, 85.77,10.39],
#         "drop_joint5": [-68.76, 15.86, 77.51, 0.0, 86.61,21.24],
#     },
#     "robot2": {
#         "ip": "10.0.2.8",
#         "cam_x_off": -53.0,
#         "cam_y_off": 51.0, 
#         "home_joint": [-90.0, -94.0, 147.7, 0.0, 35.6, 0.0],
#         "end_joint": [-90.0, -94.0, 147.7, 0.0, -50.0, 0.0],
#         # 👇 추가된 중간 경유지 (Waypoint)
#         "separation_waypoint": [-90.0, 0.0, 120.0, 0.0, -30.0, 0.0], 
        
#         "separation_joint": [-90.0, -9.35, 111.55, 0.03, -12.96, -0.02],
#         "drop_joint": [-90.0, 7.89, 131.43, 0.0, -49.32, 0.0],
#         "drop_joint2": [-90.0, 9.27, 131.51, 0.0, -48.47, 0.0],  #2,3층 시퀀스만 해당
#     },
# }


# class DualRobotNode(Node):
#     def __init__(self):
#         super().__init__("dual_robot_node")

#         self.robots = {}
#         for robot_name, cfg in ROBOT_CONFIGS.items():
#             robot = rb.Cobot(cfg["ip"])
#             rc = rb.ResponseCollector()
#             robot.set_operation_mode(rc, rb.OperationMode.Real)

#             self.robots[robot_name] = {
#                 "robot": robot,
#                 "rc": rc,
#                 "ip": cfg["ip"],
#                 "cam_x_off": cfg["cam_x_off"],
#                 "cam_y_off": cfg["cam_y_off"],
#                 "home_joint": np.array(cfg["home_joint"], dtype=float),
#                 "end_joint": np.array(cfg.get("end_joint", cfg["home_joint"]), dtype=float),
                
#                 "separation_waypoint": np.array(cfg["separation_waypoint"], dtype=float) if "separation_waypoint" in cfg else None,
                
#                 "separation_joint": np.array(cfg.get("separation_joint", cfg["home_joint"]), dtype=float),
#                 "center_joint": np.array(cfg.get("center_joint", cfg["home_joint"]), dtype=float),
#                 "drop_joint": np.array(cfg.get("drop_joint", cfg["home_joint"]), dtype=float),
#                 "drop_joint2": np.array(cfg.get("drop_joint2", cfg.get("drop_joint", cfg["home_joint"])), dtype=float),
#                 "drop_joint3": np.array(cfg.get("drop_joint3", cfg.get("drop_joint", cfg["home_joint"])), dtype=float),
#                 "drop_joint4": np.array(cfg.get("drop_joint4", cfg.get("drop_joint", cfg["home_joint"])), dtype=float),
#                 "drop_joint5": np.array(cfg.get("drop_joint5", cfg.get("drop_joint", cfg["home_joint"])), dtype=float),
#                 "last_target": None,
#             }

#             self.create_service(
#                 Trigger,
#                 f"/{robot_name}/robot_home",
#                 lambda req, res, name=robot_name: self.home_cb(name, req, res),
#             )
#             self.create_service(
#                 GetTargetPose,
#                 f"/{robot_name}/robot_move_step",
#                 lambda req, res, name=robot_name: self.move_step_cb(name, req, res),
#             )

#             self.get_logger().info(
#                 f"{robot_name} ready: ip={cfg['ip']}, "
#                 f"services=/{robot_name}/robot_home, /{robot_name}/robot_move_step"
#             )

#         self.L_VEL = 500
#         self.L_ACC = 800
#         self.get_logger().info("Dual Robot Node Ready")

#     def wait_move(self, robot_name, name="MOVE"):
#         handle = self.robots[robot_name]
#         robot = handle["robot"]
#         rc = handle["rc"]

#         started_result = robot.wait_for_move_started(rc, 1.0)
#         started = started_result.is_success() if hasattr(started_result, "is_success") else False

#         if not started:
#             self.get_logger().warn(f"{robot_name} {name} START SKIPPED")
#             return True

#         robot.wait_for_move_finished(rc)
#         return True

#     def home_cb(self, robot_name, req, res):
#         try:
#             handle = self.robots[robot_name]
#             if (
#                 robot_name == "robot2"
#                 and handle.get("last_target") == "SEPARATION"
#                 and handle.get("separation_waypoint") is not None
#             ):
#                 handle["robot"].move_j(handle["rc"], handle["separation_waypoint"], 255, 255)
#                 self.wait_move(robot_name, "SEPARATION_WAYPOINT_TO_HOME")

#             handle["robot"].move_j(handle["rc"], handle["home_joint"], 255, 255)
#             self.wait_move(robot_name, "HOME")
#             handle["last_target"] = "HOME"
#             res.success = True
#         except Exception as e:
#             self.get_logger().error(f"{robot_name} HOME Error: {e}")
#             res.success = False
#         return res

#     def move_step_cb(self, robot_name, req, res):
#         try:
#             handle = self.robots[robot_name]
#             robot = handle["robot"]
#             rc = handle["rc"]

#             if req.target_size == "YAW":
#                 if abs(req.yaw) < 0.1:
#                     self.get_logger().info(f"{robot_name} YAW skipped: {req.yaw:.2f}")
#                     res.success = True
#                     return res

#                 pose = np.array([0, 0, 0, 0, 0, req.yaw], dtype=float)
#                 robot.move_l_rel(rc, pose, self.L_VEL, self.L_ACC, rb.ReferenceFrame.Tool)
#                 self.wait_move(robot_name, "YAW")

#             elif req.target_size == "XY":
#                 dx = -(req.x * 1000.0) + handle["cam_y_off"]
#                 dy = (req.y * 1000.0) + handle["cam_x_off"]
#                 pose = np.array([dy, dx, 0, 0, 0, 0], dtype=float)
#                 robot.move_l_rel(rc, pose, self.L_VEL, self.L_ACC, rb.ReferenceFrame.Tool)
#                 self.wait_move(robot_name, "XY")

#             elif req.target_size == "Z":
#                 pose = np.array([0, 0, req.z, 0, 0, 0], dtype=float)
#                 robot.move_l_rel(rc, pose, self.L_VEL, self.L_ACC, rb.ReferenceFrame.Tool)
#                 self.wait_move(robot_name, f"Z_MOVE({req.z:.1f})")

#             elif req.target_size == "APPROACH":
#                 # 조립 마스터(master_node)용: YAW + XY + Z 한 모션 접근
#                 # move_l_rel(Tool)의 병진은 이동 시작 프레임 기준으로 적용되고,
#                 # rz(yaw)는 tool Z축 방향을 바꾸지 않으므로 dx/dy/z/yaw를 한 번에 합쳐도 된다.
#                 dx = -(req.x * 1000.0) + handle["cam_y_off"]
#                 dy = (req.y * 1000.0) + handle["cam_x_off"]
#                 pose = np.array([dy, dx, req.z, 0, 0, req.yaw], dtype=float)
#                 robot.move_l_rel(rc, pose, self.L_VEL, self.L_ACC, rb.ReferenceFrame.Tool)
#                 self.wait_move(robot_name, f"APPROACH(yaw={req.yaw:.1f})")

#             elif req.target_size == "SEPARATION":
#                 if handle.get("separation_waypoint") is not None:
#                     robot.move_j(rc, handle["separation_waypoint"], 255, 255)
#                     self.wait_move(robot_name, "SEPARATION_WAYPOINT")
                
#                 robot.move_j(rc, handle["separation_joint"], 255, 255)
#                 self.wait_move(robot_name, "SEPARATION")

#             elif req.target_size == "CENTER":
#                 robot.move_j(rc, handle["center_joint"], 255, 255)
#                 self.wait_move(robot_name, "CENTER")

#             elif req.target_size == "DROP":
#                 robot.move_j(rc, handle["drop_joint"], 255, 255)
#                 self.wait_move(robot_name, "DROP")

#             elif req.target_size == "DROP2":
#                 robot.move_j(rc, handle["drop_joint2"], 255, 255)
#                 self.wait_move(robot_name, "DROP2")

#             elif req.target_size == "DROP3":
#                 robot.move_j(rc, handle["drop_joint3"], 255, 255)
#                 self.wait_move(robot_name, "DROP3")

#             elif req.target_size == "DROP4":
#                 robot.move_j(rc, handle["drop_joint4"], 255, 255)
#                 self.wait_move(robot_name, "DROP4")

#             elif req.target_size == "DROP5":
#                 robot.move_j(rc, handle["drop_joint5"], 255, 255)
#                 self.wait_move(robot_name, "DROP5")

#             elif req.target_size == "END":
#                 robot.move_j(rc, handle["end_joint"], 255, 255)
#                 self.wait_move(robot_name, "END")

#             else:
#                 self.get_logger().error(f"{robot_name} unknown target_size: {req.target_size}")
#                 res.success = False
#                 return res

#             res.success = True
#             handle["last_target"] = req.target_size
#         except Exception as e:
#             self.get_logger().error(f"{robot_name} Move Error: {e}")
#             res.success = False

#         return res


# def main():
#     rclpy.init()
#     node = DualRobotNode()
#     try:
#         rclpy.spin(node)
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == "__main__":
#     main()

import rclpy
from rclpy.node import Node
from srvs_pkg.srv import GetTargetPose
from std_srvs.srv import Trigger
import rbpodo as rb
import numpy as np
###최신

ROBOT_CONFIGS = {
    "robot1": {
        "ip": "10.0.2.7",
        "cam_x_off": -52.5, # 바깥으로 가고싶으면 음수, 몸쪽으로 오고 싶으면 양수
        "cam_y_off": -34.485, #-29.485, #32.485
        "home_joint": [-90.0, 6.67, 35.34, 0.0, 138.0, 0.0],
        "end_joint": [-90.0, -65.0, 110.0, 0.0, 140.0, 0.0],
        "center_joint": [-108.2, -10.14, 104.67, 0.0, 85.48, -18.2],
        "separation_joint": [-91.02, 21.07, 26.68, 0.11, 131.03, -91.23],
        "drop_joint": [-90.0, 19.26, 74.23, 0.02, 86.51, 0.0],
        "drop_joint2": [-100.03, 20.95, 72.05, 0.02, 87.00, -10.03],
        "drop_joint3": [-108.77, 25.01, 66.67, 0.02, 88.32, -18.76],
        "drop_joint4": [-79.86, 20.43, 72.73, 0.02, 86.85, 10.14],
        "drop_joint5": [-70.49, 24.35, 67.56, 0.02, 88.09, 19.52],
        "assembly_drop_joint_s": [-157.95, 0.71, 112.47, 61.43, 59.02, -188.06],
        "assembly_drop_joint_m": [-157.95, -0.30, 111.25, 60.29, 60.10, -185.80],
        "assembly_drop_joint_l": [-157.95, -0.71, 110.69, 59.81, 60.58, -184.84],
        "drop_after_home_joint": [-90.70, -103.8, 144.2, 2.59, 51.69, -4.05],
        "drop_after_grip_joint": [-91.17, -56.46, 142.27, 1.13, 4.65, -4.05],
        "drop_after_drop_joint": [-100.02, -16.63, 141.86, 6.93, -15.10, -10.03],

    },
    "robot2": {
        "ip": "10.0.2.8",
        "cam_x_off": -53.0,
        "cam_y_off": 51.0, 
        "home_joint": [-90.0, -94.0, 147.7, 0.0, 35.6, 0.0],
        "end_joint": [-90.0, -94.0, 147.7, 0.0, -50.0, 0.0],
        "assembly_joint": [-90, -73.29, 120.58, 0, 0, 180],
        # 👇 추가된 중간 경유지 (Waypoint)
        "separation_waypoint": [-90.0, 0.0, 120.0, 0.0, -30.0, 0.0], 
        "separation_joint": [-90.0, -9.73, 112.34, 0.0, -14.06, 0.0],
        "drop_joint": [-95.09, 4.38, 135.12, -2.38, -41.78, 0.0],
        "drop_joint2": [-95.0, 8.48, 132.29, -2.78, -47.6, 2.73], # 2,3층 시퀀스만 해당
        "drop_joint3": [-95.0, 4.25, 131.84, -3.01, -42.93, 3.06] # 4층 시퀀스  
    }
}


class DualRobotNode(Node):
    def __init__(self):
        super().__init__("dual_robot_node")

        self.robots = {}
        for robot_name, cfg in ROBOT_CONFIGS.items():
            robot = rb.Cobot(cfg["ip"])
            rc = rb.ResponseCollector()
            robot.set_operation_mode(rc, rb.OperationMode.Real)

            self.robots[robot_name] = {
                "robot": robot,
                "rc": rc,
                "ip": cfg["ip"],
                "cam_x_off": cfg["cam_x_off"],
                "cam_y_off": cfg["cam_y_off"],
                "home_joint": np.array(cfg["home_joint"], dtype=float),
                "end_joint": np.array(cfg.get("end_joint", cfg["home_joint"]), dtype=float),
                "assembly_joint": np.array(cfg.get("assembly_joint", cfg["home_joint"]), dtype=float),
                
                "separation_waypoint": np.array(cfg["separation_waypoint"], dtype=float) if "separation_waypoint" in cfg else None,
                
                "separation_joint": np.array(cfg.get("separation_joint", cfg["home_joint"]), dtype=float),
                "center_joint": np.array(cfg.get("center_joint", cfg["home_joint"]), dtype=float),
                "drop_joint": np.array(cfg.get("drop_joint", cfg["home_joint"]), dtype=float),
                "drop_joint2": np.array(cfg.get("drop_joint2", cfg.get("drop_joint", cfg["home_joint"])), dtype=float),
                "drop_joint3": np.array(cfg.get("drop_joint3", cfg.get("drop_joint", cfg["home_joint"])), dtype=float),
                "drop_joint4": np.array(cfg.get("drop_joint4", cfg.get("drop_joint", cfg["home_joint"])), dtype=float),
                "drop_joint5": np.array(cfg.get("drop_joint5", cfg.get("drop_joint", cfg["home_joint"])), dtype=float),
                "assembly_drop_joint_s": np.array(cfg.get("assembly_drop_joint_s", cfg["home_joint"]), dtype=float),
                "assembly_drop_joint_m": np.array(cfg.get("assembly_drop_joint_m", cfg["home_joint"]), dtype=float),
                "assembly_drop_joint_l": np.array(cfg.get("assembly_drop_joint_l", cfg["home_joint"]), dtype=float),
                "drop_after_home_joint": np.array(cfg.get("drop_after_home_joint", cfg["home_joint"]), dtype=float),
                "drop_after_grip_joint": np.array(cfg.get("drop_after_grip_joint", cfg["home_joint"]), dtype=float),
                "drop_after_drop_joint": np.array(cfg.get("drop_after_drop_joint", cfg["home_joint"]), dtype=float),
                "last_target": None,
            }

            self.create_service(
                Trigger,
                f"/{robot_name}/robot_home",
                lambda req, res, name=robot_name: self.home_cb(name, req, res),
            )
            self.create_service(
                GetTargetPose,
                f"/{robot_name}/robot_move_step",
                lambda req, res, name=robot_name: self.move_step_cb(name, req, res),
            )

            self.get_logger().info(
                f"{robot_name} ready: ip={cfg['ip']}, "
                f"services=/{robot_name}/robot_home, /{robot_name}/robot_move_step"
            )

        self.L_VEL = 700
        self.L_ACC = 1500
        self.get_logger().info("Dual Robot Node Ready")

    def wait_move(self, robot_name, name="MOVE"):
        handle = self.robots[robot_name]
        robot = handle["robot"]
        rc = handle["rc"]

        started_result = robot.wait_for_move_started(rc, 1.0)
        started = started_result.is_success() if hasattr(started_result, "is_success") else False

        if not started:
            self.get_logger().warn(f"{robot_name} {name} START SKIPPED")
            return True

        robot.wait_for_move_finished(rc)
        return True

    def home_cb(self, robot_name, req, res):
        try:
            handle = self.robots[robot_name]
            if (
                robot_name == "robot2"
                and handle.get("last_target") == "SEPARATION"
                and handle.get("separation_waypoint") is not None
            ):
                handle["robot"].move_j(handle["rc"], handle["separation_waypoint"], 255, 255)
                self.wait_move(robot_name, "SEPARATION_WAYPOINT_TO_HOME")

            handle["robot"].move_j(handle["rc"], handle["home_joint"], 255, 255)
            self.wait_move(robot_name, "HOME")
            handle["last_target"] = "HOME"
            res.success = True
        except Exception as e:
            self.get_logger().error(f"{robot_name} HOME Error: {e}")
            res.success = False
        return res

    def move_step_cb(self, robot_name, req, res):
        try:
            handle = self.robots[robot_name]
            robot = handle["robot"]
            rc = handle["rc"]

            if req.target_size == "YAW":
                if abs(req.yaw) < 0.1:
                    self.get_logger().info(f"{robot_name} YAW skipped: {req.yaw:.2f}")
                    res.success = True
                    return res

                pose = np.array([0, 0, 0, 0, 0, req.yaw], dtype=float)
                robot.move_l_rel(rc, pose, self.L_VEL, self.L_ACC, rb.ReferenceFrame.Tool)
                self.wait_move(robot_name, "YAW")

            elif req.target_size == "XY":
                dx = (req.x * 1000.0)
                dy = -(req.y * 1000.0)
                pose = np.array([dy, dx, 0, 0, 0, 0], dtype=float)
                self.get_logger().info(
                    f"[{robot_name}][XY] req=(x={req.x*1000:.1f}mm, y={req.y*1000:.1f}mm) "
                    f"→ Tool(X={dy:.1f}mm, Y={dx:.1f}mm)"
                )
                robot.move_l_rel(rc, pose, self.L_VEL, self.L_ACC, rb.ReferenceFrame.Tool)
                self.wait_move(robot_name, "XY")

            elif req.target_size == "Z":
                pose = np.array([0, 0, req.z, 0, 0, 0], dtype=float)
                robot.move_l_rel(rc, pose, self.L_VEL, self.L_ACC, rb.ReferenceFrame.Tool)
                self.wait_move(robot_name, f"Z_MOVE({req.z:.1f})")

            elif req.target_size == "APPROACH":
                # 조립 마스터(master_node)용: YAW + XY + Z 한 모션 접근
                # move_l_rel(Tool)의 병진은 이동 시작 프레임 기준으로 적용되고,
                # rz(yaw)는 tool Z축 방향을 바꾸지 않으므로 dx/dy/z/yaw를 한 번에 합쳐도 된다.
                dx = (req.x * 1000.0) + handle["cam_y_off"]
                dy = -(req.y * 1000.0) + handle["cam_x_off"]
                pose = np.array([dy, dx, req.z, 0, 0, req.yaw], dtype=float)
                self.get_logger().info(
                    f"[{robot_name}][APPROACH] req=(x={req.x*1000:.1f}mm, y={req.y*1000:.1f}mm, "
                    f"z={req.z:.1f}mm, yaw={req.yaw:.1f}°) "
                    f"→ Tool(X={dy:.1f}mm, Y={dx:.1f}mm, Z={req.z:.1f}mm)"
                )
                robot.move_l_rel(rc, pose, self.L_VEL, self.L_ACC, rb.ReferenceFrame.Tool)
                self.wait_move(robot_name, f"APPROACH(yaw={req.yaw:.1f})")

            elif req.target_size == "APPROACH_DELTA":
                # 888 정밀 인식 실패 시 fallback 경로용.
                # 이미 APPROACH로 스캔 위치까지 이동(cam 오프셋 1회 적용)한 상태이고,
                # master가 (목표 - 스캔) delta를 미리 계산해 넘기므로 cam 오프셋은 상쇄됨.
                # -> APPROACH와 동일하되 cam_x_off / cam_y_off 를 다시 더하지 않는다.
                dx = (req.x * 1000.0)
                dy = -(req.y * 1000.0)
                pose = np.array([dy, dx, req.z, 0, 0, req.yaw], dtype=float)
                robot.move_l_rel(rc, pose, self.L_VEL, self.L_ACC, rb.ReferenceFrame.Tool)
                self.wait_move(robot_name, f"APPROACH_DELTA(yaw={req.yaw:.1f})")

            elif req.target_size == "HOME_X_SEARCH":
                # HOME 기준 좌우 탐색 전용.
                # Tool 프레임으로 HOME에서 바로 밀면 관절이 더 펴질 수 있어
                # Base 프레임 기준 직선 이동으로 x축 탐색만 수행한다.
                pose = np.array([req.x * 1000.0, 0, 0, 0, 0, 0], dtype=float)
                robot.move_l_rel(rc, pose, self.L_VEL, self.L_ACC, rb.ReferenceFrame.Base)
                self.wait_move(robot_name, f"HOME_X_SEARCH(x={req.x * 1000.0:.1f}mm)")

            elif req.target_size == "SEPARATION":
                if handle.get("separation_waypoint") is not None:
                    robot.move_j(rc, handle["separation_waypoint"], 255, 255)
                    self.wait_move(robot_name, "SEPARATION_WAYPOINT")
                
                robot.move_j(rc, handle["separation_joint"], 255, 255)
                self.wait_move(robot_name, "SEPARATION")

            elif req.target_size == "CENTER":
                robot.move_j(rc, handle["center_joint"], 255, 255)
                self.wait_move(robot_name, "CENTER")

            elif req.target_size == "DROP":
                robot.move_j(rc, handle["drop_joint"], 255, 255)
                self.wait_move(robot_name, "DROP")

            elif req.target_size == "DROP2":
                robot.move_j(rc, handle["drop_joint2"], 255, 255)
                self.wait_move(robot_name, "DROP2")

            elif req.target_size == "DROP3":
                robot.move_j(rc, handle["drop_joint3"], 255, 255)
                self.wait_move(robot_name, "DROP3")

            elif req.target_size == "DROP4":
                robot.move_j(rc, handle["drop_joint4"], 255, 255)
                self.wait_move(robot_name, "DROP4")

            elif req.target_size == "DROP5":
                robot.move_j(rc, handle["drop_joint5"], 255, 255)
                self.wait_move(robot_name, "DROP5")

            elif req.target_size == "END":
                robot.move_j(rc, handle["end_joint"], 255, 255)
                self.wait_move(robot_name, "END")

            elif req.target_size == "ASSEMBLY_JOINT":
                robot.move_j(rc, handle["assembly_joint"], 255, 255)
                self.wait_move(robot_name, "ASSEMBLY_JOINT")

            elif req.target_size == "ASSEMBLY_DROP_S":
                robot.move_j(rc, handle["assembly_drop_joint_s"], 255, 255)
                self.wait_move(robot_name, "ASSEMBLY_DROP_S")

            elif req.target_size == "ASSEMBLY_DROP_M":
                robot.move_j(rc, handle["assembly_drop_joint_m"], 255, 255)
                self.wait_move(robot_name, "ASSEMBLY_DROP_M")

            elif req.target_size == "ASSEMBLY_DROP_L":
                robot.move_j(rc, handle["assembly_drop_joint_l"], 255, 255)
                self.wait_move(robot_name, "ASSEMBLY_DROP_L")

            elif req.target_size == "DROP_AFTER_HOME":
                robot.move_j(rc, handle["drop_after_home_joint"], 255, 255)
                self.wait_move(robot_name, "DROP_AFTER_HOME")

            elif req.target_size == "DROP_AFTER_GRIP":
                robot.move_j(rc, handle["drop_after_grip_joint"], 255, 255)
                self.wait_move(robot_name, "DROP_AFTER_GRIP")

            elif req.target_size == "DROP_AFTER_DROP":
                robot.move_j(rc, handle["drop_after_drop_joint"], 255, 255)
                self.wait_move(robot_name, "DROP_AFTER_DROP")

            elif req.target_size == "Z_BASE":
                pose = np.array([0, 0, req.z, 0, 0, 0], dtype=float)
                robot.move_l_rel(rc, pose, self.L_VEL, self.L_ACC, rb.ReferenceFrame.Base)
                self.wait_move(robot_name, f"Z_BASE({req.z:.1f})")

            else:
                self.get_logger().error(f"{robot_name} unknown target_size: {req.target_size}")
                res.success = False
                return res

            res.success = True
            handle["last_target"] = req.target_size
        except Exception as e:
            self.get_logger().error(f"{robot_name} Move Error: {e}")
            res.success = False

        return res


def main():
    rclpy.init()
    node = DualRobotNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
