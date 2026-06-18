# import rclpy
# from rclpy.node import Node
# from std_srvs.srv import SetBool
# import serial
# import time

# class GripperNode(Node):
#     def __init__(self):
#         super().__init__('gripper_node')
#         self.srv = self.create_service(SetBool, 'control_gripper', self.control_cb)
        
#         try:
#             # 시리얼 포트 연결 (노트북 설정에 맞춰 ACM0 또는 ACM1 확인 필요)
#             self.ser = serial.Serial("/dev/ttyACM0", 115200, timeout=1)
            
#             # 아두이노/그리퍼 컨트롤러 리셋 대기
#             time.sleep(2.0)
#             self.get_logger().info("✅ Gripper Serial Connected")

#             # [수정 사항] 노드 시작 시 자동으로 그리퍼를 엽니다.
#             self.get_logger().info("➡️ Initializing Gripper: Sending 'open'...")
#             self.ser.write(b"open\n")
            
#         except Exception as e:
#             self.get_logger().error(f"❌ Serial Error: {e}")

#     def control_cb(self, request, response):
#         """
#         Service Callback
#         request.data 가 True면 grip, False면 open 명령을 보냅니다.
#         """
#         try:
#             if request.data:  # True -> Grip
#                 self.ser.write(b"grip\n")
#                 self.get_logger().info("📌 Sent: grip")
#                 response.message = "Grip Command Sent"
#             else:             # False -> Open
#                 self.ser.write(b"open\n")
#                 self.get_logger().info("📌 Sent: open")
#                 response.message = "Open Command Sent"
            
#             response.success = True
#         except Exception as e:
#             self.get_logger().error(f"❌ Service Error: {e}")
#             response.success = False
#             response.message = str(e)
            
#         return response

# def main(args=None):
#     rclpy.init(args=args)
#     node = GripperNode()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         node.get_logger().info('Keyboard Interrupt (SIGINT)')
#     finally:
#         if hasattr(node, 'ser') and node.ser.is_open:
#             node.ser.close()
#             node.get_logger().info("✅ Serial Closed")
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == "__main__":
#     main()

# import rclpy
# from rclpy.node import Node
# from std_srvs.srv import SetBool
# import serial
# import time

# class GripperNode(Node):
#     def __init__(self):
#         super().__init__('gripper_node')
#         self.srv = self.create_service(SetBool, 'control_gripper', self.control_cb)
        
#         try:
#             # 시리얼 포트 연결 (노트북 설정에 맞춰 ACM0 또는 ACM1 확인 필요)
#             self.ser = serial.Serial("/dev/ttyACM0", 115200, timeout=1)
            
#             # 아두이노/그리퍼 컨트롤러 리셋 대기
#             time.sleep(2.0)
#             self.get_logger().info("✅ Gripper Serial Connected")

#             # [수정 사항] 노드 시작 시 자동으로 그리퍼를 엽니다.
#             self.get_logger().info("➡️ Initializing Gripper: Sending 'open'...")
#             self.ser.write(b"open\n")
            
#         except Exception as e:
#             self.get_logger().error(f"❌ Serial Error: {e}")

#     def control_cb(self, request, response):
#         """
#         Service Callback
#         request.data 가 True면 grip, False면 open 명령을 보냅니다.
#         """
#         try:
#             if request.data:  # True -> Grip
#                 self.ser.write(b"grip\n")
#                 self.get_logger().info("📌 Sent: grip")
#                 response.message = "Grip Command Sent"
#             else:             # False -> Open
#                 self.ser.write(b"open\n")
#                 self.get_logger().info("📌 Sent: open")
#                 response.message = "Open Command Sent"
            
#             response.success = True
#         except Exception as e:
#             self.get_logger().error(f"❌ Service Error: {e}")
#             response.success = False
#             response.message = str(e)
            
#         return response

# def main(args=None):
#     rclpy.init(args=args)
#     node = GripperNode()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         node.get_logger().info('Keyboard Interrupt (SIGINT)')
#     finally:
#         if hasattr(node, 'ser') and node.ser.is_open:
#             node.ser.close()
#             node.get_logger().info("✅ Serial Closed")
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == "__main__":
#     main()


import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
import serial
import time

import threading  # [새로 추가된 모듈] 백그라운드 키보드 입력을 위해 사용


class GripperNode(Node):
    def __init__(self):
        super().__init__('gripper_node')
        self.srv = self.create_service(SetBool, 'control_gripper', self.control_cb)
        
        try:
            # 시리얼 포트 연결 (노트북 설정에 맞춰 ACM0 또는 ACM1 확인 필요)
            self.ser = serial.Serial("/dev/ttyACM0", 115200, timeout=1)
            
            # 아두이노/그리퍼 컨트롤러 리셋 대기
            time.sleep(2.0)
            self.get_logger().info("✅ Gripper Serial Connected")

            # [수정 사항] 노드 시작 시 자동으로 그리퍼를 엽니다.
            self.get_logger().info("➡️ Initializing Gripper: Sending 'open'...")
            self.ser.write(b"open\n")
            
        except Exception as e:
            self.get_logger().error(f"❌ Serial Error: {e}")


        # ---- [새로 추가된 부분: 키보드 입력을 처리하는 별도 스레드 시작] ----
        self.input_thread = threading.Thread(target=self.keyboard_input_loop, daemon=True)
        self.input_thread.start()
        # ----------------------------------------------------------------------

    # ---- [새로 추가된 부분: 키보드 입력 처리 함수] ----
    def keyboard_input_loop(self):
        # 노드가 살아있는 동안 계속 반복
        while rclpy.ok():
            try:
                # 사용자가 터미널에 입력한 글자를 가져와서 소문자로 변환하고 공백 제거
                user_cmd = input("\n터미널 명령 입력 (grip / open): ").strip().lower()
                
                if user_cmd == "grip":
                    if hasattr(self, 'ser') and self.ser.is_open:
                        self.ser.write(b"grip\n")
                        self.get_logger().info("⌨️ Keyboard Command Sent: grip")
                    else:
                        print("시리얼이 연결되지 않았습니다.")
                        
                elif user_cmd == "open":
                    if hasattr(self, 'ser') and self.ser.is_open:
                        self.ser.write(b"open\n")
                        self.get_logger().info("⌨️ Keyboard Command Sent: open")
                    else:
                        print("시리얼이 연결되지 않았습니다.")
                        
                elif user_cmd in ("q", "quit", "exit", "종료"):
                    self.get_logger().info("키보드 입력 모드를 종료합니다.")
                    break
                else:
                    if user_cmd:    
                        print("잘못된 입력입니다. 'grip' 또는 'open'만 입력해주세요.")
            except EOFError:
                break
            except Exception as e:
                self.get_logger().error(f"Keyboard loop error: {e}")
                break
    # ----------------------------------------------------------------------

    def control_cb(self, request, response):
       
        try:
            if request.data:  # True -> Grip
                self.ser.write(b"grip\n")
                self.get_logger().info("📌 Sent: grip")
                response.message = "Grip Command Sent"
            else:             # False -> Open
                self.ser.write(b"open\n")
                self.get_logger().info("📌 Sent: open")
                response.message = "Open Command Sent"
            
            response.success = True
        except Exception as e:
            self.get_logger().error(f"❌ Service Error: {e}")
            response.success = False
            response.message = str(e)
            
        return response

def main(args=None):
    rclpy.init(args=args)
    node = GripperNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        if hasattr(node, 'ser') and node.ser.is_open:
            node.ser.close()
            node.get_logger().info("✅ Serial Closed")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
