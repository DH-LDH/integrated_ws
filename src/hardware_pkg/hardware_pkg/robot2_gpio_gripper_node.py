import time
import threading

import rbpodo as rb
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool


class Robot2GpioGripperNode(Node):
    def __init__(self):
        super().__init__("robot2_gpio_gripper_node")

        self.robot_ip = self.declare_parameter("robot_ip", "10.0.2.8").value
        self.service_name = self.declare_parameter(
            "service_name",
            "/robot2/control_gripper",
        ).value
        self.pulse_time = float(self.declare_parameter("pulse_time", 0.2).value)

        self.robot = rb.Cobot(self.robot_ip)
        self.rc = rb.ResponseCollector()
        self.robot.set_operation_mode(self.rc, rb.OperationMode.Real)
        self.robot.set_speed_bar(self.rc, 1.0)

        self.srv = self.create_service(SetBool, self.service_name, self.control_cb)

        self.get_logger().info(
            f"Robot2 GPIO gripper ready: ip={self.robot_ip}, service={self.service_name}"
        )
        self.get_logger().info("GPIO mapping: open=01, close=10, then reset=00")

        self.keyboard_thread = threading.Thread(target=self.keyboard_loop, daemon=True)
        self.keyboard_thread.start()

    def pulse_gpio(self, value):
        self.robot.set_dout_bit_combination(
            self.rc,
            0,
            2,
            value,
            rb.Endian.LittleEndian,
        )
        time.sleep(self.pulse_time)
        self.robot.set_dout_bit_combination(
            self.rc,
            0,
            2,
            0,
            rb.Endian.LittleEndian,
        )

    def open_gripper(self):
        self.get_logger().info("Robot2 gripper open")
        self.pulse_gpio(1)

    def close_gripper(self):
        self.get_logger().info("Robot2 gripper close")
        self.pulse_gpio(2)

    def control_cb(self, request, response):
        try:
            if request.data:
                self.close_gripper()
                response.message = "Robot2 GPIO close sent"
            else:
                self.open_gripper()
                response.message = "Robot2 GPIO open sent"
            response.success = True
        except Exception as e:
            self.get_logger().error(f"Robot2 GPIO gripper error: {e}")
            response.success = False
            response.message = str(e)
        return response

    def keyboard_loop(self):
        print("\n[Robot2 GPIO Gripper] Type: open / close / quit")
        while rclpy.ok():
            try:
                command = input("robot2-gripper> ").strip().lower()
            except EOFError:
                return

            try:
                if command in ("open", "release", "0"):
                    self.open_gripper()
                elif command in ("close", "grip", "grab", "1"):
                    self.close_gripper()
                elif command in ("quit", "exit", "q"):
                    self.get_logger().info("Keyboard exit requested")
                    rclpy.shutdown()
                    return
                elif command:
                    print("Use: open / close / quit")
            except Exception as e:
                self.get_logger().error(f"Keyboard gripper error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = Robot2GpioGripperNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()