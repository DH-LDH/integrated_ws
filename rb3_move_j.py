#!/usr/bin/env python3
import numpy as np
import rbpodo as rb


# ==============================
# 여기만 필요하면 수정
# ==============================
ROBOT_IP = "10.0.2.7"

SPEED_BAR = 0.2   # 전체 속도 비율, 0.1 ~ 1.0
MOVE_SPEED = 10.0 # move_j 속도
MOVE_ACC = 20.0   # move_j 가속도
# ==============================


def move_j(robot, rc, joints):
    q = np.array(joints, dtype=np.float64)

    print()
    print("[INFO] move_j 실행")
    print("[INFO] joints:", q.tolist())
    print("[INFO] speed:", MOVE_SPEED)
    print("[INFO] acc:", MOVE_ACC)

    robot.flush(rc)
    robot.move_j(rc, q, MOVE_SPEED, MOVE_ACC)

    ret = robot.wait_for_move_started(rc, 1.0)
    if ret.type() == rb.ReturnType.Success:
        print("[INFO] Move started")
        robot.wait_for_move_finished(rc)
        print("[INFO] Move finished")
    else:
        print("[WARN] Move did not start")

    rc.error().throw_if_not_empty()


def parse_joints(text):
    parts = text.replace(",", " ").split()

    if len(parts) != 6:
        raise ValueError("관절값은 반드시 6개 입력해야 합니다.")

    return [float(x) for x in parts]


def main():
    print("================================")
    print(" RB3 move_j 터미널 입력 프로그램")
    print("================================")
    print(f"[INFO] Robot IP: {ROBOT_IP}")
    print(f"[INFO] SPEED_BAR: {SPEED_BAR}")
    print(f"[INFO] MOVE_SPEED: {MOVE_SPEED}")
    print(f"[INFO] MOVE_ACC: {MOVE_ACC}")
    print()
    print("입력 예시:")
    print("93.26 -4.86 42.37 -1.49 119.0 2.28")
    print()
    print("종료하려면 q 입력")
    print("================================")

    robot = rb.Cobot(ROBOT_IP)
    rc = rb.ResponseCollector()

    robot.set_speed_bar(rc, SPEED_BAR)

    while True:
        try:
            text = input("\n관절값 6개 입력 > ").strip()

            if text.lower() in ["q", "quit", "exit"]:
                print("[INFO] 종료합니다.")
                break

            joints = parse_joints(text)

            confirm = input("이 좌표로 이동할까요? [y/N] ").strip().lower()
            if confirm != "y":
                print("[INFO] 이동 취소")
                continue

            move_j(robot, rc, joints)

        except KeyboardInterrupt:
            print("\n[INFO] Ctrl+C 입력됨. 종료합니다.")
            break

        except Exception as e:
            print("[ERROR]", e)
            print("다시 입력하세요.")
            print("예시: 93.26 -4.86 42.37 -1.49 119.0 2.28")


if __name__ == "__main__":
    main()
