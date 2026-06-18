## 🚀 실행 방법 (Getting Started)

본 프로젝트를 실행하기 위해서는 기본 워크스페이스(Underlay)와 현재 통합 워크스페이스(Overlay)를 순서대로 source 해야 합니다. **각 노드는 새로운 터미널을 열어 실행**하는 것을 권장합니다.

### 1. 터미널 환경 설정 (Sourcing)
새로운 터미널을 열 때마다 작업 디렉토리로 이동한 후, 아래 명령어를 순서대로 실행하여 환경을 설정합니다.

```bash
cd ~/integrated_ws

# 1. 기본이 되는 워크스페이스(Underlay) source
source ~/MasterPC_mock_test/install/setup.bash

# 2. 추가로 얹을 통합 워크스페이스(Overlay) source
source ~/integrated_ws/install/setup.bash

2. 노드 실행 (Running Nodes)
환경 설정이 완료된 터미널에서 다음 노드들을 목적에 맞게 실행합니다. (각 노드당 하나의 터미널 창을 사용해 주세요.)

📷 비전 시스템
ros2 run vision_pkg vision_node

🦾 하드웨어 및 그리퍼 제어
ros2 run hardware_pkg robot2_gpio_gripper_node
ros2 run hardware_pkg gripper_node

⚙️ 메인 매니퓰레이터 제어 및 명령
ros2 run control_pkg robot_node
ros2 run control_pkg command_node

🖥️ 수동 조작 및 시스템 테스트
ros2 run sml_system_pkg manual_order_node_test

---
