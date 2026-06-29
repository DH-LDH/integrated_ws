# rb3_handeye_calib

Rainbow Robotics **RB3** + **Intel RealSense D435** (eye-in-hand) 핸드아이 캘리브레이션 ROS2 패키지.

> **Eye-in-hand** : 카메라가 그리퍼(TCP)에 장착됨.  
> 결과: **T_cam2gripper** – 카메라 원점이 그리퍼 좌표계에서 어디에 있는지.

---

## 목차

1. [환경 요구사항](#1-환경-요구사항)
2. [빌드](#2-빌드)
3. [STEP 0 – ChArUco 보드 출력](#3-step-0--charuco-보드-출력)
4. [STEP 1 – TCP pose publisher 확인](#4-step-1--tcp-pose-publisher-확인)
5. [STEP 2 – 샘플 수집](#5-step-2--샘플-수집)
6. [STEP 3 – 캘리브레이션 계산](#6-step-3--캘리브레이션-계산)
7. [결과 파일 설명](#7-결과-파일-설명)
8. [파라미터 레퍼런스](#8-파라미터-레퍼런스)
9. [자주 발생하는 문제](#9-자주-발생하는-문제)
10. [Euler 순서 확인 방법](#10-euler-순서-확인-방법)

---

## 1. 환경 요구사항

| 항목 | 버전 |
|------|------|
| OS | Ubuntu 22.04 |
| ROS2 | Humble |
| Python | 3.10.12 |
| OpenCV | ≥ 4.6 (4.7+ 권장) |
| rbpodo | Rainbow Robotics SDK |
| realsense2_camera | ROS2 Humble용 |

```bash
# RealSense 드라이버
sudo apt install ros-humble-realsense2-camera

# Python 의존성
pip install opencv-contrib-python numpy scipy pyyaml
```

---

## 2. 빌드

```bash
cd ~/integrated_ws
colcon build --packages-select rb3_handeye_calib
source install/setup.bash
```

---

## 3. STEP 0 – ChArUco 보드 출력

```bash
ros2 run rb3_handeye_calib charuco_board_generator \
  --ros-args \
  -p squares_x:=7 \
  -p squares_y:=5 \
  -p square_length_mm:=30.0 \
  -p marker_length_mm:=22.0 \
  -p output_path:=/tmp/charuco_board.png
```

출력된 `/tmp/charuco_board.png` 를 **100% 비율** (맞춤 인쇄 X)로 인쇄합니다.

> **중요: 인쇄 후 ruler로 흰 체스 칸의 실제 크기를 측정하세요.**  
> 측정값이 `square_length_mm` 와 다르면 해당 값을 실측값으로 바꿔 캘리브레이션하세요.

보드를 폼보드 / 알루미늄 판 등 **평평하고 딱딱한 면**에 구김 없이 부착합니다.

---

## 4. STEP 1 – TCP pose publisher 확인

```bash
# 터미널 A: TCP pose publish 시작
ros2 run rb3_handeye_calib tcp_pose_publisher \
  --ros-args \
  -p robot_ip:=10.0.2.7 \
  -p robot_name:=robot1
```

```bash
# 터미널 B: 데이터 확인
ros2 topic echo /robot1/tcp_pose_array
```

예상 출력:
```
data: [x_mm, y_mm, z_mm, rx_deg, ry_deg, rz_deg]
```

### Euler 순서 확인 (중요)

로봇을 알려진 자세로 이동시킨 후 TCP에서 읽은 rx/ry/rz 값이  
`euler_order='xyz'` 로 계산한 회전행렬과 일치하는지 확인합니다.  
([STEP 10 Euler 순서 확인 방법](#10-euler-순서-확인-방법) 참조)

---

## 5. STEP 2 – 샘플 수집

### 5-1. 전체 스택 실행 (RealSense + TCP publisher + collector)

```bash
ros2 launch rb3_handeye_calib realsense_handeye.launch.py \
    robot_ip:=10.0.2.7 \
    robot_name:=robot1 \
    session_dir:=/home/user/handeye_samples/session1
```

RealSense 없이 이미 카메라 드라이버가 실행 중이면:

```bash
ros2 launch rb3_handeye_calib collect_samples.launch.py \
    robot_ip:=10.0.2.7 \
    robot_name:=robot1 \
    session_dir:=/home/user/handeye_samples/session1
```

### 5-2. 샘플 캡처 절차

1. 보드를 카메라 앞에 놓습니다 (카메라가 그리퍼에 장착된 상태).
2. 로봇을 **다양한 자세**로 이동합니다 (아래 팁 참조).
3. 각 자세에서 다른 터미널로 캡처 서비스를 호출합니다:

```bash
ros2 service call /handeye/capture_sample std_srvs/srv/Trigger
```

성공 응답 예:
```
success: True
message: "샘플 #3 저장 완료. TCP=(...) | board_t=(...) | corners=24 | 총 4개"
```

실패 예:
```
success: False
message: "코너 수 부족: 5 < min_corners=8. 보드를 카메라에 더 가까이..."
```

**재시작/리셋:**
```bash
ros2 service call /handeye/reset_samples std_srvs/srv/Trigger
```

### 5-3. 좋은 샘플 수집 팁

| 권장 | 이유 |
|------|------|
| 최소 15개, 가급적 25개 이상 | 알고리즘 수렴 안정성 |
| 보드를 다양한 거리(0.3~0.8m)에 놓기 | 깊이 다양성 |
| 로봇 TCP를 ±30° 이상 기울이기 | 회전 다양성 |
| 이미지 내 보드가 화면의 다른 위치에 오도록 | 번역 다양성 |
| 로봇이 완전히 정지한 상태에서 캡처 | 모션 블러 방지 |
| `save_annotated:=true` 로 검출 이미지 확인 | 품질 검증 |

저장되는 파일:
```
session_dir/
  samples.yaml        ← 모든 샘플 데이터
  sample_000.png      ← 각 캡처의 annotated 이미지
  sample_001.png
  ...
```

---

## 6. STEP 3 – 캘리브레이션 계산

```bash
ros2 run rb3_handeye_calib handeye_solver \
  --ros-args \
  -p samples_yaml:=/home/user/handeye_samples/session1/samples.yaml \
  -p method:=Tsai \
  -p euler_order:=xyz
```

여러 method 비교 권장:

```bash
for METHOD in Tsai Park Horaud Andreff Daniilidis; do
  ros2 run rb3_handeye_calib handeye_solver \
    --ros-args \
    -p samples_yaml:=/home/user/handeye_samples/session1/samples.yaml \
    -p method:=$METHOD \
    -p output_yaml:=/home/user/handeye_samples/session1/result_${METHOD}.yaml
done
```

**잔차가 가장 낮은 method의 결과**를 사용합니다.  
잔차 기준: 회전 < 1.0 deg, 이동 < 2.0 mm (좋음), < 0.5 deg / < 1.0 mm (우수)

---

## 7. 결과 파일 설명

`result_handeye.yaml` 예시:

```yaml
method: Tsai
euler_order: xyz
n_samples: 20
residual_rotation_deg_mean: 0.312
residual_translation_mm_mean: 0.847
T_cam2gripper:
  rotation_matrix:
    - [0.9998, -0.0123,  0.0156]
    - [0.0121,  0.9999,  0.0089]
    - [-0.0157, -0.0087,  0.9998]
  quaternion_xyzw: [-0.00435, 0.00782, 0.00615, 0.99994]
  rpy_deg: [0.512, 0.897, 0.703]   # euler_order='xyz'
  translation_mm: [-52.3, -34.1, 12.7]   # 카메라 원점 위치 (mm)
  translation_m: [-0.0523, -0.0341, 0.0127]
```

`translation_mm` 의 물리적 의미:  
카메라 원점이 그리퍼(TCP) 좌표계에서 어떤 오프셋에 있는지를 나타냅니다.  
예: `robot_node.py` 의 `cam_x_off`, `cam_y_off` 와 비교해 검증하세요.

---

## 8. 파라미터 레퍼런스

### tcp_pose_publisher

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `robot_ip` | `10.0.2.7` | 로봇 IP |
| `robot_name` | `robot1` | topic namespace |
| `publish_rate` | `20.0` | Hz |
| `frame_id` | `base` | PoseStamped frame |
| `euler_order` | `xyz` | TCP Euler 순서 |

### sample_collector

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `robot_name` | `robot1` | tcp_pose_array topic namespace |
| `session_dir` | 자동 | 샘플 저장 디렉토리 |
| `squares_x` | `7` | |
| `squares_y` | `5` | |
| `square_length_mm` | `30.0` | **실측값으로 맞출 것** |
| `marker_length_mm` | `22.0` | **실측값으로 맞출 것** |
| `dictionary` | `DICT_4X4_50` | |
| `min_corners` | `8` | 최소 ChArUco 코너 수 |
| `euler_order` | `xyz` | |
| `save_annotated` | `true` | |

### handeye_solver

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `samples_yaml` | **필수** | 수집된 samples.yaml 경로 |
| `output_yaml` | 자동 | result_handeye.yaml 저장 경로 |
| `method` | `Tsai` | Tsai/Park/Horaud/Andreff/Daniilidis |
| `euler_order` | `xyz` | |

---

## 9. 자주 발생하는 문제

### "ArUco 마커 검출 실패"
- 조명을 밝게 하고 보드 정면으로 카메라를 향하게 합니다.
- `min_corners` 를 낮추거나 (최소 4) 보드를 크게 출력합니다.
- `cv2.aruco.detectMarkers` 가 마커를 못 찾는 경우: 카메라가 언디스토션이 필요하거나 해상도가 너무 낮습니다.

### "TCP pose가 수신되지 않았습니다"
- `tcp_pose_publisher` 가 실행 중인지 확인.
- rbpodo API의 CobotData 필드명이 다를 수 있음 → `tcp_pose_publisher.py`의 `state.sdata.tcp_pos` 를 실제 API 필드명으로 수정.

### 잔차가 너무 큼 (> 2 deg / > 5 mm)
1. `euler_order` 가 실제 로봇과 맞는지 확인 ([STEP 10](#10-euler-순서-확인-방법))
2. 샘플 수를 늘리고 자세 다양성을 높임
3. `square_length_mm` / `marker_length_mm` 를 실측값으로 보정
4. 모든 method 를 시도해 가장 낮은 잔차 선택
5. 이상치 샘플(코너 수 적음, 모션 블러)을 samples.yaml 에서 수동으로 제거 후 재실행

### calibrateHandEye 실패 / 수렴 안 됨
- 최소 3개의 샘플이 필요하나 실제로는 15개 이상부터 안정적
- 모든 샘플에서 로봇 자세가 거의 동일하면 실패 → 자세 다양성 확보 필요

---

## 10. Euler 순서 확인 방법

RB3 TCP의 rx/ry/rz 가 어떤 Euler 순서인지는 rbpodo 공식 문서에서 확인하거나 아래 실험으로 검증합니다.

```python
import numpy as np
from scipy.spatial.transform import Rotation

# 로봇을 rx=30, ry=0, rz=0 자세로 이동 후 TCP 읽기
# 예: tcp = [x, y, z, 30.0, 0.0, 0.0]

for order in ['xyz', 'zyx', 'zyz']:
    R = Rotation.from_euler(order, [30, 0, 0], degrees=True).as_matrix()
    print(f"{order}: {R}")
```

로봇 팔이 X축 방향으로만 30° 회전했을 때의 회전행렬과 일치하는 `order` 를 선택합니다.

일반적으로 Rainbow Robotics RB 시리즈는 **extrinsic XYZ** (`euler_order='xyz'`) 를 사용합니다.  
단, 최신 펌웨어 또는 Task Coordinate 설정에 따라 다를 수 있으니 반드시 확인하세요.
