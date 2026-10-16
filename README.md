# 🦾 LEAP Hand Left Teleoperation & Control System

본 레포지토리는 **LEAP Hand (왼손, XC330-M288-T 모터 16개)**의 비전 기반 텔레오퍼레이션(Teleoperation), 관절 각도 캘리브레이션, 그리고 안정적인 물체(원통형 컵 등) 파지를 위한 하드웨어 제어 파이프라인을 포함하고 있습니다.

---

## 📁 디렉토리 구조 및 핵심 파일 안내

```text
leap_hand/
├── leap_teleop/                                  # ROS 2 텔레옵 패키지
│   ├── config/
│   │   └── left_hand_motor_map.json              # [핵심] 16개 모터 캘리브레이션 맵 (Open/Close 각도, 틱, 방향)
│   ├── launch/                                   # RViz 및 시각화 런치 파일
│   ├── leap_teleop/
│   │   ├── rviz_teleop_node.py                   # 웹캠/RealSense 영상 기반 MediaPipe 손 추적 및 토픽 발행
│   │   └── left_hand_teleop_node.py              # 왼손 전용 텔레옵 실행 노드
│   └── rviz/                                     # RViz 설정 파일
├── left_hand_project/                            # 실기체 하드웨어 제어 및 유틸리티
│   ├── left_hand_finger_ratio_node_safe.py       # [핵심] 안전 하드웨어 제어 노드 (ARM/DISARM, 속도/가감속 프로파일)
│   ├── dxl_torque_off.py                         # [핵심] 비상 토크 OFF 및 하드웨어 에러 모터 자동 리부트 스크립트
│   ├── left_hand_dxl_run.py                      # 단일 손가락 및 관절 각도 검증용 스크립트 (Dry-run 지원)
│   ├── dxl_status_scan.py                        # 전체 모터 상태/전압/에러 진단 스캔
│   └── dxl_ping_scan.py                          # Dynamixel Ping 스캔
└── README.md
```

---

## ⚙️ 하드웨어 사양 및 모터 ID 매핑

* **통신 포트**: `/dev/ttyUSB0`
* **통신 속도**: `4,000,000 bps (4 Mbps)`
* **프로토콜**: Dynamixel Protocol 2.0
* **모터 기종**: ROBOTIS Dynamixel XC330-M288-T (16개)

### 관절 배치 및 모터 ID (Fingertip $\rightarrow$ Palm)
```text
  [엄지 Thumb]   [검지 Index]   [중지 Middle]   [약지 Ring]
  ID 15 (DIP)    ID 3  (DIP)    ID 7  (DIP)     ID 11 (DIP)   <- 끝마디 굽힘
  ID 14 (PIP)    ID 2  (PIP)    ID 6  (PIP)     ID 10 (PIP)   <- 중간마디 굽힘
  ID 13 (MCP)    ID 1  (Spread) ID 5  (Spread)  ID 9  (Spread)<- 좌우 벌림 (0° Neutral)
  ID 12 (Rot)    ID 0  (MCP)    ID 4  (MCP)     ID 8  (MCP)   <- 뿌리마디 굽힘
```

---

## 🎯 주요 캘리브레이션 및 제어 튜닝 내용

### 1. 전압 강하(`0x01 Input Voltage Error`) 및 스톨 해결
* **원인**: 데이지 체인 구조상 가장 끝단에 위치한 DIP 모터(ID 3, 7, 11)가 과도하게 굽혀져 기계적 플라스틱 한계(하드스톱)에 부딪힐 때 순간 전력 폭증으로 전압 강하 발생.
* **해결**: 끝단 가동 범위를 기존 `109.23°`에서 **`79.23°`**로 30° 축소하여 스톨 방지 및 전원 안정성 확보 (전 모터 4.9~5.0V 정상 유지).

### 2. 90도 회전(수직 파지) 시 중력 부하 대응
* 컵을 잡을 때 중간 마디(PIP: ID 2, 6, 10)가 컵 표면에 과도하게 짓눌리지 않도록 가동 각도를 기존 `70.8°`에서 **`60.0°`**로 완화.

### 3. 모터 떨림(Chatter) 억제 게인 튜닝
* 모터 위치 P Gain: `600`, I Gain: `0`, **D Gain: `200`** (미세 진동 억제)
* **Goal Current**: `350 mA` (과도한 쥐어짜기 방지 및 모터 보호)
* **Profile Acceleration**: `10` (급격한 가속 충격 완화)

### 4. 엄지 비동기 템포 제어 (Grasp Trajectory Shaping)
* **문제점**: 엄지 끝마디(14, 15번)가 먼저 접히면 주먹을 쥔 채 컵에 접근하여 컵이 튕겨나감.
* **해결**: 인간의 파지 시퀀스(Preshape $\rightarrow$ Enclosure)를 반영하여 속도 프로파일 차등 적용:
  * **엄지 회전/대립 (ID 12, 13)**: Velocity = **`15`**, Acc = **`15`** *(빠르게 컵 맞은편 위치 확보)*
  * **기타 손가락 (ID 0 ~ 11)**: Velocity = **`10`**, Acc = **`10`** *(자연스러운 파지 템포)*
  * **엄지 마디 굽힘 (ID 14, 15)**: Velocity = **`7`**, Acc = **`7`** *(엄지가 펴진 상태로 접근 후 마지막에 감쌈)*

---

## 🚀 실행 가이드 (Quick Start)

### 1. 패키지 빌드 (초기 1회 또는 설정 파일 수정 시)
```bash
cd ~/Documents/양태헌교수님학연생/arm_lab_control/leap_hand
colcon build --packages-select leap_teleop --symlink-install
source install/setup.bash
```

### 2. 노드 실행 (3개 터미널 권장)

#### **터미널 1: 실기체 안전 제어 노드**
```bash
cd ~/Documents/양태헌교수님학연생/arm_lab_control/leap_hand
source install/setup.bash
python3 left_hand_project/left_hand_finger_ratio_node_safe.py
```
> *초기 실행 시 손이 가볍게 펴진 상태를 엔코더가 스캔하여 Open 기준으로 잡습니다.*

#### **터미널 2: 웹캠 비전 텔레옵 노드**
```bash
cd ~/Documents/양태헌교수님학연생/arm_lab_control/leap_hand
source install/setup.bash
ros2 run leap_teleop left_hand_teleop_node
```

#### **터미널 3: 비상 정지 및 상태 점검 (필요 시)**
```bash
# 비상 정지: 모든 모터 토크 OFF 및 에러 모터 자동 리부트
python3 left_hand_project/dxl_torque_off.py

# 모터 동작 활성화 (ARM 명령 - 웹캠 연동 시작 시):
ros2 topic pub --once /left_hand/arm std_msgs/msg/Bool "{data: true}"

# 모터 동작 중지 (DISARM 명령):
ros2 topic pub --once /left_hand/arm std_msgs/msg/Bool "{data: false}"
```

---

## 🛠️ 주요 트러블슈팅

* **모터에 빨간 불이 들어오거나 텔레옵 중 갑자기 멈췄을 때**:
  ```bash
  python3 left_hand_project/dxl_torque_off.py
  ```
  위 스크립트를 실행하면 하드웨어 에러 레지스터(`0x70`)를 조회하여 에러가 발생한 모터만 자동으로 `Reboot`하고 전체 토크를 안전하게 해제합니다.
* **단일 손가락 각도만 단독 테스트하고 싶을 때**:
  ```bash
  # 검지 50% 굽힘 드라이런 (명령 전송 안함)
  python3 left_hand_project/left_hand_dxl_run.py --finger index --ratio 0.5

  # 실제 모터 동작 실행 시 --execute 추가
  python3 left_hand_project/left_hand_dxl_run.py --finger index --ratio 0.5 --execute
  ```

---

## 🔮 향후 작업 계획 (Next Step)

1. **GoPro 손목 스트랩 기반 RealSense 마운트 결합**:
   * 사전 설계/제작 완료된 전용 마운트를 조종자 손목에 착용 (Wrist-mounted 1인칭 시점 확보).
   * 카메라 시점 고정으로 인한 폐색(Occlusion) 한계 극복 및 지속적인 랜드마크 추적 실증.
2. **로봇 암(Arm) + LEAP Hand 통합 매니퓰레이션 연동**:
   * 손목 카메라 궤적과 엔드이펙터 좌표계를 매핑하여 실제 물체 픽앤플레이스(Pick-and-Place) 수행.
