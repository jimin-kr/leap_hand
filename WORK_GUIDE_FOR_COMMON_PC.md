# 🖥️ 공용 PC 작업 전환 및 실행 완벽 가이드

본 문서는 개인 PC(노트북)에서 튜닝을 마친 **LEAP Hand 텔레옵 프로젝트**를 공용 PC로 옮겨서 바로 구동할 수 있도록 정리한 실전 가이드입니다.

---

## 🔌 1. 하드웨어 연결 (USB 포트 3개)

공용 PC 본체 후면(또는 USB 3.0 이상 권장 포트)에 3개의 기기를 연결합니다:
1. **LEAP Hand**: U2D2 변환기 연결 $\rightarrow$ 리눅스 장치명: `/dev/ttyUSB0`
2. **RealSense 카메라**: GoPro 손목 스트랩 마운트에 장착된 RealSense 카메라 연결 (파란색 USB 3.0 포트 필수)
3. **개인 USB / 외장 드라이브**: 데이터 저장 및 백업용

---

## 📂 2. 프로젝트 파일 복사

개인 USB에 담긴 `LEAP_HAND_TRANSFER/leap_hand` 폴더를 공용 PC의 작업 공간으로 복사합니다.

```bash
# 예시: 홈 디렉토리 아래 작업 폴더 생성 후 복사
mkdir -p ~/arm_lab_control
cp -r /media/$USER/78B1-8D0E/LEAP_HAND_TRANSFER/leap_hand ~/arm_lab_control/
cd ~/arm_lab_control/leap_hand
```
*(또는 이미 GitHub `https://github.com/jimin-kr/leap_hand.git`에 최신 파일이 푸시되어 있으므로, `git clone`으로 받아도 무방합니다)*

---

## ⚡ 3. 공용 PC 원클릭 환경 설정 (초기 1회)

공용 PC에 필요한 패키지 설치, 권한 설정, colcon 빌드를 자동으로 수행합니다:

```bash
cd ~/arm_lab_control/leap_hand
chmod +x setup_common_pc.sh
./setup_common_pc.sh
```

### [중요] USB 시리얼 포트 권한 확인
다이나믹셀 통신을 위해 현재 계정이 `dialout` 그룹에 속해 있어야 합니다:
* 스크립트 실행 후 만약 다이나믹셀 포트 접근 오류(Permission Denied)가 발생하면:
  ```bash
  # 일시적 권한 즉시 적용 (터미널마다)
  sudo chmod 666 /dev/ttyUSB0
  # 또는
  newgrp dialout
  ```

---

## 🔍 4. 연결 상태 자가 진단 (하드웨어 점검)

실제 구동 전에 1분 만에 점검할 수 있는 체크리스트입니다:

### ① 다이나믹셀 모터 16개 통신 및 전압 확인
```bash
cd ~/arm_lab_control/leap_hand
python3 left_hand_project/dxl_torque_off.py
```
* **정상 기준**: ID 0~15번까지 16개 모터 모두 `volt=4.9~5.0V`, `HW ERROR=0x00 [OK]`가 출력되어야 합니다.
* 만약 특정 모터에 에러가 감지되면 해당 스크립트가 자동으로 `Reboot` 및 안전 해제(Torque OFF)를 진행합니다.

### ② 카메라 인식 확인
```bash
# RealSense 카메라 인식 여부 확인
lsusb | grep Intel
# 또는 GUI 뷰어로 영상 스트림 확인
realsense-viewer
```

---

## 🚀 5. 실전 텔레옵 실행 (터미널 3개)

모든 터미널에서 반드시 ROS 환경을 source한 뒤 실행합니다:

```bash
source /opt/ros/humble/setup.bash
source ~/arm_lab_control/leap_hand/install/setup.bash
```

### [터미널 1] 실기체 모터 제어 노드
```bash
cd ~/arm_lab_control/leap_hand
python3 left_hand_project/left_hand_finger_ratio_node_safe.py
```
* *노드 실행 시 손가락이 가볍게 펴진 상태를 엔코더가 스캔하여 시작 기준으로 잡습니다.*
* *엄지 회전(15), 엄지 굽힘(7), 손가락 굽힘(10) 속도 프로파일이 기본 적용되어 있습니다.*

### [터미널 2] 비전 손 랜드마크 추적 노드
```bash
cd ~/arm_lab_control/leap_hand
ros2 run leap_teleop left_hand_teleop_node
```
* 카메라 화면에 손이 인식되면 초록색 랜드마크와 함께 실시간 파지 비율이 계산되어 토픽으로 발행됩니다.

### [터미널 3] 파지 시작 (ARM) 및 비상 대처
```bash
# 손 동작을 실제 모터로 전달 시작 (ARM):
ros2 topic pub --once /left_hand/arm std_msgs/msg/Bool "{data: true}"

# 일시 중지 (DISARM):
ros2 topic pub --once /left_hand/arm std_msgs/msg/Bool "{data: false}"

# 비상 정지 (즉시 전 모터 힘 풀기):
python3 left_hand_project/dxl_torque_off.py
```

---

## 💡 6. 꿀팁 및 트러블슈팅

1. **포트 이름이 `/dev/ttyUSB0`가 아닐 때**:
   * 카메라나 다른 USB 시리얼 기기 순서 때문에 `/dev/ttyUSB1` 등으로 잡힐 수 있습니다.
   * `ls -l /dev/ttyUSB*`로 확인 후 노드 실행 시 `--port /dev/ttyUSB1` 옵션을 붙여주시면 됩니다.
2. **엄지 굽힘 템포 조절**:
   * 손목 카메라 시점에서 컵을 잡을 때 엄지가 감싸는 속도를 바꾸고 싶다면:
     * 더 천천히: `--thumb-bend-velocity 5`
     * 더 빠르게: `--thumb-bend-velocity 10`
3. **원격/협업 동기화**:
   * 공용 PC에서 수정한 파라미터나 코드는 바로 GitHub에 커밋/푸시하시면 언제든 개인 노트북과 동기화할 수 있습니다:
     `git add . && git commit -m "Update" && git push`
