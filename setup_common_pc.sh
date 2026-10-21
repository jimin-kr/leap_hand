#!/usr/bin/env bash
set -e

echo "=========================================================="
echo "🦾 LEAP Hand 공용 PC 환경 설정 스크립트"
echo "=========================================================="

# 1. ROS 2 환경 확인
if [ -z "$ROS_DISTRO" ]; then
    if [ -f "/opt/ros/humble/setup.bash" ]; then
        echo "[1/5] ROS 2 Humble 환경을 로드합니다..."
        source /opt/ros/humble/setup.bash
    else
        echo "[경고] /opt/ros/humble/setup.bash 를 찾을 수 없습니다."
        echo "공용 PC에 ROS 2 (Humble)가 설치되어 있는지 확인해 주세요."
    fi
else
    echo "[1/5] 현재 ROS 2 ($ROS_DISTRO) 환경이 감지되었습니다."
fi

# 2. 필수 시스템 패키지 설치
echo "[2/5] 필수 빌드 도구 및 라이브러리 설치 확인..."
REQUIRED_PKGS="python3-pip python3-colcon-common-extensions ros-$ROS_DISTRO-joint-state-publisher ros-$ROS_DISTRO-robot-state-publisher ros-$ROS_DISTRO-rviz2"
MISSING_PKGS=""
for pkg in $REQUIRED_PKGS; do
    if ! dpkg -s "$pkg" &>/dev/null; then
        MISSING_PKGS="$MISSING_PKGS $pkg"
    fi
done

if [ -n "$MISSING_PKGS" ]; then
    echo "  -> 미설치 시스템 패키지 설치 진행:$MISSING_PKGS"
    sudo apt update
    sudo apt install -y $MISSING_PKGS
else
    echo "  -> 필수 시스템 패키지가 이미 모두 설치되어 있습니다."
fi

# 3. 파이썬 의존성 패키지 설치
echo "[3/5] 파이썬 패키지 설치 (dynamixel_sdk, opencv, mediapipe, pyrealsense2 등)..."
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
pip3 install --user -r "${SCRIPT_DIR}/requirements.txt"


# 4. 시리얼 포트(다이나믹셀 USB) 권한 설정
echo "[4/5] USB 시리얼 포트(/dev/ttyUSB*) 권한 설정 (dialout 그룹)..."
if groups $USER | grep &>/dev/null '\bdialout\b'; then
    echo "  -> 이미 dialout 그룹에 포함되어 있습니다."
else
    echo "  -> 사용자를 dialout 그룹에 추가합니다: sudo usermod -aG dialout $USER"
    sudo usermod -aG dialout $USER
    echo "  [주의] dialout 그룹 적용을 위해 세션을 로그아웃 후 다시 로그인하거나,"
    echo "         새 터미널에서 'newgrp dialout' 명령을 실행하세요."
fi

# 5. colcon 빌드
echo "[5/5] leap_teleop 패키지 빌드 진행..."
cd "${SCRIPT_DIR}"
colcon build --packages-select leap_teleop --symlink-install

echo "=========================================================="
echo "🎉 모든 환경 설정 및 빌드가 완료되었습니다!"
echo "실행 전 다음 명령어를 터미널마다 실행해 주세요:"
echo "source /opt/ros/humble/setup.bash"
echo "source ${SCRIPT_DIR}/install/setup.bash"
echo "=========================================================="
