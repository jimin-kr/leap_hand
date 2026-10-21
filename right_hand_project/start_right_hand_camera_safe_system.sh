#!/bin/bash
# Colleague's SAFE real-hand pipeline (simple URDF in RViz).
# Paths are resolved from this script's location (the old version assumed ~/left_hand_project).
# Motor calibration: ../leap_teleop/config/left_hand_motor_map.json (open = 2048 for all motors)

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PORT="${PORT:-/dev/ttyUSB0}"

if [ ! -e "$PORT" ]; then
    echo "[ERROR] $PORT not found. Check U2D2 USB connection / hand power."
    exit 1
fi

if [ ! -w "$PORT" ]; then
    echo "[ERROR] no write permission on $PORT (user must be in 'dialout' group)."
    exit 1
fi

# Activate a venv only if one exists (the copy here excludes .venv)
ENV_SETUP="source /opt/ros/humble/setup.bash"
if [ -f "$PROJECT_DIR/.venv/bin/activate" ]; then
    ENV_SETUP="$ENV_SETUP && source $PROJECT_DIR/.venv/bin/activate"
fi

gnome-terminal --title="T1 SAFE REAL HAND NODE" -- bash -c "
cd '$PROJECT_DIR' && $ENV_SETUP
python3 left_hand_finger_ratio_node_safe.py --port $PORT --profile-velocity 8
exec bash
"

sleep 1

gnome-terminal --title="T2 FINGER JOINT STATE PUB" -- bash -c "
cd '$PROJECT_DIR' && $ENV_SETUP
python3 left_hand_finger_joint_state_pub.py
exec bash
"

sleep 1

gnome-terminal --title="T3 ROBOT STATE PUBLISHER" -- bash -c "
cd '$PROJECT_DIR' && source /opt/ros/humble/setup.bash
ros2 run robot_state_publisher robot_state_publisher left_hand_simple.urdf
exec bash
"

sleep 1

gnome-terminal --title="T4 RVIZ" -- bash -c "
cd '$PROJECT_DIR' && source /opt/ros/humble/setup.bash
rviz2
exec bash
"

sleep 1

gnome-terminal --title="T5 STABLE CAMERA PUBLISHER" -- bash -c "
cd '$PROJECT_DIR' && $ENV_SETUP
python3 left_hand_camera_finger_pub_stable.py --camera 0 --mirror --max-ratio 0.7 --alpha 0.12 --max-step 0.025 --deadzone 0.10 --lost-timeout 0.7
exec bash
"

sleep 1

gnome-terminal --title="T6 COMMAND TERMINAL" -- bash -c "
cd '$PROJECT_DIR' && $ENV_SETUP
echo 'Command terminal ready.'
echo 'Arm:'
echo 'ros2 topic pub --once /left_hand/arm std_msgs/msg/Bool \"{data: true}\"'
echo ''
echo 'Disarm:'
echo 'ros2 topic pub --once /left_hand/arm std_msgs/msg/Bool \"{data: false}\"'
exec bash
"
