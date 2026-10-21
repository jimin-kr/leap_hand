#!/usr/bin/env bash

PROJECT_DIR="$HOME/left_hand_project"
PORT="/dev/ttyUSB0"
PROFILE_VELOCITY="15"
CAMERA_INDEX="0"
MAX_RATIO="0.5"

cd "$PROJECT_DIR" || exit 1

echo "Starting LEAP Left Hand Camera Tracking System..."
echo "Project: $PROJECT_DIR"
echo "Dynamixel port: $PORT"
echo "Camera index: $CAMERA_INDEX"
echo "Max ratio: $MAX_RATIO"
echo "Topic: /left_hand/finger_ratios"

if [ ! -e "$PORT" ]; then
    echo "[ERROR] $PORT not found."
    echo "Check U2D2 USB connection."
    exit 1
fi

sudo chmod a+rw "$PORT"

gnome-terminal --title="T1 REAL HAND FINGER NODE" -- bash -c "
cd $PROJECT_DIR
source /opt/ros/humble/setup.bash
source .venv/bin/activate
python3 left_hand_finger_ratio_node.py --port $PORT --profile-velocity $PROFILE_VELOCITY
exec bash
"

sleep 2

gnome-terminal --title="T2 FINGER JOINT STATE PUB" -- bash -c "
cd $PROJECT_DIR
source /opt/ros/humble/setup.bash
source .venv/bin/activate
python3 left_hand_finger_joint_state_pub.py
exec bash
"

sleep 1

gnome-terminal --title="T3 ROBOT STATE PUBLISHER" -- bash -c "
cd $PROJECT_DIR
source /opt/ros/humble/setup.bash
ros2 run robot_state_publisher robot_state_publisher left_hand_simple.urdf
exec bash
"

sleep 1

if [ -f "$PROJECT_DIR/left_hand_simple.rviz" ]; then
    gnome-terminal --title="T4 RVIZ" -- bash -c "
    cd $PROJECT_DIR
    source /opt/ros/humble/setup.bash
    rviz2 -d left_hand_simple.rviz
    exec bash
    "
else
    gnome-terminal --title="T4 RVIZ" -- bash -c "
    cd $PROJECT_DIR
    source /opt/ros/humble/setup.bash
    rviz2
    exec bash
    "
fi

sleep 2

gnome-terminal --title="T5 CAMERA FINGER PUBLISHER" -- bash -c "
cd $PROJECT_DIR
source /opt/ros/humble/setup.bash
source .venv/bin/activate
python3 left_hand_camera_finger_pub.py --camera $CAMERA_INDEX --mirror --max-ratio $MAX_RATIO
exec bash
"

sleep 1

gnome-terminal --title="T6 CHECK / COMMAND TERMINAL" -- bash -c "
cd $PROJECT_DIR
source /opt/ros/humble/setup.bash
source .venv/bin/activate

echo ''
echo 'Camera tracking command terminal ready.'
echo ''
echo 'Check nodes:'
echo 'ros2 node list | grep left_hand'
echo ''
echo 'Check topics:'
echo 'ros2 topic list | grep -E \"finger|joint_states\"'
echo ''
echo 'Manual open command:'
echo 'ros2 topic pub --once /left_hand/finger_ratios std_msgs/msg/Float32MultiArray \"{data: [0.0, 0.0, 0.0, 0.0]}\"'
echo ''
echo 'Manual full fist command:'
echo 'ros2 topic pub --once /left_hand/finger_ratios std_msgs/msg/Float32MultiArray \"{data: [1.0, 1.0, 1.0, 1.0]}\"'
echo ''
echo 'Camera window starts as PUBLISH OFF.'
echo 'Click camera window and press s to start/stop publishing.'
echo 'Press q to quit camera.'
echo ''
exec bash
"

echo "Done. Camera tracking system terminals opened."
echo "Important: Camera publisher starts with PUBLISH OFF. Press 's' in camera window to start tracking."
