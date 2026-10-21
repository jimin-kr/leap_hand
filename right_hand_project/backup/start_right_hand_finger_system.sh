#!/usr/bin/env bash

PROJECT_DIR="$HOME/left_hand_project"
PORT="/dev/ttyUSB0"
PROFILE_VELOCITY="20"

cd "$PROJECT_DIR" || exit 1

echo "Starting LEAP Left Hand Finger Ratio ROS2 system..."
echo "Project: $PROJECT_DIR"
echo "Port: $PORT"
echo "Topic: /left_hand/finger_ratios"

if [ ! -e "$PORT" ]; then
    echo "[ERROR] $PORT not found."
    echo "Check U2D2 USB connection."
    exit 1
fi

sudo chmod a+rw "$PORT"

gnome-terminal --title="T1 FINGER REAL HAND NODE" -- bash -c "
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

sleep 1

gnome-terminal --title="T5 FINGER COMMAND TERMINAL" -- bash -c "
cd $PROJECT_DIR
source /opt/ros/humble/setup.bash
source .venv/bin/activate

echo ''
echo 'Finger command terminal ready.'
echo ''
echo 'Topic: /left_hand/finger_ratios'
echo 'Data order: [thumb, index, middle, ring]'
echo ''
echo 'Test commands:'
echo 'open:'
echo 'ros2 topic pub --once /left_hand/finger_ratios std_msgs/msg/Float32MultiArray \"{data: [0.0, 0.0, 0.0, 0.0]}\"'
echo ''
echo 'index only:'
echo 'ros2 topic pub --once /left_hand/finger_ratios std_msgs/msg/Float32MultiArray \"{data: [0.0, 1.0, 0.0, 0.0]}\"'
echo ''
echo 'full fist:'
echo 'ros2 topic pub --once /left_hand/finger_ratios std_msgs/msg/Float32MultiArray \"{data: [1.0, 1.0, 1.0, 1.0]}\"'
echo ''
exec bash
"

echo "Done. Finger ratio system terminals opened."
