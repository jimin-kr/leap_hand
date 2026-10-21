#!/usr/bin/env bash

PROJECT_DIR="$HOME/left_hand_project"
PORT="/dev/ttyUSB0"
PROFILE_VELOCITY="20"

cd "$PROJECT_DIR" || exit 1

echo "Starting LEAP Left Hand ROS2 system..."
echo "Project: $PROJECT_DIR"
echo "Port: $PORT"

if [ ! -e "$PORT" ]; then
    echo "[ERROR] $PORT not found."
    echo "Check U2D2 USB connection."
    exit 1
fi

sudo chmod a+rw "$PORT"

gnome-terminal --title="T1 left_hand_ratio_node REAL HAND" -- bash -c "
cd $PROJECT_DIR
source /opt/ros/humble/setup.bash
source .venv/bin/activate
python3 left_hand_ratio_node.py --port $PORT --profile-velocity $PROFILE_VELOCITY
exec bash
"

sleep 2

gnome-terminal --title="T2 left_hand_joint_state_pub RVIZ JOINTS" -- bash -c "
cd $PROJECT_DIR
source /opt/ros/humble/setup.bash
source .venv/bin/activate
python3 left_hand_joint_state_pub.py
exec bash
"

sleep 1

gnome-terminal --title="T3 robot_state_publisher URDF" -- bash -c "
cd $PROJECT_DIR
source /opt/ros/humble/setup.bash
ros2 run robot_state_publisher robot_state_publisher left_hand_simple.urdf
exec bash
"

sleep 1

if [ -f "$PROJECT_DIR/left_hand_simple.rviz" ]; then
    gnome-terminal --title="T4 RViz" -- bash -c "
    cd $PROJECT_DIR
    source /opt/ros/humble/setup.bash
    rviz2 -d left_hand_simple.rviz
    exec bash
    "
else
    gnome-terminal --title="T4 RViz" -- bash -c "
    cd $PROJECT_DIR
    source /opt/ros/humble/setup.bash
    rviz2
    exec bash
    "
fi

sleep 1

gnome-terminal --title="T5 command terminal" -- bash -c "
cd $PROJECT_DIR
source /opt/ros/humble/setup.bash
echo ''
echo 'Command terminal ready.'
echo ''
echo 'Test commands:'
echo 'ros2 topic pub --once /left_hand/close_ratio std_msgs/msg/Float32 \"{data: 0.5}\"'
echo 'ros2 topic pub --once /left_hand/close_ratio std_msgs/msg/Float32 \"{data: 0.0}\"'
echo ''
exec bash
"

echo "Done. Terminals opened."
