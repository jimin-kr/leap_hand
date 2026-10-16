#!/usr/bin/env bash
source /opt/ros/humble/setup.bash
ros2 topic pub --once /left_hand/finger_ratios std_msgs/msg/Float32MultiArray "{data: [0.3, 0.0, 0.0, 1.0]}"
