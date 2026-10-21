#!/usr/bin/env bash
source /opt/ros/humble/setup.bash
ros2 topic pub --once /left_hand/arm std_msgs/msg/Bool "{data: true}"
