#!/usr/bin/env bash

PROJECT_DIR=/home/alswotms/left_hand_project
PORT=/dev/ttyUSB0
PROFILE_VELOCITY=15
CAMERA_INDEX=0

MAX_RATIO=0.7
ALPHA=0.18
MAX_STEP=0.04
DEADZONE=0.06
LOST_TIMEOUT=0.5

cd /home/alswotms/left_hand_project || exit 1

echo Starting
