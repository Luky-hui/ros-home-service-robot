#!/usr/bin/env bash
set -euo pipefail

MISSION_TYPE="${1:-auto}"
TARGET_OBJECT="${2:-}"

source /opt/ros/noetic/setup.bash
source /home/robot/bobac3_ws/devel/setup.bash
source /home/robot/ros_workspace/devel/setup.bash --extend

rosrun home_service_mission home_config_check.py

roslaunch home_service_mission home_competition.launch \
  mission_type:="${MISSION_TYPE}" \
  target_object:="${TARGET_OBJECT}"
