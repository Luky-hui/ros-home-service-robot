#!/usr/bin/env bash
set -euo pipefail

source /opt/ros/noetic/setup.bash

REPO_URL="${REI_ROBOT_BASE_REPO:-https://gitee.com/reinovo/rei_robot_base.git}"
WORKSPACE="${REI_ROBOT_BASE_WORKSPACE:-/home/robot/bobac3_ws}"
TARGET="${WORKSPACE}/src/rei_robot_base"

if [ -d "${TARGET}/.git" ]; then
  echo "rei_robot_base already exists: ${TARGET}"
  git -C "${TARGET}" remote -v
else
  echo "Cloning rei_robot_base from ${REPO_URL}"
  git clone "${REPO_URL}" "${TARGET}"
fi

if [ -d "${TARGET}/libs" ] || find "${TARGET}" -maxdepth 3 -name '*.so' | grep -q .; then
  echo "Binary libraries found under ${TARGET}:"
  find "${TARGET}" -maxdepth 4 -name '*.so' -print
else
  echo "WARN no .so files found under ${TARGET}. Verify the official package includes required binary libraries."
fi

source /home/robot/bobac3_ws/devel/setup.bash
if [ -f /home/robot/ros_workspace/devel/setup.bash ]; then
  source /home/robot/ros_workspace/devel/setup.bash --extend
fi

cd "${WORKSPACE}"
catkin_make

source "${WORKSPACE}/devel/setup.bash"
rospack find rei_robot_base
echo "REI_ROBOT_BASE_INSTALL_OK"
