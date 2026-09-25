#!/usr/bin/env bash
set -u

source /opt/ros/noetic/setup.bash
source /home/robot/bobac3_ws/devel/setup.bash
source /home/robot/ros_workspace/devel/setup.bash --extend

export LASER_TYPE="${LASER_TYPE:-CSPC}"
export REI_ROBOT="${REI_ROBOT:-bobac3}"

failures=0

check_package() {
  local package="$1"
  if rospack find "${package}" >/dev/null 2>&1; then
    echo "OK package ${package}: $(rospack find "${package}")"
  else
    echo "FAIL package ${package}: not found"
    failures=$((failures + 1))
  fi
}

check_file() {
  local path="$1"
  if [ -e "${path}" ]; then
    echo "OK file ${path}"
  else
    echo "FAIL file ${path}: missing"
    failures=$((failures + 1))
  fi
}

echo "=== Environment ==="
echo "ROS_DISTRO=${ROS_DISTRO:-}"
echo "LASER_TYPE=${LASER_TYPE}"
echo "REI_ROBOT=${REI_ROBOT}"
echo "ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH:-}"

echo "=== Required Packages ==="
for package in \
  rei_robot_base \
  bobac3_description \
  bobac3_navigation \
  cspc_lidar \
  map_manager \
  ros_map_edit \
  relative_move \
  ar_pose \
  face_rec \
  usb_cam
do
  check_package "${package}"
done

echo "=== Required Config Files ==="
check_file "/home/robot/bobac3_ws/src/ar_pose/config/${REI_ROBOT}.yaml"

echo "=== Camera Devices ==="
if [ -e "${BASE_CAMERA_DEVICE:-/dev/base_camera}" ]; then
  echo "OK base camera device ${BASE_CAMERA_DEVICE:-/dev/base_camera}"
else
  echo "WARN base camera device ${BASE_CAMERA_DEVICE:-/dev/base_camera} missing"
  echo "     Set BASE_CAMERA_DEVICE=/dev/videoN or create a udev symlink before using ar_base.launch."
fi

if ls /dev/video* >/dev/null 2>&1; then
  ls -l /dev/video*
fi

echo "=== Launch Parse Checks ==="
if roslaunch --nodes bobac3_navigation bobac3_nav_2d.launch >/tmp/real_nav_nodes.$$ 2>/tmp/real_nav_nodes.err.$$; then
  echo "OK bobac3_nav_2d.launch parses"
  cat /tmp/real_nav_nodes.$$
else
  echo "FAIL bobac3_nav_2d.launch does not parse"
  cat /tmp/real_nav_nodes.err.$$
  failures=$((failures + 1))
fi
rm -f /tmp/real_nav_nodes.$$ /tmp/real_nav_nodes.err.$$

if roslaunch --nodes ar_pose ar_base.launch >/tmp/ar_base_nodes.$$ 2>/tmp/ar_base_nodes.err.$$; then
  echo "OK ar_base.launch parses"
  cat /tmp/ar_base_nodes.$$
else
  echo "FAIL ar_base.launch does not parse"
  cat /tmp/ar_base_nodes.err.$$
  failures=$((failures + 1))
fi
rm -f /tmp/ar_base_nodes.$$ /tmp/ar_base_nodes.err.$$

echo "=== Runtime Checks If ROS Master Is Running ==="
if timeout 3 rostopic list >/tmp/real_robot_topics.$$ 2>/tmp/real_robot_topics.err.$$; then
  for topic in /cmd_vel /odom /scan /map /amcl_pose /move_base/status; do
    if grep -qx "${topic}" /tmp/real_robot_topics.$$; then
      echo "OK topic ${topic}: $(rostopic type "${topic}" 2>/dev/null || true)"
    else
      echo "WARN topic ${topic}: not present now"
    fi
  done
  if rosservice list 2>/dev/null | grep -qx /relative_move; then
    echo "OK service /relative_move: $(rosservice type /relative_move)"
  else
    echo "WARN service /relative_move: not present now"
  fi
else
  echo "INFO ROS master is not running; skipped runtime topic/service checks."
fi
rm -f /tmp/real_robot_topics.$$ /tmp/real_robot_topics.err.$$

if [ "${failures}" -eq 0 ]; then
  echo "REAL_ROBOT_PREFLIGHT_OK"
else
  echo "REAL_ROBOT_PREFLIGHT_NOT_READY failures=${failures}"
fi

exit "${failures}"
