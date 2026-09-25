#!/usr/bin/env bash
set -Eeuo pipefail

LOG_DIR="${RAICOM_ALL_LOG_DIR:-/tmp/raicom_national_all_$(date +%Y%m%d_%H%M%S)}"
MAP_FILE_NAME="${RAICOM_MAP_FILE_NAME:-national_2026}"
START_NAV="${RAICOM_ALL_START_NAV:-true}"
CLEAN_START="${RAICOM_ALL_CLEAN_START:-true}"
RESET_START="${RAICOM_ALL_RESET_START:-true}"
OPEN_NAV_RVIZ="${RAICOM_OPEN_NAV_RVIZ:-true}"
GAZEBO_GUI="${RAICOM_GAZEBO_GUI:-true}"
KEEP_NAV_AFTER_ALL="${RAICOM_KEEP_NAV_AFTER_ALL:-true}"
FAST_TASK_SWITCH="${RAICOM_FAST_TASK_SWITCH:-true}"
HEAD_CAMERA_CHECK_EACH_TASK="${RAICOM_HEAD_CAMERA_CHECK_EACH_TASK:-false}"
REUSE_HOME_STACK="${RAICOM_REUSE_HOME_STACK:-true}"

mkdir -p "${LOG_DIR}"

echo "RAICOM national all-task log directory: ${LOG_DIR}"

timestamp() {
  date '+%Y-%m-%d %H:%M:%S'
}

set +u
source /opt/ros/noetic/setup.bash
source /home/robot/bobac3_ws/devel/setup.bash
source /home/robot/ros_workspace/devel/setup.bash --extend
set -u

export ROS_MASTER_URI="${ROS_MASTER_URI:-http://localhost:11311}"
export ROS_HOSTNAME="${ROS_HOSTNAME:-ubuntu}"
export LASER_TYPE="${LASER_TYPE:-CSPC}"
export REI_ROBOT="${REI_ROBOT:-bobac3sim}"
export DISPLAY="${DISPLAY:-:0}"
export RAICOM_FACE_MODE="${RAICOM_FACE_MODE:-any}"
export RAICOM_ASSISTANT_VISION_MONITOR="${RAICOM_ASSISTANT_VISION_MONITOR:-true}"
unset LIBGL_ALWAYS_SOFTWARE || true

NAV_PID=""

cleanup_on_error() {
  local code=$?
  echo "RAICOM all-task script stopped with code ${code}. Logs: ${LOG_DIR}" >&2
  if [[ -n "${NAV_PID}" && "${KEEP_NAV_AFTER_ALL}" != "true" ]]; then
    kill "${NAV_PID}" >/dev/null 2>&1 || true
  fi
  exit "${code}"
}
trap cleanup_on_error ERR INT TERM

clean_previous_sim() {
  echo "Cleaning previous simulation/navigation processes..."
  pkill -f 'roslaunch bobac3_navigation demo_nav_2d.launch' || true
  pkill -f 'roslaunch bobac3_description gazebo.launch' || true
  pkill -f gzserver || true
  pkill -f gzclient || true
  pkill -f rosmaster || true
  pkill -f roscore || true
  sleep 4
}

start_navigation() {
  echo "Starting navigation and simulation: map=${MAP_FILE_NAME} rviz=${OPEN_NAV_RVIZ} gazebo_gui=${GAZEBO_GUI}"
  roslaunch bobac3_navigation demo_nav_2d.launch \
    map_file_name:="${MAP_FILE_NAME}" \
    open_nav_rviz:="${OPEN_NAV_RVIZ}" \
    gazebo_gui:="${GAZEBO_GUI}" \
    >"${LOG_DIR}/navigation.log" 2>&1 &
  NAV_PID="$!"
  echo "Navigation roslaunch pid=${NAV_PID}; log=${LOG_DIR}/navigation.log"
  sleep 5
}

reset_start_pose() {
  if [[ "${RESET_START}" == "true" ]]; then
    echo "Resetting robot pose to start..."
    rosrun raicom_supervisor reset_national_robot_pose.py --waypoint start | tee "${LOG_DIR}/reset_start.log"
  fi
}


wait_home_stack_clear() {
  local remaining=""
  for _ in $(seq 1 30); do
    remaining=$(rosnode list 2>/dev/null | grep -E '^/(home_service_mission|local_voice_node|raicom_vision|face_detection|head_camera)$' || true)
    if [[ -z "${remaining}" ]]; then
      sleep 1
      return 0
    fi
    echo "Waiting for previous task nodes to exit: ${remaining//$'\n'/ }"
    sleep 1
  done
  echo "WARN previous task nodes still visible after wait: ${remaining//$'\n'/ }" >&2
}

wait_head_camera_device() {
  local enabled="${RAICOM_WAIT_HEAD_CAMERA:-true}"
  if [[ "${enabled}" != "true" && "${enabled}" != "1" && "${enabled}" != "yes" ]]; then
    return 0
  fi
  if ! command -v v4l2-ctl >/dev/null 2>&1; then
    sleep "${RAICOM_HEAD_CAMERA_SETTLE_SECONDS:-3}"
    return 0
  fi
  local device="${HEAD_CAMERA_DEVICE:-}"
  if [[ -z "${device}" ]]; then
    device="$("$(rospack find ar_pose)/scripts/resolve_head_camera_device.sh" 2>/dev/null || true)"
  fi
  device="${device:-/dev/video0}"
  local attempts="${RAICOM_HEAD_CAMERA_READY_ATTEMPTS:-12}"
  local delay="${RAICOM_HEAD_CAMERA_READY_DELAY:-2}"
  local settle="${RAICOM_HEAD_CAMERA_SETTLE_SECONDS:-1}"
  local log="/tmp/raicom_head_camera_ready.$$"
  for _ in $(seq 1 "${attempts}"); do
    if [[ -e "${device}" ]] && timeout 8 v4l2-ctl -d "${device}" --stream-mmap --stream-count=2 >"${log}" 2>&1; then
      echo "Head camera is ready: ${device}"
      rm -f "${log}"
      sleep "${settle}"
      return 0
    fi
    echo "Waiting for head camera ${device} to become stream-ready..."
    tail -3 "${log}" 2>/dev/null || true
    sleep "${delay}"
  done
  echo "WARN head camera ${device} was not stream-ready; continuing so roslaunch can retry/respawn." >&2
  tail -8 "${log}" 2>/dev/null || true
  rm -f "${log}"
}

run_task() {
  local task_name="$1"
  local log_name="$2"
  shift 2
  TASK_INDEX=$((TASK_INDEX + 1))
  echo
  echo "========== START ${task_name} $(timestamp) =========="
  if [[ "${TASK_INDEX}" -eq 1 || "${HEAD_CAMERA_CHECK_EACH_TASK}" == "true" || "${HEAD_CAMERA_CHECK_EACH_TASK}" == "1" || "${HEAD_CAMERA_CHECK_EACH_TASK}" == "yes" ]]; then
    wait_head_camera_device
  else
    echo "Skipping repeated head-camera precheck for faster task switch."
  fi
  if [[ "${FAST_TASK_SWITCH}" == "true" || "${FAST_TASK_SWITCH}" == "1" || "${FAST_TASK_SWITCH}" == "yes" ]]; then
    if [[ "${TASK_INDEX}" -gt 1 ]]; then
      export RAICOM_FAST_NAV_READY="${RAICOM_FAST_NAV_READY:-true}"
      export RAICOM_SKIP_AUDIO_SETUP="${RAICOM_SKIP_AUDIO_SETUP:-true}"
      export RAICOM_WAIT_HEAD_CAMERA="${RAICOM_WAIT_HEAD_CAMERA:-false}"
    fi
  fi
  "$@" 2>&1 | tee "${LOG_DIR}/${log_name}.log"
  local status=${PIPESTATUS[0]}
  if [[ "${status}" -ne 0 ]]; then
    echo "Task ${task_name} failed with status ${status}; log=${LOG_DIR}/${log_name}.log" >&2
    exit "${status}"
  fi
  if grep -Eq 'Home mission failed|RuntimeError:' "${LOG_DIR}/${log_name}.log"; then
    echo "Task ${task_name} reported mission failure; log=${LOG_DIR}/${log_name}.log" >&2
    exit 1
  fi
  echo "========== DONE ${task_name} $(timestamp) =========="
  wait_home_stack_clear
}

if [[ "${START_NAV}" == "true" ]]; then
  if [[ "${CLEAN_START}" == "true" ]]; then
    clean_previous_sim
  fi
  start_navigation
else
  echo "RAICOM_ALL_START_NAV=false, using existing navigation stack."
fi

reset_start_pose

TASK_INDEX=0
export RAICOM_FIND_FACE_CAMERA_TOPIC="${RAICOM_FIND_FACE_CAMERA_TOPIC:-/head_camera/image_raw}"
export RAICOM_FIND_FACE_USE_USB_CAMERA="${RAICOM_FIND_FACE_USE_USB_CAMERA:-true}"
if [[ "${REUSE_HOME_STACK}" == "true" || "${REUSE_HOME_STACK}" == "1" || "${REUSE_HOME_STACK}" == "yes" ]]; then
  run_task "TASKS national_reuse_all" "tasks_national_reuse_all" \
    rosrun raicom_supervisor start_competition_task.sh national_reuse_all
else
  run_task "TASK 1 national_guide" "task1_national_guide" \
    rosrun raicom_supervisor start_competition_task.sh national_guide

  run_task "TASK 2 national_assistant" "task2_national_assistant" \
    rosrun raicom_supervisor start_competition_task.sh national_assistant

  run_task "TASK 3 national_find" "task3_national_find" \
    rosrun raicom_supervisor start_competition_task.sh national_find

  run_task "TASK 4 national_charge" "task4_national_charge" \
    rosrun raicom_supervisor start_competition_task.sh national_charge
fi

echo
echo "RAICOM national all tasks completed. Logs: ${LOG_DIR}"
if [[ -n "${NAV_PID}" && "${KEEP_NAV_AFTER_ALL}" != "true" ]]; then
  kill "${NAV_PID}" >/dev/null 2>&1 || true
fi
