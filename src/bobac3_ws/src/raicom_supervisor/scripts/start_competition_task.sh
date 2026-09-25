#!/usr/bin/env bash
set -euo pipefail

TASK="${1:-}"
TARGET_OBJECT="${2:-}"

if [[ -z "${TASK}" ]]; then
  echo "Usage: start_competition_task.sh provincial_welcome|provincial_patrol|provincial_patrol_scripted|national_auto|national_reuse_all|national_guide|national_assistant|national_find [cell phone|backpack]|national_charge"
  exit 2
fi

set +u
source /opt/ros/noetic/setup.bash
source /home/robot/bobac3_ws/devel/setup.bash
source /home/robot/ros_workspace/devel/setup.bash --extend
set -u

export RAICOM_HEAD_CAMERA_PREFERRED_ROLE="${RAICOM_HEAD_CAMERA_PREFERRED_ROLE:-front}"

setup_audio() {
  local preferred_source="${RAICOM_AUDIO_SOURCE:-alsa_input.usb-Generic_AB13X_USB_Audio_20230726905926-00.analog-stereo}"
  local preferred_sink="${RAICOM_AUDIO_SINK:-alsa_output.usb-USB_AUDIO_DAC_USB_AUDIO_DAC-00.analog-stereo}"
  local source_volume="${RAICOM_AUDIO_SOURCE_VOLUME:-85%}"
  local sink_volume="${RAICOM_AUDIO_SINK_VOLUME:-100%}"
  local mic_card="${RAICOM_MIC_CARD:-1}"
  local mic_gain="${RAICOM_MIC_GAIN:-20}"
  local selected_source=""
  local selected_sink=""

  export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/run/user/$(id -u)}"
  if [[ -z "${PULSE_SERVER:-}" && -S "${XDG_RUNTIME_DIR}/pulse/native" ]]; then
    export PULSE_SERVER="unix:${XDG_RUNTIME_DIR}/pulse/native"
  fi
  if ! pactl info >/dev/null 2>&1; then
    pulseaudio --start >/dev/null 2>&1 || true
    sleep 1
  fi
  if ! pactl info >/dev/null 2>&1 && [[ -z "${PULSE_SERVER:-}" ]]; then
    local pulse_socket
    pulse_socket="$(find /tmp -maxdepth 2 -type s -path '/tmp/pulse-*/native' 2>/dev/null | head -1 || true)"
    if [[ -n "${pulse_socket}" ]]; then
      export PULSE_SERVER="unix:${pulse_socket}"
    fi
  fi

  if pactl list short sources >/tmp/raicom_sources.$$ 2>/tmp/raicom_sources_err.$$; then
    if awk '{print $2}' /tmp/raicom_sources.$$ | grep -qx "${preferred_source}"; then
      selected_source="${preferred_source}"
    else
      selected_source="$(pactl get-default-source 2>/dev/null || true)"
      if [[ "${selected_source}" == *.monitor ]] || \
         ! awk '{print $2}' /tmp/raicom_sources.$$ | grep -qx "${selected_source}"; then
        selected_source="$(awk 'index($2, ".monitor")==0 {print $2; exit}' /tmp/raicom_sources.$$)"
      fi
      echo "WARN audio source ${preferred_source} not found; using ${selected_source:-none}" >&2
    fi
    if [[ -n "${selected_source}" ]]; then
      pactl set-default-source "${selected_source}" || true
      pactl set-source-volume "${selected_source}" "${source_volume}" || true
      export PULSE_SOURCE="${selected_source}"
    fi
  else
    echo "WARN pactl source list failed; audio input was not configured." >&2
    cat /tmp/raicom_sources_err.$$ >&2 || true
  fi
  rm -f /tmp/raicom_sources.$$ /tmp/raicom_sources_err.$$

  if pactl list short sinks >/tmp/raicom_sinks.$$ 2>/tmp/raicom_sinks_err.$$; then
    if awk '{print $2}' /tmp/raicom_sinks.$$ | grep -qx "${preferred_sink}"; then
      selected_sink="${preferred_sink}"
    else
      selected_sink="$(pactl get-default-sink 2>/dev/null || true)"
      if ! awk '{print $2}' /tmp/raicom_sinks.$$ | grep -qx "${selected_sink}"; then
        selected_sink="$(awk '{print $2; exit}' /tmp/raicom_sinks.$$)"
      fi
      echo "WARN audio sink ${preferred_sink} not found; using ${selected_sink:-none}" >&2
    fi
    if [[ -n "${selected_sink}" ]]; then
      pactl set-default-sink "${selected_sink}" || true
      pactl set-sink-volume "${selected_sink}" "${sink_volume}" || true
    fi
  else
    echo "WARN pactl sink list failed; audio output was not configured." >&2
    cat /tmp/raicom_sinks_err.$$ >&2 || true
  fi
  rm -f /tmp/raicom_sinks.$$ /tmp/raicom_sinks_err.$$

  amixer -c "${mic_card}" sset Mic "${mic_gain}" cap >/dev/null 2>&1 || \
    echo "WARN amixer mic setup failed for card ${mic_card}; continuing." >&2
}

if [[ "${RAICOM_SKIP_AUDIO_SETUP:-false}" == "true" || "${RAICOM_SKIP_AUDIO_SETUP:-false}" == "1" || "${RAICOM_SKIP_AUDIO_SETUP:-false}" == "yes" ]]; then
  echo "Audio setup skipped by RAICOM_SKIP_AUDIO_SETUP=${RAICOM_SKIP_AUDIO_SETUP}"
else
  setup_audio
fi

if [[ -n "${RAICOM_CHARGE_AR_SIM:-}" ]]; then
  CHARGE_AR_SIM="${RAICOM_CHARGE_AR_SIM}"
elif [[ "${REI_ROBOT:-}" == "bobac3" ]]; then
  CHARGE_AR_SIM="false"
else
  CHARGE_AR_SIM="true"
fi
ENABLE_CHARGE_AR="${RAICOM_ENABLE_CHARGE_AR:-true}"

if [[ -n "${RAICOM_FACE_MODE:-}" ]]; then
  FACE_MODE="${RAICOM_FACE_MODE}"
elif [[ "${REI_ROBOT:-}" == "bobac3" ]]; then
  FACE_MODE="any"
else
  FACE_MODE="sim"
fi


require_navigation_ready() {
  echo 'Checking navigation stack readiness...'
  local fast_ready="${RAICOM_FAST_NAV_READY:-false}"
  if [[ "${fast_ready}" == "true" || "${fast_ready}" == "1" || "${fast_ready}" == "yes" ]]; then
    if ! timeout 3 rostopic list >/tmp/raicom_topics.$$ 2>/tmp/raicom_topics_err.$$; then
      echo 'ERROR: ROS master is not reachable during fast navigation readiness check.' >&2
      rm -f /tmp/raicom_topics.$$ /tmp/raicom_topics_err.$$
      exit 3
    fi
    if ! grep -qx '/map' /tmp/raicom_topics.$$ || ! grep -qx '/move_base/status' /tmp/raicom_topics.$$; then
      echo 'ERROR: /map or /move_base/status is not available during fast navigation readiness check.' >&2
      rm -f /tmp/raicom_topics.$$ /tmp/raicom_topics_err.$$
      exit 3
    fi
    rm -f /tmp/raicom_topics.$$ /tmp/raicom_topics_err.$$
    if ! timeout 3 rostopic echo -n 1 /map/header >/dev/null 2>&1; then
      echo 'ERROR: /map exists but no map message was received during fast readiness check.' >&2
      exit 3
    fi
    if ! timeout 5 python3 - <<'PY_FAST_TF_CHECK' >/tmp/raicom_tf.$$ 2>&1
import rospy
import tf
rospy.init_node('raicom_fast_nav_ready_tf_check', anonymous=True, disable_signals=True)
listener = tf.TransformListener()
rospy.sleep(0.3)
listener.waitForTransform('map', 'base_footprint', rospy.Time(0), rospy.Duration(2.0))
translation, rotation = listener.lookupTransform('map', 'base_footprint', rospy.Time(0))
print('TF_OK map base_footprint %.3f %.3f %.3f' % (translation[0], translation[1], translation[2]))
PY_FAST_TF_CHECK
    then
      echo 'ERROR: TF map -> base_footprint is not ready during fast readiness check.' >&2
      cat /tmp/raicom_tf.$$ >&2 || true
      rm -f /tmp/raicom_tf.$$
      exit 3
    fi
    cat /tmp/raicom_tf.$$ || true
    rm -f /tmp/raicom_tf.$$
    echo 'Navigation stack is fast-ready.'
    return 0
  fi
  if ! timeout 3 rostopic list >/tmp/raicom_topics.$$ 2>/tmp/raicom_topics_err.$$; then
    echo 'ERROR: ROS master is not reachable. Please start navigation first:' >&2
    echo '  roslaunch bobac3_navigation demo_nav_2d.launch open_nav_rviz:=true gazebo_gui:=true map_file_name:=national_2026' >&2
    rm -f /tmp/raicom_topics.$$ /tmp/raicom_topics_err.$$
    exit 3
  fi

  if ! grep -qx '/map' /tmp/raicom_topics.$$; then
    echo 'ERROR: /map is not available. Start navigation first and wait for map_server.' >&2
    echo 'Command:' >&2
    echo '  roslaunch bobac3_navigation demo_nav_2d.launch open_nav_rviz:=true gazebo_gui:=true map_file_name:=national_2026' >&2
    rm -f /tmp/raicom_topics.$$ /tmp/raicom_topics_err.$$
    exit 3
  fi

  if ! grep -qx '/move_base/status' /tmp/raicom_topics.$$; then
    echo 'ERROR: /move_base/status is not available. move_base is not ready.' >&2
    rm -f /tmp/raicom_topics.$$ /tmp/raicom_topics_err.$$
    exit 3
  fi
  rm -f /tmp/raicom_topics.$$ /tmp/raicom_topics_err.$$

  if ! timeout 5 rostopic echo -n 1 /map/header >/dev/null 2>&1; then
    echo 'ERROR: /map exists but no map message was received within 5 seconds.' >&2
    exit 3
  fi

  local tf_ready="false"
  for _ in $(seq 1 45); do
    if timeout 5 python3 - <<'PY_TF_CHECK' >/tmp/raicom_tf.$$ 2>&1
import rospy
import tf
rospy.init_node('raicom_nav_ready_tf_check', anonymous=True, disable_signals=True)
listener = tf.TransformListener()
rospy.sleep(0.5)
listener.waitForTransform('map', 'base_footprint', rospy.Time(0), rospy.Duration(3.0))
translation, rotation = listener.lookupTransform('map', 'base_footprint', rospy.Time(0))
print('TF_OK map base_footprint %.3f %.3f %.3f' % (translation[0], translation[1], translation[2]))
PY_TF_CHECK
    then
      tf_ready="true"
      break
    fi
    sleep 2
  done

  if [[ "${tf_ready}" != "true" ]]; then
    echo 'ERROR: TF map -> base_footprint is not ready. Wait until AMCL/localization is ready.' >&2
    cat /tmp/raicom_tf.$$ >&2 || true
    rm -f /tmp/raicom_tf.$$
    exit 3
  fi
  cat /tmp/raicom_tf.$$ || true
  rm -f /tmp/raicom_tf.$$

  if ! timeout 130 python3 - <<'PY_ACTION_CHECK' >/tmp/raicom_action.$$ 2>&1
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction
rospy.init_node('raicom_nav_ready_action_check', anonymous=True, disable_signals=True)
client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
if not client.wait_for_server(rospy.Duration(120.0)):
    raise RuntimeError('move_base action server /move_base is not ready')
print('ACTION_OK /move_base')
PY_ACTION_CHECK
  then
    echo 'WARN: move_base action server /move_base did not complete actionlib handshake.' >&2
    cat /tmp/raicom_action.$$ >&2 || true
    rm -f /tmp/raicom_action.$$
    echo 'Checking whether /move_base_node and /move_base/status are still alive...' >&2
    if timeout 10 rosnode ping -c 1 /move_base_node >/tmp/raicom_move_base_ping.$$ 2>&1 && \
       timeout 10 rostopic echo -n 1 /move_base/status >/tmp/raicom_move_base_status.$$ 2>&1; then
      echo 'WARN: /move_base_node is alive and /move_base/status is publishing; continuing.' >&2
      rm -f /tmp/raicom_move_base_ping.$$ /tmp/raicom_move_base_status.$$
    else
      echo 'ERROR: move_base is not healthy enough to continue.' >&2
      cat /tmp/raicom_move_base_ping.$$ >&2 || true
      cat /tmp/raicom_move_base_status.$$ >&2 || true
      rm -f /tmp/raicom_move_base_ping.$$ /tmp/raicom_move_base_status.$$
      exit 3
    fi
  else
    cat /tmp/raicom_action.$$ || true
    rm -f /tmp/raicom_action.$$
  fi
  echo 'Navigation stack is ready.'
}

for node in \
  /raicom_supervisor \
  /raicom_vision \
  /raicom_vision_find \
  /local_voice_node \
  /face_detection \
  /face_rec_service \
  /head_camera \
  /service_group_welcome_mission \
  /service_group_patrol_mission \
  /home_service_mission \
  /national_assistant_vision_monitor \
  /national_find_vision_monitor
do
  rosnode kill "${node}" >/dev/null 2>&1 || true
done

ASSISTANT_VISION_MONITOR_PID=""
FIND_VISION_MONITOR_PID=""

start_assistant_vision_monitor() {
  local enabled="${RAICOM_ASSISTANT_VISION_MONITOR:-true}"
  if [[ "${enabled}" != "true" && "${enabled}" != "1" && "${enabled}" != "yes" ]]; then
    echo "Assistant vision monitor disabled by RAICOM_ASSISTANT_VISION_MONITOR=${enabled}"
    return
  fi
  export DISPLAY="${DISPLAY:-:0}"
  local monitor_config
  monitor_config="$(rospack find home_service_mission)/config/national_home_waypoints.yaml"
  local monitor_log="/tmp/raicom_assistant_vision_monitor.log"
  local args=(--config "${monitor_config}" --rate "${RAICOM_ASSISTANT_VISION_RATE:-0.5}" --show-on-start)
  if [[ -n "${RAICOM_ASSISTANT_VISION_TOPIC:-}" ]]; then
    args+=(--topic "${RAICOM_ASSISTANT_VISION_TOPIC}")
  fi
  if [[ -n "${RAICOM_ASSISTANT_VISION_IMAGE_PATH:-}" ]]; then
    args+=(--image-path "${RAICOM_ASSISTANT_VISION_IMAGE_PATH}")
  fi
  if [[ -n "${RAICOM_ASSISTANT_VISION_CONFIDENCE:-}" ]]; then
    args+=(--confidence "${RAICOM_ASSISTANT_VISION_CONFIDENCE}")
  fi
  rosrun raicom_supervisor watch_national_assistant_vision_gui.py "${args[@]}" >"${monitor_log}" 2>&1 &
  ASSISTANT_VISION_MONITOR_PID="$!"
  echo "Assistant vision monitor started: ${monitor_log} pid=${ASSISTANT_VISION_MONITOR_PID}"
}

cleanup_assistant_vision_monitor() {
  if [[ -n "${ASSISTANT_VISION_MONITOR_PID}" ]]; then
    kill "${ASSISTANT_VISION_MONITOR_PID}" >/dev/null 2>&1 || true
    wait "${ASSISTANT_VISION_MONITOR_PID}" >/dev/null 2>&1 || true
    ASSISTANT_VISION_MONITOR_PID=""
  fi
  rosnode kill /national_assistant_vision_monitor >/dev/null 2>&1 || true
}

start_find_vision_monitor() {
  local enabled="${RAICOM_FIND_VISION_MONITOR:-true}"
  if [[ "${enabled}" != "true" && "${enabled}" != "1" && "${enabled}" != "yes" ]]; then
    echo "Find vision monitor disabled by RAICOM_FIND_VISION_MONITOR=${enabled}"
    return
  fi
  export DISPLAY="${DISPLAY:-:0}"
  local monitor_config
  monitor_config="$(rospack find home_service_mission)/config/national_home_waypoints.yaml"
  local monitor_log="/tmp/raicom_find_vision_monitor.log"
  local args=(
    --config "${monitor_config}"
    --mode find
    --service "${RAICOM_FIND_VISION_SERVICE:-/raicom_vision_find/detect}"
    --annotated-topic "${RAICOM_FIND_VISION_ANNOTATED_TOPIC:-/raicom_vision_find/annotated}"
    --window-name "RAICOM Task3 Vision Monitor"
    --show-on-start
    --node-name national_find_vision_monitor
    --rate "${RAICOM_FIND_VISION_RATE:-0.5}"
  )
  if [[ -n "${RAICOM_FIND_VISION_TOPIC:-}" ]]; then
    args+=(--topic "${RAICOM_FIND_VISION_TOPIC}")
  fi
  if [[ -n "${RAICOM_FIND_VISION_LABELS:-}" ]]; then
    args+=(--labels "${RAICOM_FIND_VISION_LABELS}")
  fi
  if [[ -n "${RAICOM_FIND_VISION_CONFIDENCE:-}" ]]; then
    args+=(--confidence "${RAICOM_FIND_VISION_CONFIDENCE}")
  fi
  rosrun raicom_supervisor watch_national_assistant_vision_gui.py "${args[@]}" >"${monitor_log}" 2>&1 &
  FIND_VISION_MONITOR_PID="$!"
  echo "Find vision monitor started: ${monitor_log} pid=${FIND_VISION_MONITOR_PID}"
}

cleanup_find_vision_monitor() {
  if [[ -n "${FIND_VISION_MONITOR_PID}" ]]; then
    kill "${FIND_VISION_MONITOR_PID}" >/dev/null 2>&1 || true
    wait "${FIND_VISION_MONITOR_PID}" >/dev/null 2>&1 || true
    FIND_VISION_MONITOR_PID=""
  fi
  rosnode kill /national_find_vision_monitor >/dev/null 2>&1 || true
}

case "${TASK}" in
  provincial_welcome|selection_welcome)
    require_navigation_ready
    exec roslaunch raicom_supervisor selection_welcome_stack.launch
    ;;
  provincial_patrol|selection_patrol)
    require_navigation_ready
    exec roslaunch raicom_supervisor selection_patrol_stack.launch
    ;;
  provincial_patrol_scripted|selection_patrol_scripted)
    require_navigation_ready
    exec roslaunch raicom_supervisor selection_patrol_stack.launch       config_file:="$(rospack find service_group_mission)/config/provincial_patrol_test.yaml"
    ;;
  home_auto)
    rosrun home_service_mission home_config_check.py
    exec roslaunch raicom_supervisor home_stack.launch mission_type:=auto
    ;;
  home_guide)
    rosrun home_service_mission home_config_check.py
    exec roslaunch raicom_supervisor home_stack.launch mission_type:=guide
    ;;
  home_assistant)
    rosrun home_service_mission home_config_check.py
    exec roslaunch raicom_supervisor home_stack.launch mission_type:=assistant
    ;;
  home_find)
    if [[ "${TARGET_OBJECT}" != "cell phone" && "${TARGET_OBJECT}" != "backpack" ]]; then
      echo "home_find target must be: cell phone or backpack"
      exit 2
    fi
    rosrun home_service_mission home_config_check.py
    exec roslaunch raicom_supervisor home_stack.launch \
      mission_type:=find_object \
      target_object:="${TARGET_OBJECT}"
    ;;
  national_auto)
    require_navigation_ready
    rosrun home_service_mission home_config_check.py \
      _config_file:="$(rospack find home_service_mission)/config/national_home_waypoints.yaml"
    exec roslaunch raicom_supervisor home_stack.launch \
      mission_type:=auto \
      face_mode:="${FACE_MODE}" \
      speak_unknown_fallback:="${RAICOM_SPEAK_UNKNOWN_FALLBACK:-true}" \
      semantic_reply_cooldown:="${RAICOM_SEMANTIC_REPLY_COOLDOWN:-6.0}" \
      enable_charge_ar:="${ENABLE_CHARGE_AR}" \
      charge_ar_sim:="${CHARGE_AR_SIM}" \
      config_file:="$(rospack find home_service_mission)/config/national_home_waypoints.yaml" \
      vision_config_file:="$(rospack find raicom_vision)/config/vision_raicom16.yaml"
    ;;
  national_reuse_all|national_sequence)
    require_navigation_ready
    rosrun home_service_mission home_config_check.py \
      _config_file:="$(rospack find home_service_mission)/config/national_home_waypoints.yaml"
    start_assistant_vision_monitor
    start_find_vision_monitor
    trap 'cleanup_assistant_vision_monitor; cleanup_find_vision_monitor' EXIT INT TERM
    roslaunch raicom_supervisor home_stack.launch \
      mission_type:=national_sequence \
      face_mode:="${FACE_MODE}" \
      target_object:="${TARGET_OBJECT}" \
      require_fresh_commands:=true \
      speak_unknown_fallback:="${RAICOM_SPEAK_UNKNOWN_FALLBACK:-true}" \
      semantic_reply_cooldown:="${RAICOM_SEMANTIC_REPLY_COOLDOWN:-6.0}" \
      face_camera_topic:="${RAICOM_FIND_FACE_CAMERA_TOPIC:-/head_camera/image_raw}" \
      face_use_usb_camera:="${RAICOM_FIND_FACE_USE_USB_CAMERA:-true}" \
      head_camera_watchdog:="${RAICOM_HEAD_CAMERA_WATCHDOG:-true}" \
      enable_charge_ar:="${ENABLE_CHARGE_AR}" \
      charge_ar_sim:="${CHARGE_AR_SIM}" \
      charge_ar_image_topic:="${RAICOM_CHARGE_AR_IMAGE_TOPIC:-/bottom_camera/image_raw}" \
      charge_ar_camera_info_topic:="${RAICOM_CHARGE_AR_CAMERA_INFO_TOPIC:-/bottom_camera/camera_info}" \
      config_file:="$(rospack find home_service_mission)/config/national_home_waypoints.yaml" \
      vision_config_file:="$(rospack find raicom_vision)/config/vision_raicom16.yaml" \
      enable_find_vision:=true \
      vision_find_config_file:="$(rospack find raicom_vision)/config/vision_phone_bag_v5_find_service.yaml" \
      find_vision_service:=/raicom_vision_find/detect
    status=$?
    cleanup_assistant_vision_monitor
    cleanup_find_vision_monitor
    trap - EXIT INT TERM
    exit "${status}"
    ;;
  national_guide)
    require_navigation_ready
    rosrun home_service_mission home_config_check.py \
      _config_file:="$(rospack find home_service_mission)/config/national_home_waypoints.yaml"
    exec roslaunch raicom_supervisor home_stack.launch \
      mission_type:=guide \
      face_mode:="${FACE_MODE}" \
      speak_unknown_fallback:="${RAICOM_SPEAK_UNKNOWN_FALLBACK:-true}" \
      semantic_reply_cooldown:="${RAICOM_SEMANTIC_REPLY_COOLDOWN:-6.0}" \
      enable_charge_ar:="${ENABLE_CHARGE_AR}" \
      charge_ar_sim:="${CHARGE_AR_SIM}" \
      config_file:="$(rospack find home_service_mission)/config/national_home_waypoints.yaml" \
      vision_config_file:="$(rospack find raicom_vision)/config/vision_raicom16.yaml"
    ;;
  national_assistant)
    require_navigation_ready
    rosrun home_service_mission home_config_check.py \
      _config_file:="$(rospack find home_service_mission)/config/national_home_waypoints.yaml"
    start_assistant_vision_monitor
    trap cleanup_assistant_vision_monitor EXIT INT TERM
    roslaunch raicom_supervisor home_stack.launch \
      mission_type:=assistant \
      face_mode:="${FACE_MODE}" \
      speak_unknown_fallback:="${RAICOM_SPEAK_UNKNOWN_FALLBACK:-true}" \
      semantic_reply_cooldown:="${RAICOM_SEMANTIC_REPLY_COOLDOWN:-6.0}" \
      enable_charge_ar:="${ENABLE_CHARGE_AR}" \
      charge_ar_sim:="${CHARGE_AR_SIM}" \
      config_file:="$(rospack find home_service_mission)/config/national_home_waypoints.yaml" \
      vision_config_file:="$(rospack find raicom_vision)/config/vision_raicom16.yaml"
    status=$?
    cleanup_assistant_vision_monitor
    trap - EXIT INT TERM
    exit "${status}"
    ;;
  national_find)
    if [[ -n "${TARGET_OBJECT}" && "${TARGET_OBJECT}" != "cell phone" && "${TARGET_OBJECT}" != "backpack" ]]; then
      echo "national_find target must be empty, cell phone, or backpack"
      exit 2
    fi
    require_navigation_ready
    rosrun home_service_mission home_config_check.py \
      _config_file:="$(rospack find home_service_mission)/config/national_home_waypoints.yaml"
    start_find_vision_monitor
    trap cleanup_find_vision_monitor EXIT INT TERM
    roslaunch raicom_supervisor home_stack.launch \
      mission_type:=find_object \
      face_mode:="${FACE_MODE}" \
      target_object:="${TARGET_OBJECT}" \
      speak_unknown_fallback:="${RAICOM_SPEAK_UNKNOWN_FALLBACK:-true}" \
      semantic_reply_cooldown:="${RAICOM_SEMANTIC_REPLY_COOLDOWN:-6.0}" \
      face_camera_topic:="${RAICOM_FIND_FACE_CAMERA_TOPIC:-/berxel_base/color/image_raw}" \
      face_use_usb_camera:="${RAICOM_FIND_FACE_USE_USB_CAMERA:-false}" \
      enable_charge_ar:="${ENABLE_CHARGE_AR}" \
      charge_ar_sim:="${CHARGE_AR_SIM}" \
      config_file:="$(rospack find home_service_mission)/config/national_home_waypoints.yaml" \
      vision_config_file:="$(rospack find raicom_vision)/config/vision_phone_bag_v5.yaml"
    status=$?
    cleanup_find_vision_monitor
    trap - EXIT INT TERM
    exit "${status}"
    ;;
  national_charge)
    require_navigation_ready
    rosrun home_service_mission home_config_check.py \
      _config_file:="$(rospack find home_service_mission)/config/national_home_waypoints.yaml"
    exec roslaunch raicom_supervisor home_stack.launch \
      mission_type:=charge \
      face_mode:="${FACE_MODE}" \
      speak_unknown_fallback:="${RAICOM_SPEAK_UNKNOWN_FALLBACK:-true}" \
      semantic_reply_cooldown:="${RAICOM_SEMANTIC_REPLY_COOLDOWN:-6.0}" \
      enable_charge_ar:="${ENABLE_CHARGE_AR}" \
      charge_ar_sim:="${CHARGE_AR_SIM}" \
      charge_ar_image_topic:="${RAICOM_CHARGE_AR_IMAGE_TOPIC:-/bottom_camera/image_raw}" \
      charge_ar_camera_info_topic:="${RAICOM_CHARGE_AR_CAMERA_INFO_TOPIC:-/bottom_camera/camera_info}" \
      config_file:="$(rospack find home_service_mission)/config/national_home_waypoints.yaml" \
      vision_config_file:="$(rospack find raicom_vision)/config/vision_raicom16.yaml"
    ;;
  *)
    echo "Unknown task: ${TASK}"
    exit 2
    ;;
esac
