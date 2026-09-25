#!/usr/bin/env bash
set -euo pipefail

COMMAND_TOPIC="/service_group_mission/voice_command"
DEMO_LAUNCH_PID=""
VOICE_LAUNCH_PID=""

cn() {
  python3 - "$1" <<'PY'
import sys
print(sys.argv[1].encode("utf-8").decode("unicode_escape"))
PY
}

source_ros() {
  source /opt/ros/noetic/setup.bash
  source /home/robot/bobac3_ws/devel/setup.bash
  source /home/robot/ros_workspace/devel/setup.bash --extend
}

wait_for_master() {
  echo "Checking ROS master..."
  until rostopic list >/dev/null 2>&1; do
    echo "ROS master is not ready. Start Gazebo + navigation + RViz first."
    sleep 2
  done
}

wait_for_command_subscriber() {
  echo "Waiting for subscriber on ${COMMAND_TOPIC}..."
  until rostopic info "${COMMAND_TOPIC}" 2>/dev/null | grep -Eq "/service_group_(welcome|patrol)_mission"; do
    sleep 1
  done
  sleep 1
}

start_voice_bridge() {
  if rosservice list 2>/dev/null | grep -qx "/voice_tts"; then
    echo "voice_tts service already exists."
    return
  fi

  echo "Start local voice bridge for TTS..."
  roslaunch local_voice_bridge local_voice.launch continuous_listen:=false execute_intents:=false &
  VOICE_LAUNCH_PID=$!

  echo "Waiting for /voice_tts..."
  until rosservice list 2>/dev/null | grep -qx "/voice_tts"; do
    sleep 1
  done
  sleep 1
}

publish_command() {
  local text="$1"
  echo "Publish command: ${text}"
  rostopic pub -1 "${COMMAND_TOPIC}" std_msgs/String "data: '${text}'" >/dev/null
}

publish_escaped_command() {
  local escaped_text="$1"
  python3 - "${COMMAND_TOPIC}" "${escaped_text}" <<'PY'
import sys
import time
import rospy
from std_msgs.msg import String

topic = sys.argv[1]
escaped_text = sys.argv[2]
text = escaped_text.encode("ascii").decode("unicode_escape")

rospy.init_node("raicom_keyboard_demo_pub", anonymous=True)
pub = rospy.Publisher(topic, String, queue_size=1, latch=True)
deadline = time.time() + 3.0
rate = rospy.Rate(10)
while pub.get_num_connections() == 0 and time.time() < deadline and not rospy.is_shutdown():
    rate.sleep()
for _ in range(5):
    pub.publish(String(text))
    rate.sleep()
PY
}

stop_demo_nodes() {
  rosnode kill /service_group_welcome_mission >/dev/null 2>&1 || true
  rosnode kill /service_group_patrol_mission >/dev/null 2>&1 || true
  if [[ -n "${DEMO_LAUNCH_PID:-}" ]]; then
    kill "${DEMO_LAUNCH_PID}" >/dev/null 2>&1 || true
    wait "${DEMO_LAUNCH_PID}" >/dev/null 2>&1 || true
    DEMO_LAUNCH_PID=""
  fi
  if [[ -n "${VOICE_LAUNCH_PID:-}" ]]; then
    kill "${VOICE_LAUNCH_PID}" >/dev/null 2>&1 || true
    wait "${VOICE_LAUNCH_PID}" >/dev/null 2>&1 || true
    VOICE_LAUNCH_PID=""
  fi
}

start_welcome() {
  stop_demo_nodes
  start_voice_bridge
  echo "Start task 1: welcome guide. Face input is skipped. Voice input is keyboard text."
  roslaunch service_group_mission welcome_mission.launch face_mode:=skip voice_mode:=topic &
  DEMO_LAUNCH_PID=$!
  wait_for_command_subscriber

  echo
  echo "Input venue: shenzhen / guangzhou / shanghai / jilin / beijing"
  read -r venue
  case "${venue}" in
    shenzhen|sz|1) publish_escaped_command '\u53ef\u4ee5\u5e26\u6211\u53bb\u53c2\u89c2\u4e00\u4e0b\u6df1\u5733\u9986\u5417\uff1f' ;;
    guangzhou|gz|2) publish_escaped_command '\u53ef\u4ee5\u5e26\u6211\u53bb\u53c2\u89c2\u4e00\u4e0b\u5e7f\u5dde\u9986\u5417\uff1f' ;;
    shanghai|sh|3) publish_escaped_command '\u53ef\u4ee5\u5e26\u6211\u53bb\u53c2\u89c2\u4e00\u4e0b\u4e0a\u6d77\u9986\u5417\uff1f' ;;
    jilin|jl|4) publish_escaped_command '\u53ef\u4ee5\u5e26\u6211\u53bb\u53c2\u89c2\u4e00\u4e0b\u5409\u6797\u9986\u5417\uff1f' ;;
    beijing|bj|5) publish_escaped_command '\u53ef\u4ee5\u5e26\u6211\u53bb\u53c2\u89c2\u4e00\u4e0b\u5317\u4eac\u9986\u5417\uff1f' ;;
    *) publish_command "${venue}" ;;
  esac

  echo "Task 1 command sent. Press Enter after the robot finishes, then this script returns to menu."
  read -r _
  stop_demo_nodes
}

start_patrol() {
  stop_demo_nodes
  start_voice_bridge
  echo "Start task 2: patrol. Admin face input is skipped. Voice input is keyboard text."
  roslaunch service_group_mission patrol_mission.launch face_mode:=skip voice_mode:=topic &
  DEMO_LAUNCH_PID=$!
  wait_for_command_subscriber

  echo
  echo "Press Enter to publish: start patrol mission"
  read -r _
  publish_escaped_command '\u5f00\u59cb\u6267\u884c\u5de1\u68c0\u4efb\u52a1'

  echo "Task 2 command sent. Press Enter after the robot finishes, then this script returns to menu."
  read -r _
  stop_demo_nodes
}

main() {
  source_ros
  wait_for_master

  while true; do
    echo
    echo "=============================="
    echo "RAICOM keyboard demo console"
    echo "1) Task 1: welcome guide"
    echo "2) Task 2: patrol"
    echo "q) Quit"
    echo "=============================="
    read -rp "Select: " choice

    case "${choice}" in
      1) start_welcome ;;
      2) start_patrol ;;
      q|Q) stop_demo_nodes; exit 0 ;;
      *) echo "Input 1, 2, or q." ;;
    esac
  done
}

trap stop_demo_nodes EXIT
main "$@"
