#!/usr/bin/env bash
set -u

source /opt/ros/noetic/setup.bash
source /home/robot/bobac3_ws/devel/setup.bash
source /home/robot/ros_workspace/devel/setup.bash --extend

mode="${1:-sim}"
if [[ "${mode}" != "sim" && "${mode}" != "real" ]]; then
  echo "用法: provincial_preflight.sh [sim|real]"
  exit 64
fi

failed=0

check_file() {
  local path="$1"
  local label="$2"
  if [[ -f "${path}" ]]; then
    echo "[OK] ${label}: ${path}"
  else
    echo "[FAIL] ${label}: ${path}"
    failed=1
  fi
}

check_package() {
  local name="$1"
  if path="$(rospack find "${name}" 2>/dev/null)"; then
    echo "[OK] ROS功能包 ${name}: ${path}"
  else
    echo "[FAIL] ROS功能包 ${name} 不存在"
    failed=1
  fi
}

echo "=== 省赛赛前检查：${mode} ==="
check_file "/home/robot/bobac3_ws/src/bobac3_navigation/maps/reicom.yaml" "省赛地图配置"
check_file "/home/robot/bobac3_ws/src/bobac3_navigation/maps/reicom.pgm" "省赛地图图像"
check_file "/home/robot/bobac3_ws/src/service_group_mission/config/service_group_waypoints.yaml" "省赛点位"
check_file "/home/robot/.ros/face_encodeing.npz" "人脸编码库"
check_file "/home/robot/ros_workspace/models/sensevoice/model.int8.onnx" "中文语音识别模型"
check_file "/home/robot/ros_workspace/models/melo_tts/model.int8.onnx" "中文语音合成模型"
check_file "/home/robot/ros_workspace/models/raicom_world.pt" "巡检视觉模型"

check_package "bobac3_navigation"
check_package "service_group_mission"
check_package "local_voice_bridge"
check_package "raicom_vision"
check_package "raicom_supervisor"
check_package "face_rec"
check_package "bobac3_description"

if [[ "${mode}" == "real" ]]; then
  check_package "rei_robot_base"
fi

if [[ -e /dev/video0 ]]; then
  echo "[OK] 摄像头: /dev/video0"
else
  echo "[FAIL] 摄像头 /dev/video0 不存在"
  failed=1
fi

expected_source="alsa_input.usb-Generalplus_Usb_Audio_Device-00.mono-fallback"
expected_sink="alsa_output.usb-USB_AUDIO_DAC_USB_AUDIO_DAC-00.analog-stereo"
default_source="$(pactl info | sed -n 's/^Default Source: //p')"
default_sink="$(pactl info | sed -n 's/^Default Sink: //p')"

if [[ "${default_source}" == "${expected_source}" ]]; then
  echo "[OK] 默认麦克风: ${default_source}"
else
  echo "[FAIL] 默认麦克风: ${default_source}"
  echo "       需要: ${expected_source}"
  failed=1
fi

if [[ "${default_sink}" == "${expected_sink}" ]]; then
  echo "[OK] 默认音响: ${default_sink}"
else
  echo "[FAIL] 默认音响: ${default_sink}"
  echo "       需要: ${expected_sink}"
  failed=1
fi

if rosnode list >/dev/null 2>&1; then
  echo "[OK] ROS Master 在线"
  if rostopic list | grep -qx "/map"; then
    echo "[OK] 地图话题 /map"
  else
    echo "[WARN] 地图话题 /map 未启动"
  fi
  if rostopic list | grep -qx "/move_base/status"; then
    echo "[OK] 导航 Action /move_base"
  else
    echo "[WARN] 导航 Action /move_base 未启动"
  fi
else
  echo "[WARN] ROS Master 未启动"
fi

echo "=== 当前设备接口 ==="
ls -l /dev/video* 2>/dev/null || true
ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || true

if [[ "${failed}" -ne 0 ]]; then
  echo "PROVINCIAL_PREFLIGHT_FAILED"
  exit 2
fi

echo "PROVINCIAL_PREFLIGHT_OK"
