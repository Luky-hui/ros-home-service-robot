#!/usr/bin/env bash
set -e

source /opt/ros/noetic/setup.bash
source /home/robot/bobac3_ws/devel/setup.bash
source /home/robot/ros_workspace/devel/setup.bash --extend

pactl set-default-source alsa_input.usb-Generalplus_Usb_Audio_Device-00.mono-fallback
pactl set-source-mute alsa_input.usb-Generalplus_Usb_Audio_Device-00.mono-fallback 0
pactl set-source-volume alsa_input.usb-Generalplus_Usb_Audio_Device-00.mono-fallback 150%
pactl set-default-sink alsa_output.usb-USB_AUDIO_DAC_USB_AUDIO_DAC-00.analog-stereo

for node in \
  /voice_aiui_node \
  /voice_awake \
  /voice_collect_node \
  /voice_interaction \
  /voice_control \
  /voice_nav \
  /local_voice_node
do
  if rosnode list 2>/dev/null | grep -Fxq "$node"; then
    rosnode kill "$node" >/dev/null
  fi
done

exec roslaunch local_voice_bridge lessons_6_10.launch \
  continuous_listen:=true \
  execute_intents:=true \
  start_control:=true \
  start_navigation:=true
