#!/usr/bin/env bash
set -euo pipefail

mkdir -p "$HOME/Videos"
output="$HOME/Videos/raicom_$(date +%Y%m%d_%H%M%S)_mic.mp4"

export DISPLAY="${DISPLAY:-:0}"
system_audio="${RAICOM_SYSTEM_AUDIO:-alsa_output.usb-USB_AUDIO_DAC_USB_AUDIO_DAC-00.analog-stereo.monitor}"
mic_audio="${RAICOM_MIC_AUDIO:-$(pactl info | awk -F': ' '/Default Source/ {print $2}')}"
screen_size="${RAICOM_SCREEN_SIZE:-$(xdpyinfo 2>/dev/null | awk '/dimensions:/ && screen_size == "" {screen_size = $2} END {print screen_size}')}"
if [[ -z "${screen_size}" ]]; then
  echo "ERROR: unable to determine X11 screen size for DISPLAY=${DISPLAY}" >&2
  exit 2
fi
screen_width="${screen_size%x*}"
screen_height="${screen_size#*x}"
screen_width=$((screen_width / 2 * 2))
screen_height=$((screen_height / 2 * 2))
capture_size="${screen_width}x${screen_height}"

echo "Recording screen: $DISPLAY+0,0 ($capture_size)"
echo "Recording system audio: $system_audio"
echo "Recording microphone: $mic_audio"
echo "Output: $output"

ffmpeg -y \
  -thread_queue_size 1024 -f x11grab -framerate 30 -video_size "$capture_size" -i "$DISPLAY+0,0" \
  -thread_queue_size 1024 -f pulse -i "$system_audio" \
  -thread_queue_size 1024 -f pulse -i "$mic_audio" \
  -filter_complex "[1:a][2:a]amix=inputs=2:duration=longest[a]" \
  -map 0:v -map "[a]" \
  -vf "scale=1920:1080:force_original_aspect_ratio=decrease,pad=1920:1080:(ow-iw)/2:(oh-ih)/2,fps=30" \
  -c:v libx264 -preset veryfast -crf 23 -pix_fmt yuv420p \
  -c:a aac -b:a 192k -movflags +faststart \
  "$output"

echo "Saved: $output"
