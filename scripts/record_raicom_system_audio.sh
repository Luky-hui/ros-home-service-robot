#!/usr/bin/env bash
set -euo pipefail

export DISPLAY="${DISPLAY:-:0}"
export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/run/user/$(id -u)}"

mkdir -p "$HOME/Videos"
OUT="${1:-$HOME/Videos/raicom_system_$(date +%Y%m%d_%H%M%S).mp4}"

SCREEN_SIZE="$(DISPLAY="$DISPLAY" xrandr --current 2>/dev/null | awk '/ connected primary / {for(i=1;i<=NF;i++){if($i ~ /^[0-9]+x[0-9]+\+/){split($i,a,"+"); print a[1]; exit}}}')"
if [[ -z "$SCREEN_SIZE" ]]; then
  SCREEN_SIZE="$(DISPLAY="$DISPLAY" xdpyinfo 2>/dev/null | awk '/dimensions:/ {print $2; exit}')"
fi

DEFAULT_SINK="$(pactl info | awk -F': ' '/Default Sink:/ {print $2; exit}')"
SYSTEM_AUDIO="${DEFAULT_SINK}.monitor"

echo "Recording screen: $SCREEN_SIZE"
echo "Recording system audio: $SYSTEM_AUDIO"
echo "Output: $OUT"
echo "Stop: Ctrl+C"

ffmpeg -y \
  -f x11grab -draw_mouse 1 -framerate 30 -video_size "$SCREEN_SIZE" -i "${DISPLAY}.0+0,0" \
  -f pulse -i "$SYSTEM_AUDIO" \
  -c:v libx264 -preset veryfast -crf 23 -pix_fmt yuv420p \
  -c:a aac -b:a 160k \
  -movflags +faststart \
  "$OUT"
