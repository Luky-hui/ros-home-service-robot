#!/usr/bin/env bash
set -euo pipefail

export DISPLAY="${DISPLAY:-:0}"
export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/run/user/$(id -u)}"
if [[ -z "${PULSE_SERVER:-}" && -S "${XDG_RUNTIME_DIR}/pulse/native" ]]; then
  export PULSE_SERVER="unix:${XDG_RUNTIME_DIR}/pulse/native"
fi

mkdir -p "$HOME/Videos"
OUT="${1:-$HOME/Videos/raicom_show_$(date +%Y%m%d_%H%M%S).mp4}"

SCREEN_SIZE="${RAICOM_RECORD_SCREEN_SIZE:-}"
if [[ -z "$SCREEN_SIZE" ]]; then
  SCREEN_SIZE="$(DISPLAY="$DISPLAY" xrandr --current 2>/dev/null | awk '/ connected primary / {for(i=1;i<=NF;i++){if($i ~ /^[0-9]+x[0-9]+\+/){split($i,a,"+"); print a[1]; exit}}} / connected / && !found {for(i=1;i<=NF;i++){if($i ~ /^[0-9]+x[0-9]+\+/){split($i,a,"+"); print a[1]; found=1; exit}}}')"
fi
if [[ -z "$SCREEN_SIZE" ]]; then
  SCREEN_SIZE="$(DISPLAY="$DISPLAY" xdpyinfo 2>/dev/null | awk '/dimensions:/ {print $2; exit}')"
fi
if [[ -z "$SCREEN_SIZE" ]]; then
  echo "ERROR: cannot detect screen size. Set RAICOM_RECORD_SCREEN_SIZE, for example: export RAICOM_RECORD_SCREEN_SIZE=1920x1080" >&2
  exit 2
fi

DEFAULT_SINK="$(pactl info | awk -F': ' '/Default Sink:/ {print $2; exit}')"
SYSTEM_AUDIO="${RAICOM_RECORD_SYSTEM_SOURCE:-${DEFAULT_SINK}.monitor}"
if ! pactl list short sources | awk '{print $2}' | grep -qx "$SYSTEM_AUDIO"; then
  SYSTEM_AUDIO="$(pactl list short sources | awk 'index($2,".monitor")>0 {print $2; exit}')"
fi

MIC_AUDIO="${RAICOM_RECORD_MIC_SOURCE:-}"
if [[ -z "$MIC_AUDIO" ]]; then
  DEFAULT_SOURCE="$(pactl info | awk -F': ' '/Default Source:/ {print $2; exit}')"
  if [[ -n "$DEFAULT_SOURCE" && "$DEFAULT_SOURCE" != *.monitor ]]; then
    MIC_AUDIO="$DEFAULT_SOURCE"
  else
    MIC_AUDIO="$(pactl list short sources | awk 'index($2,".monitor")==0 {print $2; exit}')"
  fi
fi

if [[ -z "$SYSTEM_AUDIO" ]] || ! pactl list short sources | awk '{print $2}' | grep -qx "$SYSTEM_AUDIO"; then
  echo "ERROR: system audio monitor source not found." >&2
  pactl list short sources >&2 || true
  exit 3
fi
RECORD_MIC=true
case "${MIC_AUDIO,,}" in
  none|no|false|0|off)
    RECORD_MIC=false
    MIC_AUDIO=""
    ;;
esac

if [[ "$RECORD_MIC" == "true" ]]; then
  if [[ -z "$MIC_AUDIO" ]] || ! pactl list short sources | awk '{print $2}' | grep -qx "$MIC_AUDIO"; then
    echo "ERROR: microphone source not found." >&2
    pactl list short sources >&2 || true
    exit 4
  fi
fi

FPS="${RAICOM_RECORD_FPS:-30}"
CRF="${RAICOM_RECORD_CRF:-23}"
PRESET="${RAICOM_RECORD_PRESET:-veryfast}"

if [[ "$RECORD_MIC" == "true" ]]; then
  cat <<INFO
Recording screen: ${DISPLAY}.0+0,0 ${SCREEN_SIZE} @ ${FPS}fps
Recording system audio: ${SYSTEM_AUDIO}
Recording microphone: ${MIC_AUDIO}
Output: ${OUT}
Stop: press Ctrl+C in this terminal
INFO

  ffmpeg -y \
    -f x11grab -draw_mouse 1 -framerate "$FPS" -video_size "$SCREEN_SIZE" -i "${DISPLAY}.0+0,0" \
    -f pulse -i "$SYSTEM_AUDIO" \
    -f pulse -i "$MIC_AUDIO" \
    -filter_complex "[1:a]aresample=48000,volume=${RAICOM_RECORD_SYSTEM_VOLUME:-1.0}[sys];[2:a]aresample=48000,volume=${RAICOM_RECORD_MIC_VOLUME:-1.0}[mic];[sys][mic]amix=inputs=2:duration=longest:dropout_transition=2[a]" \
    -map 0:v -map "[a]" \
    -c:v libx264 -preset "$PRESET" -crf "$CRF" -pix_fmt yuv420p \
    -c:a aac -b:a 192k \
    -movflags +faststart \
    "$OUT"
else
  cat <<INFO
Recording screen: ${DISPLAY}.0+0,0 ${SCREEN_SIZE} @ ${FPS}fps
Recording system audio: ${SYSTEM_AUDIO}
Recording microphone: disabled
Output: ${OUT}
Stop: press Ctrl+C in this terminal
INFO

  ffmpeg -y \
    -f x11grab -draw_mouse 1 -framerate "$FPS" -video_size "$SCREEN_SIZE" -i "${DISPLAY}.0+0,0" \
    -f pulse -i "$SYSTEM_AUDIO" \
    -c:v libx264 -preset "$PRESET" -crf "$CRF" -pix_fmt yuv420p \
    -c:a aac -b:a 192k \
    -movflags +faststart \
    "$OUT"
fi
