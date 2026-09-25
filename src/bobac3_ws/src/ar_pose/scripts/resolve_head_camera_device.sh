#!/usr/bin/env bash
set -euo pipefail

log_file="${RAICOM_CAMERA_RESOLVER_LOG:-/tmp/raicom_head_camera_resolver.log}"
stream_test="${RAICOM_HEAD_CAMERA_STREAM_TEST:-true}"
external_pattern="${RAICOM_HEAD_CAMERA_EXTERNAL_PATTERN:-Web Camera|Generic_Web_Camera|USB Camera|USB2.0|UVC Camera}"
front_pattern="${RAICOM_HEAD_CAMERA_FRONT_PATTERN:-HD Webcam|HD_Webcam|Integrated Camera|Integrated_Camera|Front Camera|Front_Camera|Built-in|Internal Camera|Internal_Camera}"
preferred_pattern="${RAICOM_HEAD_CAMERA_PREFERRED_PATTERN:-Camera|Webcam}"
skip_pattern="${RAICOM_HEAD_CAMERA_SKIP_PATTERN:-IR Camera|infrared|metadata|index1}"
exclude_device="${RAICOM_HEAD_CAMERA_EXCLUDE_DEVICE:-}"
preferred_role="${RAICOM_HEAD_CAMERA_PREFERRED_ROLE:-auto}"

log() {
  printf '%s\n' "$*" >>"$log_file" 2>/dev/null || true
}

real_device() {
  local device="$1"
  readlink -f "$device" 2>/dev/null || printf '%s' "$device"
}

device_label() {
  local device="$1"
  local real
  real="$(real_device "$device")"
  local link
  for link in /dev/v4l/by-id/* /dev/v4l/by-path/*; do
    [ -e "$link" ] || continue
    if [ "$(real_device "$link")" = "$real" ]; then
      printf '%s' "$(basename "$link")"
      if command -v v4l2-ctl >/dev/null 2>&1; then
        v4l2-ctl -d "$device" --info 2>/dev/null | awk -F: '/Card type/ {gsub(/^[ \t]+/, "", $2); printf " %s", $2; exit}'
      fi
      return 0
    fi
  done
  printf '%s' "$device"
  if command -v v4l2-ctl >/dev/null 2>&1; then
    v4l2-ctl -d "$device" --info 2>/dev/null | awk -F: '/Card type/ {gsub(/^[ \t]+/, "", $2); printf " %s", $2; exit}'
  fi
}

is_capture_device() {
  local device="$1"
  [ -e "$device" ] || return 1
  if command -v v4l2-ctl >/dev/null 2>&1; then
    v4l2-ctl -d "$device" --all 2>/dev/null | awk '
      /^[[:space:]]*Device Caps/ { in_device_caps = 1; next }
      in_device_caps && /^[[:space:]]*Video Capture/ { found = 1 }
      in_device_caps && /^[^[:space:]]/ { in_device_caps = 0 }
      END { exit found ? 0 : 1 }
    ' || return 1
    v4l2-ctl -d "$device" --list-formats-ext 2>/dev/null | grep -Eq "'(MJPG|YUYV|UYVY|RGB3|BGR3|GREY)'" || return 1
  fi
  return 0
}

can_stream() {
  local device="$1"
  [ "$stream_test" = "true" ] || [ "$stream_test" = "1" ] || return 0
  command -v v4l2-ctl >/dev/null 2>&1 || return 0
  timeout 4 v4l2-ctl -d "$device" --stream-mmap --stream-count=2 --stream-to=/tmp/raicom_head_camera_test.raw >/dev/null 2>&1
}

candidate_score() {
  local device="$1"
  local label="$2"
  local score=50
  if [ -n "$exclude_device" ] && [ "$(real_device "$device")" = "$(real_device "$exclude_device")" ]; then
    echo 999
    return
  fi
  if printf '%s\n' "$label" | grep -Eiq "$skip_pattern"; then
    echo 999
    return
  fi
  if [ -n "${HEAD_CAMERA_DEVICE:-}" ] && [ "$(real_device "$device")" = "$(real_device "$HEAD_CAMERA_DEVICE")" ]; then
    score=0
  elif [ "$preferred_role" = "front" ] && printf '%s\n' "$label" | grep -Eiq "$front_pattern"; then
    score=10
  elif [ "$preferred_role" = "external" ] && printf '%s\n' "$label" | grep -Eiq "$external_pattern"; then
    score=10
  elif [ "$device" = "/dev/head_camera" ]; then
    score=10
  elif printf '%s\n' "$label" | grep -Eiq "$external_pattern"; then
    score=20
  elif printf '%s\n' "$label" | grep -Eiq "$front_pattern"; then
    score=40
  elif printf '%s\n' "$label" | grep -Eiq "$preferred_pattern"; then
    score=60
  fi
  case "$device" in
    *video0) score=$((score + 0)) ;;
    *video1|*video2|*video3) score=$((score + 20)) ;;
  esac
  echo "$score"
}

collect_candidates() {
  {
    [ -n "${HEAD_CAMERA_DEVICE:-}" ] && printf '%s\n' "$HEAD_CAMERA_DEVICE"
    printf '%s\n' /dev/head_camera
    for link in /dev/v4l/by-id/* /dev/v4l/by-path/*; do
      [ -e "$link" ] && printf '%s\n' "$link"
    done
    for device in /dev/video*; do
      [ -e "$device" ] && printf '%s\n' "$device"
    done
  } | awk 'NF && !seen[$0]++'
}

choose_camera() {
  local tmp
  tmp="$(mktemp)"
  local candidate label real score
  while IFS= read -r candidate; do
    [ -e "$candidate" ] || continue
    label="$(device_label "$candidate")"
    real="$(real_device "$candidate")"
    score="$(candidate_score "$candidate" "$label")"
    printf '%s\t%s\t%s\t%s\n' "$score" "$real" "$candidate" "$label" >>"$tmp"
  done < <(collect_candidates)

  while IFS=$'\t' read -r score real candidate label; do
    [ "$score" -lt 999 ] || continue
    log "TRY score=$score device=$candidate real=$real label=$label"
    if is_capture_device "$candidate" && can_stream "$candidate"; then
      rm -f "$tmp" /tmp/raicom_head_camera_test.raw
      log "SELECT $real label=$label"
      printf '%s' "$real"
      return 0
    fi
    log "SKIP unusable device=$candidate real=$real label=$label"
  done < <(sort -n "$tmp")

  rm -f "$tmp" /tmp/raicom_head_camera_test.raw
  return 1
}

if ! choose_camera; then
  fallback="${HEAD_CAMERA_DEVICE:-/dev/video0}"
  log "FALLBACK $fallback"
  printf '%s' "$fallback"
fi
