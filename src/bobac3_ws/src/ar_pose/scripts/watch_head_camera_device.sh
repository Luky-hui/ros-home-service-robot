#!/usr/bin/env bash
set -euo pipefail

interval="${RAICOM_HEAD_CAMERA_WATCH_INTERVAL:-4}"
resolver="${RAICOM_HEAD_CAMERA_RESOLVER:-/home/robot/bobac3_ws/src/ar_pose/scripts/resolve_head_camera_device.sh}"
camera_node="${RAICOM_HEAD_CAMERA_NODE:-/head_camera}"
video_param="${RAICOM_HEAD_CAMERA_VIDEO_PARAM:-/head_camera/video_device}"
image_topic="${RAICOM_HEAD_CAMERA_IMAGE_TOPIC:-/head_camera/image_raw}"
miss_limit="${RAICOM_HEAD_CAMERA_MISS_LIMIT:-4}"
restore_confirm="${RAICOM_HEAD_CAMERA_RESTORE_CONFIRM:-4}"
restart_cooldown="${RAICOM_HEAD_CAMERA_RESTART_COOLDOWN:-20}"

timestamp() {
  date '+%H:%M:%S'
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
      basename "$link"
      return 0
    fi
  done
  if command -v v4l2-ctl >/dev/null 2>&1; then
    local card
    card="$(v4l2-ctl -d "$real" --info 2>/dev/null | awk -F: '/Card type/ {sub(/^[ \t]+/, "", $2); print $2; exit}')"
    if [ -n "$card" ]; then
      printf '%s' "$card"
      return 0
    fi
  fi
  printf '%s' "$device"
}

device_role() {
  local label="$1"
  if printf '%s\n' "$label" | grep -Eiq "${RAICOM_HEAD_CAMERA_EXTERNAL_PATTERN:-Web Camera|Generic_Web_Camera|USB Camera|USB2.0|UVC Camera}"; then
    printf 'external'
  elif printf '%s\n' "$label" | grep -Eiq "${RAICOM_HEAD_CAMERA_FRONT_PATTERN:-HD Webcam|Integrated Camera|Front Camera|Built-in|Internal Camera}"; then
    printf 'front'
  else
    printf 'camera'
  fi
}

describe_device() {
  local device="$1"
  local real label role
  real="$(real_device "$device")"
  label="$(device_label "$device")"
  role="$(device_role "$label")"
  printf '%s %s (%s)' "$role" "$real" "$label"
}

device_role_for_device() {
  device_role "$(device_label "$1")"
}

role_priority() {
  case "$1" in
    external) echo 10 ;;
    front) echo 20 ;;
    *) echo 50 ;;
  esac
}

log() {
  printf '[CAMERA_WATCH %s] %s\n' "$(timestamp)" "$*"
}

node_alive() {
  rosnode ping -c 1 "$camera_node" >/dev/null 2>&1
}

topic_alive() {
  timeout "${RAICOM_HEAD_CAMERA_TOPIC_TIMEOUT:-6}" rostopic echo -n 1 "$image_topic" >/dev/null 2>&1
}

current_device() {
  rosparam get "$video_param" 2>/dev/null || true
}

restart_camera() {
  local reason="$1"
  local desired_device="$2"
  local now
  now="$(date +%s)"
  if [ $((now - last_restart_epoch)) -lt "$restart_cooldown" ]; then
    log "$reason; restart suppressed by cooldown (${restart_cooldown}s)"
    return 0
  fi
  if [ -n "$desired_device" ]; then
    rosparam set "$video_param" "$desired_device" >/dev/null 2>&1 || true
  fi
  last_restart_epoch="$now"
  log "$reason; restarting $camera_node so usb_cam re-resolves the device"
  rosnode kill "$camera_node" >/dev/null 2>&1 || true
}

log "started: hold current camera while images are fresh; prefer external only after stable restore; fall back on stream loss"

last_selected=""
last_active=""
miss_count=0
restore_candidate=""
restore_hits=0
last_restart_epoch=0

while true; do
  current="$(current_device)"
  current_real="$(real_device "$current")"

  if node_alive; then
    if topic_alive; then
      miss_count=0
      if [ -n "$current" ] && [ "$current_real" != "$last_active" ]; then
        log "active stream on $(describe_device "$current")"
        last_active="$current_real"
      fi

      current_role="$(device_role_for_device "$current")"
      if [ "$current_role" = "external" ]; then
        restore_candidate=""
        restore_hits=0
      else
        desired="$("$resolver" 2>/dev/null || true)"
        desired_real="$(real_device "$desired")"
        desired_role="$(device_role_for_device "$desired")"
        if [ -n "$desired" ] && [ "$desired_real" != "$current_real" ] && \
           [ "$(role_priority "$desired_role")" -lt "$(role_priority "$current_role")" ]; then
          if [ "$desired_real" = "$restore_candidate" ]; then
            restore_hits=$((restore_hits + 1))
          else
            restore_candidate="$desired_real"
            restore_hits=1
          fi
          log "better camera candidate $(describe_device "$desired") stable ${restore_hits}/${restore_confirm}"
          if [ "$restore_hits" -ge "$restore_confirm" ]; then
            restart_camera "external camera restored: current=$(describe_device "$current") desired=$(describe_device "$desired")" "$desired_real"
            restore_hits=0
          fi
        else
          restore_candidate=""
          restore_hits=0
        fi
      fi
    else
      miss_count=$((miss_count + 1))
      log "no fresh image on $image_topic ($miss_count/$miss_limit)"
      if [ "$miss_count" -ge "$miss_limit" ]; then
        desired="$(RAICOM_HEAD_CAMERA_EXCLUDE_DEVICE="$current_real" "$resolver" 2>/dev/null || true)"
        desired_real="$(real_device "$desired")"
        if [ -n "$desired" ] && [ "$desired_real" != "$last_selected" ]; then
          log "resolver selected $(describe_device "$desired")"
          last_selected="$desired_real"
        fi
        restart_camera "image stream lost on $(describe_device "$current"); switching to $(describe_device "$desired")" "$desired_real"
        miss_count=0
      fi
    fi
  else
    miss_count=0
    desired="$("$resolver" 2>/dev/null || true)"
    desired_real="$(real_device "$desired")"
    if [ -n "$desired" ] && [ "$desired_real" != "$current_real" ]; then
      rosparam set "$video_param" "$desired_real" >/dev/null 2>&1 || true
      log "$camera_node is not running; prepared $(describe_device "$desired") for roslaunch respawn/start"
    else
      log "$camera_node is not running; waiting for roslaunch respawn/start"
    fi
  fi

  sleep "$interval"
done
