#!/usr/bin/env bash
set -u

model_name=""
args=("$@")
for ((i = 0; i < ${#args[@]}; i++)); do
  if [ "${args[$i]}" = "-model" ] && [ $((i + 1)) -lt ${#args[@]} ]; then
    model_name="${args[$((i + 1))]}"
    break
  fi
done

tmp_log="/tmp/raicom_spawn_model_${model_name:-unknown}_$$.log"
rosrun gazebo_ros spawn_model "$@" >"$tmp_log" 2>&1
status=$?
if [ "$status" -eq 0 ]; then
  cat "$tmp_log"
  rm -f "$tmp_log"
  exit 0
fi

if [ -z "$model_name" ]; then
  cat "$tmp_log"
  rm -f "$tmp_log"
  exit "$status"
fi

for _ in $(seq 1 15); do
  if timeout 2 rostopic echo -n 1 /gazebo/model_states 2>/dev/null | grep -q -- "- $model_name"; then
    echo "WARN spawn_model returned $status, but model '$model_name' is present in /gazebo/model_states; treating as success."
    rm -f "$tmp_log"
    exit 0
  fi
  sleep 1
done

cat "$tmp_log"
rm -f "$tmp_log"
exit "$status"
