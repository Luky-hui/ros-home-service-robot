#!/usr/bin/env bash
set +e

for name in roslaunch rosmaster rosout gzserver gzclient rviz move_base map_server amcl robot_state_publisher; do
  pids=$(ps -eo pid,comm | awk -v n="$name" '$2==n {print $1}')
  if [ -n "$pids" ]; then
    kill $pids 2>/dev/null || true
  fi
done

sleep 2

for name in gzserver gzclient roslaunch rosmaster; do
  pids=$(ps -eo pid,comm | awk -v n="$name" '$2==n {print $1}')
  if [ -n "$pids" ]; then
    kill -9 $pids 2>/dev/null || true
  fi
done

echo "service simulation cleaned"
