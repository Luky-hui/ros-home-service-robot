# 常用复现指令

## 启动导航与仿真

```bash
source /opt/ros/noetic/setup.bash
source /home/robot/bobac3_ws/devel/setup.bash
source /home/robot/ros_workspace/devel/setup.bash --extend
pkill -f 'roslaunch bobac3_navigation demo_nav_2d.launch' || true
pkill -f 'roslaunch bobac3_description gazebo.launch' || true
pkill -f gzserver || true
pkill -f gzclient || true
pkill -f rosmaster || true
pkill -f roscore || true
sleep 4
export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=ubuntu
export LASER_TYPE=CSPC
export REI_ROBOT=bobac3sim
export DISPLAY=:0
unset LIBGL_ALWAYS_SOFTWARE
roslaunch bobac3_navigation demo_nav_2d.launch map_file_name:=national_2026 open_nav_rviz:=true
```

## 执行任务一到任务四

```bash
source /opt/ros/noetic/setup.bash
source /home/robot/bobac3_ws/devel/setup.bash
source /home/robot/ros_workspace/devel/setup.bash --extend
export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=ubuntu
export LASER_TYPE=CSPC
export REI_ROBOT=bobac3sim
export DISPLAY=:0
export RAICOM_FACE_MODE=any
export RAICOM_AUDIO_SOURCE=alsa_input.usb-Generalplus_Usb_Audio_Device-00.mono-fallback
export RAICOM_ASSISTANT_VISION_MONITOR=true
rosrun raicom_supervisor run_national_tasks_only.sh
```

## 录制系统声音

```bash
cd /home/robot/Desktop
./record_raicom_system_audio.sh
```
