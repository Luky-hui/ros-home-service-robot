# service_group_mission

服务组比赛任务总控包，当前覆盖两个选拔赛任务：

- 任务一：迎宾引导，流程为人脸检测唤醒、语音命令确认、导航到深圳馆、播报介绍、返回出发区。
- 任务二：巡检，流程为管理员人脸识别、语音命令确认、遍历吉林/广州/北京/上海/深圳、识别巡检事件、报警播报、导航到充电桩。

## 启动前置

先启动服务组仿真导航底座：

```bash
source /opt/ros/noetic/setup.bash
source ~/bobac3_ws/devel/setup.bash
export MAP_DIRECTORY=~/bobac3_ws/src/bobac3_navigation/maps
roslaunch bobac3_navigation demo_nav_2d.launch open_nav_rviz:=true map_file_name:=reicom map_directory:=$MAP_DIRECTORY
```

人脸和语音可以接真实模块，也可以先用调试 topic 跑流程。

## 迎宾引导

真实人脸检测 + topic 模拟语音命令：

```bash
roslaunch face_rec face_detection.launch
roslaunch service_group_mission welcome_mission.launch face_mode:=any voice_mode:=topic
```

发送裁判命令：

```bash
rostopic pub -1 /service_group_mission/voice_command std_msgs/String "data: '可以带我去参观一下深圳馆吗？'"
```

如果想先跳过人脸和语音，只测导航流程：

```bash
roslaunch service_group_mission welcome_mission.launch face_mode:=skip voice_mode:=skip
```

## 巡检

管理员人脸识别需要先准备 `face_encodeing.npz`，并启动：

```bash
roslaunch face_rec face_verification.launch
roslaunch service_group_mission patrol_mission.launch face_mode:=admin voice_mode:=topic
```

发送管理员命令：

```bash
rostopic pub -1 /service_group_mission/voice_command std_msgs/String "data: '开始执行巡检任务'"
```

调试时可以跳过人脸和语音：

```bash
roslaunch service_group_mission patrol_mission.launch face_mode:=skip voice_mode:=skip
```

## 巡检事件识别

默认从 `/head_camera/image_raw` 取图，通过 `/raicom_vision/detect` 连续检测三帧：

- 任一帧检出 `fire`：播报“在XX馆发现火源”
- 三帧均未检出 `fire extinguisher`：播报“XX馆未放置灭火器”
- 检出灭火器且未检出火源：正常
- 视觉服务或相机不可用：记录故障，不产生误报警

如果要先做全流程联调，可以在 `config/service_group_waypoints.yaml` 里把 `inspection.event_source` 设为非 `vision`，或填入：

```yaml
scripted_events:
  beijing: fire
  shenzhen: extinguisher_missing
```

## 后续要突破的点

- 校准 `service_group_waypoints.yaml` 中 50cm 白线框的精确坐标和朝向。
- 将语音 topic 调试替换为 `robot_audio` 的真实听写/语义链路。
- 将颜色规则替换为火源/灭火器缺失检测模型。
- 充电桩目前先导航到对接点，后续需要补对齐、接触确认和失败重试。
