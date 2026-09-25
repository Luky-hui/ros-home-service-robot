# ROS Home Service Robot

> 基于 ROS、Gazebo 与本地视觉/语音模型的居家服务机器人系统，实现语音交互、自主导航、食材识别、智能寻物与视觉回充等完整服务流程。

<p align="left">
  <img src="https://img.shields.io/badge/ROS-Noetic-blue?logo=ros" />
  <img src="https://img.shields.io/badge/Gazebo-Simulation-orange" />
  <img src="https://img.shields.io/badge/RViz-Visualization-green" />
  <img src="https://img.shields.io/badge/Python-Robotics-blue?logo=python" />
  <img src="https://img.shields.io/badge/C++-ROS_Node-blue?logo=cplusplus" />
  <img src="https://img.shields.io/badge/YOLO-Detection-purple" />
  <img src="https://img.shields.io/badge/ONNX_Runtime-Inference-red" />
</p>

## 项目演示

[![Bilibili](https://img.shields.io/badge/Bilibili-观看完整演示-00A1D6?logo=bilibili&logoColor=white)](https://www.bilibili.com/video/BV1i3hy6aEdQ/)

▶ **演示视频：**  
https://www.bilibili.com/video/BV1i3hy6aEdQ/

演示视频展示机器人在家庭仿真环境中的多任务服务流程：

- 语音唤醒与语义指令理解
- 多房间自主导航与语音导览
- 厨房食材识别与菜品推荐
- 手机/书包目标寻找
- 视觉辅助回充

> 视频内容基于 Gazebo 仿真环境录制，主要用于展示系统集成、任务状态机、导航与感知流程。

---

## 项目简介

本项目构建了一个面向居家场景的服务机器人系统。机器人从出发区启动后，可前往走廊待机，通过人脸检测或接近触发完成唤醒，并根据用户语音指令执行不同服务任务。

系统围绕三个高频居家服务场景设计：

```text
语音导览
  出发区 -> 走廊待机 -> 语音指令 -> 餐厅 -> 厨房 -> 客厅 -> 卧室 -> 返回

智慧助手
  出发区 -> 走廊待机 -> 厨房 -> 食材识别 -> 菜品推荐 -> 返回

智能寻物
  出发区 -> 走廊待机 -> 多房间搜索 -> 手机/书包识别 -> 结果播报 -> 返回
```

把 **导航、语音、视觉、状态机、回充与仿真验证** 组织成一套可运行的完整机器人应用。

---

## 系统架构

```text
User / Human Presence
        ↓
Face Detection / Wake Trigger
        ↓
Offline ASR
        ↓
Semantic Intent Router
        ↓
Mission State Machine
 ├── Guide Mission
 ├── Food Assistant Mission
 ├── Object Finding Mission
 └── Charging Mission
        ↓
Navigation Client
        ↓
move_base / Costmap / AMCL
        ↓
Gazebo Robot Simulation
        ↓
Camera / Laser / TF Feedback
        └──────────────→ Mission State Machine
```

视觉识别链路：

```text
Camera Image
    ↓
Vision Detection ROS Service
    ↓
YOLO / ONNX Runtime
    ↓
Detection Filtering
    ↓
Task-specific Speech Report
```

---

## 核心功能

| 模块 | 实现内容 |
| --- | --- |
| 任务状态机 | 三类居家服务任务与回充任务统一调度 |
| 自主导航 | 基于地图、AMCL、move_base 的多点导航 |
| 路径配置 | 地图点位、路线、中转点、误差阈值参数化 |
| 人脸唤醒 | 机器人到走廊后等待人员靠近再响应 |
| 语音交互 | 离线 ASR、TTS、语义意图分流与干扰命令回答 |
| 食材识别 | 14 类食材检测、置信度排序、菜品推荐 |
| 智能寻物 | 手机/书包检测、近距离面积过滤与结果播报 |
| 视觉回充 | 底部相机识别 AR 标记并辅助对准充电桩 |
| 可视化工具 | RViz、识别监视窗口、地图禁行区/慢行区标注 |
| 录屏脚本 | Ubuntu 画面与系统声音录制脚本 |

---

## 食材识别与菜品推荐

智慧助手任务使用独立蔬菜/食材识别模型。识别逻辑会对同类目标保留最高置信度结果，并按置信度排序选择主要食材生成播报。

当前模型：

```text
models/veg14_v15_newonly/veg14_v15_newonly_best.onnx
```

支持类别包括：

```text
上海青、玉米、虾、黄瓜、猪肉、茄子、土豆、鸡蛋、
番茄、鱼肉、青椒、豆腐、香蕉、苹果
```

---

## 智能寻物

智能寻物任务使用独立手机/书包识别模型，与食材识别模型分离。

当前模型：

```text
models/phone_bag_v6_combined/phone_bag_yolo11n_v6_combined_best.onnx
```

识别类别：

```text
phone
bag
```

系统在每个目标区域完成检测后，根据结果播报：

- 找到目标物
- 未发现目标物
- 发现其他相关物品

---

## 技术栈

**机器人与中间件：**  
`ROS Noetic` · `move_base` · `AMCL` · `TF` · `RViz`

**仿真：**  
`Gazebo` · `URDF / Xacro` · `map_server`

**视觉：**  
`YOLO` · `ONNX Runtime` · `OpenCV` · `cv_bridge`

**语音：**  
`SenseVoice` · `MeloTTS` · `VAD` · `sherpa-onnx`

**开发：**  
`Python` · `C++` · `Bash` · `YAML`

---

## 项目结构

```text
ros-home-service-robot/
│
├── src/
│   ├── bobac3_ws/
│   │   └── src/
│   │       ├── bobac3_description/
│   │       ├── bobac3_navigation/
│   │       ├── face_rec/
│   │       ├── home_service_mission/
│   │       ├── mission supervisor/
│   │       ├── vision service/
│   │       ├── robot_audio/
│   │       └── ...
│   │
│   └── ros_workspace/
│       └── src/
│           ├── local_voice_bridge/
│           └── rei_voice/
│
├── models/
│   ├── veg14_v15_newonly/
│   ├── phone_bag_v6_combined/
│   ├── sensevoice/
│   ├── melo_tts/
│   ├── vad/
│   └── sherpa-onnx-kws-zipformer-zh-en-3M-2025-12-20/
│
├── scripts/
│   ├── system audio recording script
│   ├── full audio recording script
│   └── keyboard control demo
│
├── docs/
│   ├── COMMANDS.md
│   ├── MODELS.md
│   ├── PUBLICATION_NOTES.md
│   └── EXPORT_MANIFEST.md
│
├── .gitattributes
├── .gitignore
└── README.md
```

---

## 快速启动

### 1. 编译工作空间

```bash
source /opt/ros/noetic/setup.bash
cd ~/bobac3_ws
catkin_make

source /opt/ros/noetic/setup.bash
cd ~/ros_workspace
catkin_make
```

### 2. 启动仿真与导航

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

### 3. 一键运行居家服务流程

完整启动指令见 [docs/COMMANDS.md](docs/COMMANDS.md)。该文档保留了工程内部 ROS 包名、脚本名和环境变量，便于复现。

---

## 语音指令示例

语音导览：

```text
可以带我参观一下吗
带我参观一下
参观一下
```

智慧助手：

```text
给我推荐一下今天适合做什么菜
推荐一下今天适合做什么菜
去看看冰箱里有什么
看下还剩什么菜
```

智能寻物：

```text
帮我找一下手机
帮我找一下书包
```

---

## 录屏

只录制 Ubuntu 系统声音：

录屏脚本位于 `scripts/` 目录，完整用法见 [docs/COMMANDS.md](docs/COMMANDS.md)。

---

## 项目边界

本项目主要用于展示居家服务机器人系统集成能力。

当前已完成：

- 居家场景任务状态机
- 多点自主导航与路径参数化
- 人脸唤醒与语音交互
- 食材识别与菜品推荐
- 手机/书包智能寻物
- AR/视觉辅助回充
- Gazebo + RViz 仿真验证
- 录屏与识别监视工具

需要注意：

- 仓库中的演示结果来自仿真环境。
- 模型文件较大，建议使用 Git LFS 管理。
- 部分仿真资源、第三方模型和依赖应遵循其原始许可证。
- 在其他机器复现时，需要根据实际用户名、工作空间路径、音频设备和摄像头话题调整配置。

---

## 说明

本仓库整理为作品集项目，重点展示 ROS 服务机器人从感知、交互、决策到执行的完整工程链路。
