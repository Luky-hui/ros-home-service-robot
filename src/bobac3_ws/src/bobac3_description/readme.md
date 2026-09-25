# bobac3机器人模型

## 介绍

本包为Bobac3机器人模型提供了一个URDF模型，包含了机器人的所有部件。

## 配置

本包添加了新版本的机器人模型适配雷达锐驰CSPC，如果要使用该雷达需要在.bashrc文件中添加以下内容：

```bash
    # 锐驰CSPC雷达类型 添加到.bashrc文件中
    export LASER_TYPE="CSPC"
    # EAI雷达类型 添加到.bashrc文件中
    export LASER_TYPE="EAI"
```

只对模型有效，不针对仿真环境。

## 仿真环境配置

本包仿真支持ubuntu 18.04(ROS melodic)版本和unbuntu 20.04(ROS noetic)版本，需要配置好ROS环境。
运行以下指令配置所需依赖：

```bash
roscd bobac3_description/scripts
$ ./install.sh
```

## 仿真

本包自带了一个现成的仿真地图，可以直接运行gazebo.launch文件进行仿真。

```bash
roslaunch bobac3_description gazebo.launch
```
