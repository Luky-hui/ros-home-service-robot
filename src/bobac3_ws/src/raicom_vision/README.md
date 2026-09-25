# raicom_vision

本地 CPU 开放词汇目标检测节点，统一覆盖服务组赛题中的火源、灭火器、冰箱食材、手机、书包和水瓶。

## ROS 接口

- 服务：`/raicom_vision/detect`，类型 `raicom_vision/DetectObjects`
- 标注图：`/raicom_vision/annotated`
- JSON 结果：`/raicom_vision/detections`

启动：

```bash
roslaunch raicom_vision vision.launch
```

相机检测自检：

```bash
rosrun raicom_vision vision_smoke_test.py
```
