# 国赛服务组三任务

本功能包实现：

- 导览：走廊唤醒，依次介绍餐厅、厨房、客厅、卧室并返回出发区。
- 智能助理：识别冰箱食材，播报食材并给出确定的菜品和做法。
- 寻物：寻找手机、书包或水瓶，逐房间播报识别结果。

## 首次标定

家庭场景地图及点位当前设备中不存在，因此配置不会预填坐标。启动导航与定位后，把机器人依次放到六个真实点位并执行：

```bash
rosrun home_service_mission capture_home_waypoint.py start
rosrun home_service_mission capture_home_waypoint.py corridor
rosrun home_service_mission capture_home_waypoint.py restaurant
rosrun home_service_mission capture_home_waypoint.py kitchen
rosrun home_service_mission capture_home_waypoint.py living_room
rosrun home_service_mission capture_home_waypoint.py bedroom
rosrun home_service_mission home_config_check.py
```

## 启动

```bash
rosrun home_service_mission start_home_task.sh auto
rosrun home_service_mission start_home_task.sh guide
rosrun home_service_mission start_home_task.sh assistant
rosrun home_service_mission start_home_task.sh find_object "cell phone"
```

寻物目标值固定为 `cell phone`、`backpack`、`water bottle`。
