# raicom_supervisor

启动：

```bash
roslaunch raicom_supervisor supervisor.launch
```

急停：

```bash
rosservice call /raicom/emergency_stop/set "data: true"
```

解除急停：

```bash
rosservice call /raicom/emergency_stop/set "data: false"
```

急停生效时节点以 20Hz 发布零速度并取消两个已知导航 Action 的全部目标。

一键启动任务：

```bash
rosrun raicom_supervisor start_competition_task.sh selection_welcome
rosrun raicom_supervisor start_competition_task.sh selection_patrol
rosrun raicom_supervisor start_competition_task.sh home_auto
rosrun raicom_supervisor start_competition_task.sh home_guide
rosrun raicom_supervisor start_competition_task.sh home_assistant
rosrun raicom_supervisor start_competition_task.sh home_find "cell phone"
```
