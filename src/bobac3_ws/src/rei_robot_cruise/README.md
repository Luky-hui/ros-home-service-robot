# rei_robot_cruise——move_base通用多点巡航功能包

本功能包基于move_base实现多点导航功能，并具有以下特点：

1、可自定义导航回调任务

2、可通过yaml文件配置目标点信息，设置巡航任务

3、可通过service增加、修改、删除已有目标点以及设置巡航任务

4、在rviz中可查看已设置的目标点位置

5、使用者通过实例化类RobotCruise，链接librei_robot_cruise.so即可在其他功能包中使用

## 使用步骤

### 一、下载功能包

终端进入自己工作空间的src目录下，输入以下指令下载：

`$ git clone https://gitee.com/maintainer/rei_robot_cruise.git`

### 二、编译

使用catkin_make即可编译

### 三、创建自己的功能包

rei_robot_cruise只编译生成了动态库librei_robot_cruise.so，使用时需要

为用户创建自己的功能包。

必要依赖：rei_robot_cruise

例如：输入以下指令终端创建功能包

```bash
$ catkin_create_pkg curise_demo roscpp rei_robot_cruise
```

#### Demo

```c++
#include <rei_robot_cruise/cruise.h>
bool hello(int i) {
  ROS_INFO("hello");
  int step = i;
  while(ros::ok()){
    //
    if(rei_cruise::RobotCruise::getInstance().GetTaskState() == rei_cruise::PAUSE){
      //检查任务是否暂停，如果是，则记录当前回调任务进度
      rei_cruise::RobotCruise::getInstance().SetCallBackFuncStep(step);
      break;
    }else if(rei_cruise::RobotCruise::getInstance().GetTaskState() == rei_cruise::STOP){
      //检查任务是否停止，如果是，则结束当前回调任务
      break;
    }
    if (step > 2) break;
    switch (step) {
      case 0:
        ROS_INFO("0");
        step++;
        break;
      case 1:
        ROS_INFO("1");
        step++;
        break;
      case 2:
        ROS_INFO("2");
        step++;
        break;
    }
    // 如果想要回调函数return false导致任务暂停后，下次开始时从回调任务函数中特定步骤开始任务，则只需要在return false 前调用SetCallBackFuncStep()设置目标步骤
    sleep(1);
  }
  return true;
}
bool bye(int i) {
  ROS_INFO("bye");
  return true;
}
int main(int argc, char** argv) {
  ros::init(argc, argv, "cruise_test_node");
  ros::NodeHandle nh;
  rei_cruise::RobotCruise& cruise = rei_cruise::RobotCruise::getInstance();
  if (!cruise.Init(nh)) return 0;  // 初始化失败，结束程序
  if (cruise.AddCallBack("hello", hello)) {
    ROS_INFO("set callback hello success");
  }
  if (cruise.AddCallBack("bye", bye)) {
    ROS_INFO("set callback bye success");
  }
  cruise.RunNavTask();
  return 0;
}
int main2(int argc, char** argv) {  // 另一种调用方式
  ros::init(argc, argv, "cruise_test_node");
  ros::NodeHandle nh;
  if (!rei_cruise::RobotCruise::getInstance().Init(nh))
    if (rei_cruise::RobotCruise::getInstance().AddCallBack("hello", hello)) {
      ROS_INFO("set callback hello success");
    }
  if (rei_cruise::RobotCruise::getInstance().AddCallBack("bye", bye)) {
    ROS_INFO("set callback bye success");
  }
  rei_cruise::RobotCruise::getInstance().RunNavTask();
  return 0;
}

```

#### 流程：

1. 调用Init()函数
2. 如果有自定义的导航回调函数需要添加则使用AddCallBack()进行添加
3. 最后再调用RunNavTask()，调用该函数，会首先根据需要读取yaml文件，然后进入循环阻塞，等待开始任务服务被指令调用
4. 当请求服务“start_stop_nav”的参数mode为“start”时，机器人开始导航
5. 当机器人**成功导航**到目标点后自动开始调用对应**导航回调函数**。否则就会暂停直到使用者**请求开始任务服务“start_stop_nav”**
6. 导航回调函数执行成功后，开始下一个点的导航

## 参数说明：

~read_param：bool型，是否读取参数服务器
~task: string型，需要加载的任务文件(yaml)

任务文件编写：

position.yaml

```yaml
clear_old_goal: True #bool型，是否清除之前存储的目标点
goals_name: ["home", "work", "room"] #string型，目标点列表，以空格分隔
home: 
  frame_id: map #string型，目标点基坐标
  x: 1.0
  y: 0.0
  theta: 0.0
room:
  frame_id: map  #string型，目标点基坐标
  x: 2.0
  y: -2.0
  theta: -3.14159
work:
  frame_id: map  #string型，目标点基坐标
  x: 0.3
  y: 1.0
  theta: 2.0
nav_order: ["work", "room"] #string型，任务顺序列表，以空格分隔
callback: ["hello", "bye"] #string型，对应任务顺序列表中每个任务点的回调任务，以空格分隔
loop_time: 3 #string型，任务循环次数
```

position.json

```yaml
{
	"clear_old_goal": true,
	"goals_name": ["home", "room", "work"],
	"home": {
		"frame_id": "map",
		"x": 1,
		"y": 0,
		"theta": 0
	},
	"room": {
		"frame_id": "map",
		"x": 2,
		"y": -2,
		"theta": -3.14159
	},
	"work": {
		"frame_id": "map",
		"x": 0.3,
		"y": 1,
		"theta": 2
	},
	"nav_order": ["room", "work"],
	"callback": ["hello", "bye"],
	"loop_time": 3
}
```
## Services

|                    服务名                     |             功能             | srv介绍                                                      |
| :-------------------------------------------: | :--------------------------: | :----------------------------------------------------------- |
|     set_goals(rei_robot_cruise/SetGoals)i     |       添加、修改目标点       | string mode 设置模式 "reset"：删掉之前保存的目标点重新设置<br/> "insert" ：直接添加目标点<br/>rei_robot_cruise/NavGoal[] goals 目标点信息<br />---<br />bool isSuccess 是否添加成功 |
| set_goals_order(rei_robot_cruise/SetNavOrder) | 设置目标点巡航顺序及回调任务 | int16 loop_time 循环次数，0表示无限循环<br />string[] goal_order 目标点巡航顺序<br />string[] callback_order 对应导航回调函数<br />---<br />string message 错误信息<br />bool isSuccess 是否操作成功 |
|   remove_goals(rei_robot_cruise/RemoveGoal)   |          删除目标点          | string mode 删除模式，“all” 删除全部的目标点，“not_all” 只删除goals_name中的目标点<br />string[] goal_names 需要删除的点<br />---<br />bool isSuccess 是否操作成功 |
| start_stop_nav(rei_robot_cruise/StartStopNav) |     开始、停止、暂停巡航     | string mode 动作指令，"start" 开始任务，“pause” 暂停任务，“stop” 停止任务<br />int8 level 指令等级，暂无作用<br />---<br />bool isSuccess 是否操作成功 |
|     load_task(rei_robot_cruise/LoadTask)      |         加载任务文件         | int8 type 文件类型，0 自动识别，1 yaml，2 json<br />string task_file 任务文件名(含路径)<br />---<br />string msg 错误信息打印<br />int8 err 错误码，0 成功，-1 正在运行任务，-2 未识别格式，-3 文件不存在或任务内容有误 |

## Topics

goals_markers(visualization_msgs/MarkerArray)：用于在rviz中显示目标点