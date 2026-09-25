#include <rei_robot_cruise/cruise.h>

namespace rei_cruise {
RobotCruise::RobotCruise()
    : mode_(TaskState::IDEL),
      loopTimes_(1),
      moveBaseClientPtr_(new MOVEBASE("move_base", false)),
      taskState_(0),
      flagPublishMarker_(false),
      currentNavingGoal_(0),
      moveBaseName_("move_base") {}
RobotCruise::~RobotCruise() { this->Stop(); }
// void RobotCruise::DynamicConfigCallback(robot_cruise::navigatorConfig
// &config, uint32_t level){

// }
bool RobotCruise::Init(ros::NodeHandle &nh) {
  nh_ = nh;
  setGoalsServer_ =
      nh_.advertiseService("set_goals", &RobotCruise::SetGoalsCallback, this);
  startStopNavServer_ = nh_.advertiseService(
      "start_stop_nav", &RobotCruise::StartStopGoalsCallback, this);
  setGoalsOrderServer_ = nh_.advertiseService(
      "set_goals_order", &RobotCruise::SetGoalsOrderCallback, this);
  removeGoalsServer_ = nh_.advertiseService(
      "remove_goals", &RobotCruise::RemoveGoalCallback, this);
  loadTaskServer_ =
      nh_.advertiseService("load_task", &RobotCruise::LoadTaskCallback, this);
  ros::SubscriberStatusCallback pub_marker_cb = std::bind([&] {
    if (markerPublisher_.getNumSubscribers() > 0) {
      flagPublishMarker_ = true;
      ROS_INFO_STREAM("Starting publish goal marker.");
    } else {
      flagPublishMarker_ = false;
      ROS_INFO_STREAM("Stopping publish goal marker.");
    }
  });

  markerPublisher_ = nh_.advertise<visualization_msgs::MarkerArray>(
      "goals_markers", 1, pub_marker_cb, pub_marker_cb);

  marker_.header.frame_id = "map";
  marker_.header.stamp = ros::Time::now();
  marker_.color.r = 1.0f;
  marker_.color.g = 0.0f;
  marker_.color.b = 0.0f;
  marker_.color.a = 0.5;
  marker_.lifetime = ros::Duration(3.0);  // 每个模型的生命周期为3秒
  marker_.action = visualization_msgs::Marker::ADD;

  callbackMap_.emplace("default", [](int) -> bool { return true; });
  startCallback_ = []() -> bool { return true; };
  return true;
}
void RobotCruise::SetLandMark(geometry_msgs::Pose pose, std::string goal_name) {
  visualization_msgs::MarkerArray markers;
  // 该模型是由箭头和方块组合而来的, 所有其实有一个目标点模型其实有两个marker
  marker_.pose = pose;

  marker_.type = visualization_msgs::Marker::CUBE;
  marker_.scale.x = 0.2;
  marker_.scale.y = 0.2;
  marker_.scale.z = 0.2;
  marker_.ns = goal_name + "_0";
  markers.markers.push_back(marker_);

  marker_.type = visualization_msgs::Marker::ARROW;
  marker_.scale.x = 0.5;
  marker_.scale.y = 0.1;
  marker_.scale.z = 0.1;
  marker_.ns = goal_name + "_1";
  markers.markers.push_back(marker_);

  landmarkersMap_[goal_name] = markers;
}

/*setGoalsCallback函数逻辑
    检查当前任务是否停止
    如果req.mode为reset即重置所有目标点——清空goals_list_、landmarkers_map_、executable_order_
    依次读取目标点存入goals_list_、landmarkers_map_中,
   已有的名字会覆盖掉对应坐标信息
*/
bool RobotCruise::SetGoalsCallback(rei_robot_cruise::SetGoals::Request &req,
                                   rei_robot_cruise::SetGoals::Response &res) {
  setlocale(LC_CTYPE, "zh_CN.utf8");
  ROS_INFO("调用set_goals服务");
  if (taskState_ != TaskState::STOP) {
    ROS_WARN("please stop the task");
    res.isSuccess = false;
    return true;
  }
  if (req.mode == "reset") {
    if (!goalsList_.empty())
      goalsList_.clear();  // 内存回收由map自身决定, 频繁操作可能会造成内存泄漏
    if (!executableOrder_.empty()) executableOrder_.clear();
    if (!landmarkersMap_.empty()) landmarkersMap_.clear();

  } else if (req.mode != "insert") {
    res.isSuccess = false;
    ROS_ERROR("模式指令错误, 可识别的指令为: insert, reset");
  }
  for (auto const &goal : req.goals) {
    auto ret = goalsList_.emplace(goal.point_name, goal);
    if (!ret.second) {
      goalsList_.find(goal.point_name)->second = goal;
    }
    SetLandMark(goal.target_pose.pose, goal.point_name);
  }
  res.isSuccess = true;
  return true;
}

/*
设置巡航顺序, 并将巡航计划内的目标点模型变为绿色
*/
bool RobotCruise::SetGoalsOrderCallback(
    rei_robot_cruise::SetNavOrder::Request &req,
    rei_robot_cruise::SetNavOrder::Response &res) {
  setlocale(LC_CTYPE, "zh_CN.utf8");
  ROS_INFO("调用set_goal_order服务");
  if (taskState_ != TaskState::STOP) {
    ROS_WARN("please stop the task");
    res.isSuccess = false;
    return true;
  }
  requestLoop_ = req.loop_time;
  loopTimes_ = requestLoop_;
  std_msgs::ColorRGBA green_color;
  green_color.r = 0.0f;
  green_color.g = 1.0f;
  green_color.b = 0.0f;
  green_color.a = 0.8;
  if (!req.goal_order.empty()) {
    std::vector<std::string> unkown_goals;
    std::vector<std::string> unkown_cbs;
    for (auto const &goal : req.goal_order) {
      auto iter = goalsList_.find(goal);
      if (iter == goalsList_.end()) unkown_goals.push_back(goal);
    }
    for (auto const &goal_cb : req.callback_order) {
      auto cb_iter = callbackMap_.find(goal_cb);
      if (cb_iter == callbackMap_.end()) unkown_cbs.push_back(goal_cb);
    }
    if (unkown_goals.empty() && unkown_cbs.empty()) {
      std::map<int, std::string> empty_order_map;
      std::map<int, std::function<bool(int)>> empty_order_cb;
      executableOrder_.swap(empty_order_map);
      executableOrderCb_.swap(empty_order_cb);
      executableOrder_.clear();
      executableOrderCb_.clear();
      for (size_t i = 0; i < req.goal_order.size(); i++) {
        executableOrder_[i] = req.goal_order[i];
        executableOrderCb_[i] = callbackMap_["default"];
        landmarkersMap_[executableOrder_[i]].markers[0].color = green_color;
        landmarkersMap_[executableOrder_[i]].markers[1].color = green_color;
      }
      for (size_t i = 0; i < req.callback_order.size(); i++) {
        executableOrderCb_[i] = callbackMap_[req.callback_order[i]];
      }
      res.isSuccess = true;
    } else {
      res.isSuccess = false;
      if (!unkown_goals.empty()) {
        ROS_ERROR("存在未知目标点");
        res.message = "There are some unkown goals: ";
        for (auto const &unkown_goal : unkown_goals) {
          res.message = res.message + unkown_goal + std::string("; ");
        }
        res.message = res.message + std::string("\n");
        return true;
      }
      if (!unkown_cbs.empty()) {
        ROS_ERROR("存在未知回调任务名");
        res.message = "There are some unkown callback: ";
        for (auto const &unkown_cb : unkown_cbs) {
          res.message = res.message + unkown_cb + std::string("; ");
        }
        res.message = res.message + std::string("\n");
        return true;
      }
    }
  }
  return true;
}
bool RobotCruise::RemoveGoalCallback(
    rei_robot_cruise::RemoveGoal::Request &req,
    rei_robot_cruise::RemoveGoal::Response &res) {
  setlocale(LC_CTYPE, "zh_CN.utf8");
  if (taskState_ != TaskState::STOP) {
    ROS_ERROR("没有正在执行或暂停中的任务");
    res.isSuccess = false;
    return true;
  }
  if (req.mode == "all") {  // req.mode为all则清空所有目标点
    std::unordered_map<std::string, rei_robot_cruise::NavGoal> empty_goals;
    std::map<int, std::string> empty_order_map;
    std::map<int, std::function<bool(int)>> empty_order_cb;
    std::map<std::string, visualization_msgs::MarkerArray> empty_landmark;
    executableOrder_.swap(empty_order_map);
    executableOrderCb_.swap(empty_order_cb);
    goalsList_.swap(empty_goals);
    landmarkersMap_.swap(empty_landmark);
    goalsList_.clear();
    executableOrder_.clear();
    executableOrderCb_.clear();
    landmarkersMap_.clear();
  } else if (
      req.mode ==
      "not_all") {  // 如果是not_all,则分别查找并删除goals_list_、landmarkers_map_以及executable_order_中对应的点
    if (!req.goal_names.empty()) {
      for (auto const &goal_name : req.goal_names) {
        auto goal_iter = goalsList_.find(goal_name);
        if (goal_iter != goalsList_.end()) {
          goalsList_.erase(goal_iter);
          landmarkersMap_.erase(landmarkersMap_.find(goal_name));
          for (auto order_iter = executableOrder_.begin();
               order_iter != executableOrder_.end();) {
            if (order_iter->second == goal_name) {
              executableOrderCb_.erase(order_iter->first);
              executableOrder_.erase(order_iter++);
            } else
              order_iter++;
          }
          ROS_INFO("已删除目标点: %s", goal_name.c_str());
        } else
          ROS_WARN("没有找到目标点: %s", goal_name.c_str());
      }
      res.isSuccess = true;
    } else
      ROS_WARN("no goal removed ");
  } else {
    res.isSuccess = false;
    ROS_ERROR("remove mode is error, which given param: %s", req.mode.c_str());
  }
  return true;
}

bool RobotCruise::StartStopGoalsCallback(
    rei_robot_cruise::StartStopNav::Request &req,
    rei_robot_cruise::StartStopNav::Response &res) {
  setlocale(LC_CTYPE, "zh_CN.utf8");
  if (req.mode == "start") {
    if (this->Start()) res.isSuccess = true;
  } else if (req.mode == "pause") {
    if (this->Pause()) res.isSuccess = true;
  } else if (req.mode == "stop") {
    if (this->Stop()) res.isSuccess = true;
  } else {
    ROS_WARN("输入的指令错误, 仅可识别: start、stop、pause");
  }
  return true;
}
bool RobotCruise::LoadTaskCallback(rei_robot_cruise::LoadTask::Request &req,
                                   rei_robot_cruise::LoadTask::Response &res) {
  if (req.type == 1) {
    res.err = LoadYamlTask(req.task_file);
  } else if (req.type == 2) {
    res.err = LoadJsonTask(req.task_file);
  } else {
    if (req.task_file.find(".yaml") != std::string::npos)
      res.err = LoadYamlTask(req.task_file);
    else if (req.task_file.find(".json") != std::string::npos) {
      res.err = LoadJsonTask(req.task_file);
    } else {
      res.err = -2;
    }
  }
  switch (res.err) {
    case -1:
      res.msg = "task is running, please stop first";
      break;
    case -2:
      res.msg = "The format is not recognized";
      break;
    case -3:
      res.msg = "no such file , or syntax error";
      break;
  }
  return true;
}
int8_t RobotCruise::LoadYamlTask(const std::string &taskFile) {
  setlocale(LC_CTYPE, "zh_CN.utf8");
  if (GetTaskState() == TaskState::ACTIVE) return -1;
  try {
    YAML::Node taskConfig = YAML::LoadFile(taskFile);
    if (taskConfig["clear_old_goal"].as<bool>()) {
      landmarkersMap_.clear();
      goalsList_.clear();
    }
    YAML::Node taskGoalsName = taskConfig["goals_name"];

    ROS_INFO("--------------加载任务文件:---------------");
    for (auto goalName : taskGoalsName) {
      double x, y, theta;
      rei_robot_cruise::NavGoal info;
      info.point_name = goalName.as<std::string>();
      std::string frame_id;
      x = taskConfig[info.point_name.c_str()]["x"].as<double>();
      y = taskConfig[info.point_name.c_str()]["y"].as<double>();
      theta = taskConfig[info.point_name.c_str()]["theta"].as<double>();
      frame_id =
          taskConfig[info.point_name.c_str()]["frame_id"].as<std::string>();
      ROS_INFO("%s: %s %lf %lf %lf", info.point_name.c_str(), frame_id.c_str(),
               x, y, theta);
      double euler[3] = {0.0, 0.0, theta};
      info.target_pose.header.frame_id = frame_id;
      info.target_pose.pose.position.x = x;
      info.target_pose.pose.position.y = y;
      info.level = 1;
      this->EulerToQuart(euler, info.target_pose.pose.orientation);
      goalsList_[info.point_name.c_str()] = info;
      SetLandMark(info.target_pose.pose, info.point_name);
    }

    std_msgs::ColorRGBA green_color;
    green_color.r = 0.0f;
    green_color.g = 1.0f;
    green_color.b = 0.0f;
    green_color.a = 0.8;
    executableOrder_.clear();

    YAML::Node navOrders = taskConfig["nav_order"];

    YAML::Node callbacksName = taskConfig["callback"];
    int mindex = 0;
    for (auto order : navOrders) {
      std::string goalName = order.as<std::string>();
      ROS_INFO("goal %d(param): %s", mindex, goalName.c_str());

      auto iter = goalsList_.find(goalName);
      if (iter != goalsList_.end()) {
        if (mindex < callbacksName.size()) {
          std::string callback = callbacksName[mindex].as<std::string>();
          ROS_INFO("callback(param): %s", callback.c_str());
          auto cb_iter = callbackMap_.find(callback);
          if (cb_iter != callbackMap_.end())
            executableOrderCb_[mindex] = callbackMap_[callback];
          else {
            ROS_WARN(
                "未检测到函数: %s, "
                "当前点无回调任务,回调任务设为默认回调函数default",
                callback.c_str());
            executableOrderCb_[mindex] = callbackMap_["default"];
          }
        } else {
          ROS_WARN(
              "当前点(%s)未设置回调任务, 回调任务默认设为默认回调函数default",
              goalName.c_str());
          executableOrderCb_[mindex] = callbackMap_["default"];
        }
        executableOrder_[mindex] = goalName;
        landmarkersMap_[executableOrder_[mindex]].markers[0].color =
            green_color;
        landmarkersMap_[executableOrder_[mindex]].markers[1].color =
            green_color;
        mindex++;
      }
    }
    requestLoop_ = taskConfig["loop_time"].as<int>();
    loopTimes_ = requestLoop_;
    ROS_INFO("loop_time(param): %d", requestLoop_);

  } catch (const YAML::BadFile &e) {
    ROS_ERROR("无法加载YAML文件: %s, %s", taskFile.c_str(), e.what());
    return -3;
  } catch (const std::exception &ex) {
    ROS_ERROR("发生错误: %s", ex.what());
    return -3;
  }
  return 0;
}
int8_t RobotCruise::LoadJsonTask(const std::string &taskFile) {
  if (GetTaskState() == TaskState::ACTIVE) return -1;
  std::ifstream jfile(taskFile);
  if (!jfile.is_open())
  {
    ROS_ERROR("no such file");
    return -3;
  }
  try {
    nlohmann::json taskConfig;
    jfile >> taskConfig;
    jfile.close();
    ROS_INFO("--------------加载任务文件:---------------");
    if (taskConfig["clear_old_goal"]) {
      landmarkersMap_.clear();
      goalsList_.clear();
    }
    std::vector<std::string> taskGoalsName = taskConfig["goals_name"];

    for (auto goalName : taskGoalsName) {
      double x, y, theta;
      rei_robot_cruise::NavGoal info;
      info.point_name = goalName;
      std::string frame_id;
      x = taskConfig[info.point_name.c_str()]["x"];
      y = taskConfig[info.point_name.c_str()]["y"];
      theta = taskConfig[info.point_name.c_str()]["theta"];
      frame_id =
          taskConfig[info.point_name.c_str()]["frame_id"];
      ROS_INFO("%s: %s %lf %lf %lf", info.point_name.c_str(), frame_id.c_str(),
               x, y, theta);
      double euler[3] = {0.0, 0.0, theta};
      info.target_pose.header.frame_id = frame_id;
      info.target_pose.pose.position.x = x;
      info.target_pose.pose.position.y = y;
      info.level = 1;
      this->EulerToQuart(euler, info.target_pose.pose.orientation);
      goalsList_[info.point_name.c_str()] = info;
      SetLandMark(info.target_pose.pose, info.point_name);
    }

    std_msgs::ColorRGBA green_color;
    green_color.r = 0.0f;
    green_color.g = 1.0f;
    green_color.b = 0.0f;
    green_color.a = 0.8;
    executableOrder_.clear();

    std::vector<std::string> navOrders = taskConfig["nav_order"];

    std::vector<std::string> callbacksName = taskConfig["callback"];
    int mindex = 0;
    for (auto order : navOrders) {
      std::string goalName = order;
      ROS_INFO("goal %d(param): %s", mindex, goalName.c_str());

      auto iter = goalsList_.find(goalName);
      if (iter != goalsList_.end()) {
        if (mindex < callbacksName.size()) {
          std::string callback = callbacksName[mindex];
          ROS_INFO("callback(param): %s", callback.c_str());
          auto cb_iter = callbackMap_.find(callback);
          if (cb_iter != callbackMap_.end())
            executableOrderCb_[mindex] = callbackMap_[callback];
          else {
            ROS_WARN(
                "未检测到函数: %s, "
                "当前点无回调任务,回调任务设为默认回调函数default",
                callback.c_str());
            executableOrderCb_[mindex] = callbackMap_["default"];
          }
        } else {
          ROS_WARN(
              "当前点(%s)未设置回调任务, 回调任务默认设为默认回调函数default",
              goalName.c_str());
          executableOrderCb_[mindex] = callbackMap_["default"];
        }
        executableOrder_[mindex] = goalName;
        landmarkersMap_[executableOrder_[mindex]].markers[0].color =
            green_color;
        landmarkersMap_[executableOrder_[mindex]].markers[1].color =
            green_color;
        mindex++;
      }
    }
    requestLoop_ = taskConfig["loop_time"];
    loopTimes_ = requestLoop_;
    ROS_INFO("loop_time(param): %d", requestLoop_);
  } catch (const nlohmann::detail::parse_error &e) {
    ROS_ERROR("发生错误(parse_error): %s, %s", taskFile.c_str(), e.what());
    return -3;
  } catch (const nlohmann::detail::exception &e) {
    ROS_ERROR("发生错误(exception): %s, %s", taskFile.c_str(), e.what());
    return -3;
  }
  return 0;
}
bool RobotCruise::Start() {
  setlocale(LC_CTYPE, "zh_CN.utf8");
  if (taskState_ == TaskState::ACTIVE) {
    return false;
    ROS_WARN("已有任务正在执行...");
  }
  if (executableOrder_.empty()) {
    ROS_ERROR("需导航路点列表为空, 请先调用'set_goal_order'服务设置导航顺序");
    return false;
  } else {
    if (!moveBaseClientPtr_->waitForServer(ros::Duration(10.0))) {
      ROS_ERROR("没有检测到move_base服务器: %s", moveBaseName_.c_str());
      return false;
    }
    if (!startCallback_()) {
      ROS_ERROR("起始回调函数返回失败");
      return false;
    }
    mode_ = TaskState::SENDGOAL;
    ROS_INFO("开始巡航任务");
  }
  return true;
}
bool RobotCruise::Pause() {
  setlocale(LC_CTYPE, "zh_CN.utf8");
  if (GetTaskState() == TaskState::ACTIVE) {
    ROS_INFO("暂停巡航任务");
    moveBaseClientPtr_->cancelGoal();
    SetTaskState(TaskState::PAUSE);
    mode_ = TaskState::IDEL;
  } else {
    ROS_WARN("没有正在执行的任务");
    return false;
  }
  return true;
}
bool RobotCruise::Stop() {
  if (GetTaskState() == TaskState::ACTIVE) {
    moveBaseClientPtr_->cancelGoal();
  }
  currentNavingGoal_ = 0;
  SetCallBackFuncStep(0);
  loopTimes_ = requestLoop_;
  SetTaskState(TaskState::STOP);
  mode_ = TaskState::IDEL;
  ROS_INFO("停止任务");
  return true;
}
bool RobotCruise::DeleteCallBack(std::string funcName) {
  setlocale(LC_CTYPE, "zh_CN.utf8");
  if (funcName == "default") {
    ROS_WARN("default为内部已设置的默认回调函数, 不可删除");
    return false;
  } else {
    auto iter = callbackMap_.find(funcName);
    if (iter != callbackMap_.end()) {
      callbackMap_.erase(iter);
      for (auto executableIt = executableOrderCb_.begin();
           executableIt != executableOrderCb_.end(); executableIt++) {
        executableIt->second = callbackMap_["default"];
      }
    } else
      ROS_WARN("未发现回调函数%s", funcName.c_str());
  }
  ROS_INFO("删除回调函数 %s 成功", funcName.c_str());
  return true;
}
void RobotCruise::RunNavTask() {
  ros::Rate loop_rate(2);
  setlocale(LC_CTYPE, "zh_CN.utf8");
  ros::NodeHandle nh("~");
  nh.param<bool>("read_params", readParams_, false);
  if (readParams_) {
    std::string taskFile;
    ROS_INFO("read_params为true, 开始读取参数");
    nh.param<std::string>("task", taskFile, "");
    if (LoadYamlTask(taskFile) < 0) ROS_ERROR("load task error");
  }
  ros::AsyncSpinner spinner(2);
  // 开始多线程接收
  spinner.start();
  while (ros::ok()) {
    move_base_msgs::MoveBaseGoal goal;
    if (flagPublishMarker_) {
      visualization_msgs::MarkerArray landmarks;
      // 当所有目标点模型放入markerarry中
      for (auto iter = landmarkersMap_.begin(); iter != landmarkersMap_.end();
           iter++) {
        landmarks.markers.push_back(iter->second.markers[0]);
        landmarks.markers.push_back(iter->second.markers[1]);
      }
      markerPublisher_.publish(landmarks);
    }
    ROS_INFO_THROTTLE(20, "_mode_: %d, taskState_: %d", mode_, taskState_);
    switch (mode_) {             // 当执行SENDGOAL和CHECKSTATE时 taskState_
                                 // 状态都为ACTIVE
      case TaskState::SENDGOAL:  // 发送目标点
        goal.target_pose =
            goalsList_[executableOrder_[currentNavingGoal_]].target_pose;
        if (moveBaseClientPtr_->waitForServer(
                ros::Duration(10.0))) {  // 检查move_base是否运行
          ROS_INFO("发送目标点(%s)导航指令",
                   executableOrder_[currentNavingGoal_].c_str());
          moveBaseClientPtr_->sendGoal(goal);
          mode_ = TaskState::CHECKSTATE;
          taskState_ = TaskState::ACTIVE;
        } else {
          ROS_ERROR("没有检测到move_base服务器: %s", moveBaseName_.c_str());
          this->Stop();
        }
        break;
      case TaskState::CHECKSTATE:  // 检查当前目标点导航状态
        if (moveBaseClientPtr_->waitForResult(ros::Duration(1.0))) {
          if (moveBaseClientPtr_->getState() ==
              actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("goal %s reached",
                     executableOrder_[currentNavingGoal_].c_str());
            mode_ = TaskState::CALLBACK;
          } else if (moveBaseClientPtr_->getState() ==
                     actionlib::SimpleClientGoalState::PREEMPTED) {
            this->Pause();  // 如果目标点被外部其他导航指令占用, 则暂停
          } else if (moveBaseClientPtr_->getState() ==
                     actionlib::SimpleClientGoalState::ABORTED) {  // 导航失败
            if (goalsList_[executableOrder_[currentNavingGoal_]].level ==
                1) {  // 如果当前导航点为重要点, 则暂停
              this->Pause();
            } else
              mode_ = TaskState::SENDGOAL;  // 如果为可忽略点,
                                            // 则跳过开始导航到下一个目标点
          }
        }
        break;
      case TaskState::CALLBACK:
        if (!executableOrderCb_[currentNavingGoal_](callbackFuncStep_)) {
          ROS_WARN("当前点回调任务失败");
          // if (goalsList_[executableOrder_[currentNavingGoal_]].level == 1) {
          //   this->Pause();
          //   break;
          // }
          this->Pause();
        }
        SetCallBackFuncStep(0);
        if (taskState_ != TaskState::ACTIVE) break;
        if (currentNavingGoal_ < executableOrder_.size() - 1) {
          currentNavingGoal_++;
          mode_ = TaskState::SENDGOAL;  // 导航成功后开始导航到下一个目标点
        } else {
          currentNavingGoal_ = 0;
          loopTimes_--;
          if (loopTimes_ == 0) {
            this->Stop();
          } else {
            mode_ = TaskState::SENDGOAL;
          }
        }
        break;
    }
    // ros::spinOnce();
    loop_rate.sleep();
  }
}
}  // namespace rei_cruise
