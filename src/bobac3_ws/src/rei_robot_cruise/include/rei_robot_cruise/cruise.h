/*
 * @Author: Maintainer maintainer@example.com
 * @Date: 2022-06-15 13:53:15
 * @LastEditors: Maintainer maintainer@example.com
 * @LastEditTime: 2024-04-02 15:38:48
 * @Gitee: https://gitee.com/reinovo
 * Copyright (c) 2022 by 深圳市元创兴科技, All Rights Reserved.
 * @Description:
 */
#ifndef REI_ROBOT_CRUISE_H_
#define REI_ROBOT_CRUISE_H_

#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <rei_robot_cruise/LoadTask.h>
#include <rei_robot_cruise/NavGoal.h>
#include <rei_robot_cruise/RemoveGoal.h>
#include <rei_robot_cruise/SetGoals.h>
#include <rei_robot_cruise/SetNavOrder.h>
#include <rei_robot_cruise/StartStopNav.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/MarkerArray.h>
#include <yaml-cpp/yaml.h>

#include <fstream>
#include <functional>
#include <memory>
#include <mutex>
#include <nlohmann/json.hpp>
#include <unordered_map>
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MOVEBASE;
namespace rei_cruise {
class RobotCruise {
 private:
  // void DynamicConfigCallback(robot_cruise::navigatorConfig &config, uint32_t
  // level);

  /**
   * @brief 目标点设置服务回调函数
   *
   * @param req
   * @param res
   * @return true
   * @return false
   */
  bool SetGoalsCallback(rei_robot_cruise::SetGoals::Request &req,
                        rei_robot_cruise::SetGoals::Response &res);
  /**
   * @brief 设置巡航启停服务回调函数
   *
   * @param req
   * @param res
   * @return true
   * @return false
   */
  bool StartStopGoalsCallback(rei_robot_cruise::StartStopNav::Request &req,
                              rei_robot_cruise::StartStopNav::Response &res);
  /**
   * @brief 设置巡航顺序服务回调函数
   *
   * @param req
   * @param res
   * @return true
   * @return false
   */
  bool SetGoalsOrderCallback(rei_robot_cruise::SetNavOrder::Request &req,
                             rei_robot_cruise::SetNavOrder::Response &res);

  /**
   * @brief 设置清除目标点服务回调函数
   *
   * @param req
   * @param res
   * @return true
   * @return false
   */
  bool RemoveGoalCallback(rei_robot_cruise::RemoveGoal::Request &req,
                          rei_robot_cruise::RemoveGoal::Response &res);

  /**
   * @brief 加载任务文件
   *
   * @param req
   * @param res
   * @return true
   * @return false
   */
  bool LoadTaskCallback(rei_robot_cruise::LoadTask::Request &req,
                        rei_robot_cruise::LoadTask::Response &res);
  /**
   * @brief 加载任务文件
   *
   * @param type 文件类型，0表示自动识别 1表示yaml文件 2表示json文件
   * @param taskFile 任务文件名
   * @return int8_t 0表示加载成功，-1表示正在运行任务，需手动停止
   * -2表示暂不支持该类型文件格式，-3表示自动识别文件类型失败
   */
  int8_t LoadYamlTask(const std::string &taskFile);
  int8_t LoadJsonTask(const std::string &taskFile);
  /**
   * @brief 设置在rviz中显示的目标点模型
   *
   * @param pose 模型坐标
   * @param goal_name 目标点名
   */
  void SetLandMark(geometry_msgs::Pose pose, std::string goal_name);

  void EulerToQuart(const double euler[3], geometry_msgs::Quaternion &quart) {
    quart =
        tf::createQuaternionMsgFromRollPitchYaw(euler[0], euler[1], euler[2]);
  }
  RobotCruise();
  ~RobotCruise();
  RobotCruise(const RobotCruise &other) = delete;
  RobotCruise &operator=(const RobotCruise &other) = delete;
  void SetTaskState(const int &state) {
    std::unique_lock<std::mutex> myLock(taskStateMtx_);
    taskState_ = state;
  }

 private:
  ros::NodeHandle nh_;
  std::unique_ptr<MOVEBASE>
      moveBaseClientPtr_;  // move_base客户端指针，使用智能指针只是防止野指针的出现

  int loopTimes_,
      requestLoop_;  // loop_times_：当前剩余圈数，request_loop_：目标圈数
  ros::ServiceServer setGoalsServer_, startStopNavServer_, setGoalsOrderServer_,
      removeGoalsServer_;
  ros::ServiceServer loadTaskServer_;
  ros::Publisher markerPublisher_;
  bool flagPublishMarker_;

  std::mutex taskStateMtx_;
  std::mutex modeMtx_;
  std::mutex goalInfoMtx_;
  /*
  存储目标点，格式为{目标点名字: 目标点信息}
  因大部分操作为无序查找所以选择unordered_map*/
  std::unordered_map<std::string, rei_robot_cruise::NavGoal> goalsList_;
  /*
  存储目标点巡航顺序，格式为{顺序号: 目标点名字}
  因大部分操作为顺序查找所以选择map
      */
  std::map<int, std::string> executableOrder_;

  std::function<bool()> startCallback_;

  std::map<int, std::function<bool(int)>> executableOrderCb_;

  std::unordered_map<std::string, std::function<bool(int)>> callbackMap_;

  /*存储在rviz中显示的目标点模型，格式为{目标点名字: 模型形状}*/
  std::map<std::string, visualization_msgs::MarkerArray> landmarkersMap_;

  visualization_msgs::Marker marker_;

  bool readParams_;
  std::string moveBaseName_;
  int currentNavingGoal_;
  int taskState_;  // 任务当前执行阶段
  int mode_;       // 程序目标执行阶段

  int callbackFuncStep_;

 public:
  static RobotCruise &getInstance() {
    static RobotCruise instance;
    return instance;
  }

  bool Start();
  bool Pause();
  bool Stop();

  bool Init(ros::NodeHandle &nh);
  void RunNavTask();
  inline bool AddCallBack(std::string func_name,
                          const std::function<bool(int)> &func) {
    setlocale(LC_CTYPE, "zh_CN.utf8");
    if (func_name == "default") {
      ROS_WARN("default为内部已设置的默认回调函数, 不可修改或手动添加");
      return false;
    } else {
      auto ret = callbackMap_.emplace(func_name, func);
      if (!ret.second) {
        callbackMap_.find(func_name)->second = func;
      }
    }
    ROS_INFO("设置回调函数 %s 成功", func_name.c_str());
    return true;
  }
  bool AddStartCallBack(const std::function<bool()> &func) {
    setlocale(LC_CTYPE, "zh_CN.utf8");
    startCallback_ = func;
    ROS_INFO("设置起始回调函数成功");
    return true;
  }
  bool DeleteCallBack(std::string funcName);
  inline int GetTaskState() {
    std::unique_lock<std::mutex> myLock(taskStateMtx_);
    return taskState_;
  }
  std::string GetCurrentGoalName() {
    std::unique_lock<std::mutex> myLock(goalInfoMtx_);
    return executableOrder_[currentNavingGoal_];
  }
  inline void GetCurrentGoal(geometry_msgs::Pose &goal) {
    std::unique_lock<std::mutex> myLock(goalInfoMtx_);
    std::string name = executableOrder_[currentNavingGoal_];
    goal = goalsList_[name].target_pose.pose;
  }
  inline void SetCallBackFuncStep(int step) { callbackFuncStep_ = step; }
};
enum TaskState {
  STOP = 0,
  START,
  PAUSE,
  SENDGOAL,
  CHECKSTATE,
  CALLBACK,
  ACTIVE,
  IDEL
};
}  // namespace rei_cruise
#endif
