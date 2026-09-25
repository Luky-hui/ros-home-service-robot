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
