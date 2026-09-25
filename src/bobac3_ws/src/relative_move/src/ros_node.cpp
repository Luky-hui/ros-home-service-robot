#include "RelativeMove.h"

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "relative_move");
  rei_relative_move::RelativeMove node;
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  node.Init(nh);
  std::string robot_model;
  pnh.param<std::string>("robot_model", robot_model, "unknown");
  node.SetRobotModel(robot_model);
  ros::spin();
  // ros::shutdown();
  return 0;
}