#include "gazebo_kinematics.h"
namespace rei_gazebo_kinematics {
GazeboKinematics::GazeboKinematics(ros::NodeHandle& nh) : nh_(nh) {
  velSub_ = nh_.subscribe("cmd_vel", 1, &GazeboKinematics::VelCallback, this);
  modelStateSub_ = nh_.subscribe("gazebo/model_states", 1,
                                 &GazeboKinematics::StateCallback, this);
  odomPub_ = nh.advertise<nav_msgs::Odometry>("odom", 10);
  frontWheelCmdPub_ = nh.advertise<std_msgs::Float64>(
      // "/omni_bot/front_wheel_controller/command", 10);
      "front_joint_velocity_controller/command", 10);
  rightWheelCmdPub_ = nh.advertise<std_msgs::Float64>(
      // "/omni_bot/right_wheel_controller/command", 10);
      "right_joint_velocity_controller/command", 10);
  leftWheelCmdPub_ = nh.advertise<std_msgs::Float64>(
      // "/omni_bot/left_wheel_controller/command", 10);
      "left_joint_velocity_controller/command", 10);
}
GazeboKinematics::~GazeboKinematics() { ros::waitForShutdown(); }
void GazeboKinematics::SetCar(float wheelRadius, float separation) {
  this->wheelRadius_ = wheelRadius;
  this->separation_ = separation;
}
void GazeboKinematics::VelCallback(const geometry_msgs::Twist::ConstPtr& vel) {
  double motorSpeed[3] = {0};
  motorSpeed[0] = (vel->linear.y * 0.5 + vel->linear.x * 0.866025 -
                   separation_ * vel->angular.z) /
                  wheelRadius_;
  motorSpeed[1] = (vel->linear.y * 0.5 - vel->linear.x * 0.866025 -
                   separation_ * vel->angular.z) /
                  wheelRadius_;
  motorSpeed[2] =
      (-vel->linear.y - separation_ * vel->angular.z) / wheelRadius_;
  std_msgs::Float64 cmd;
  cmd.data = motorSpeed[0];
  leftWheelCmdPub_.publish(cmd);
  cmd.data = motorSpeed[1];
  rightWheelCmdPub_.publish(cmd);
  cmd.data = motorSpeed[2];
  frontWheelCmdPub_.publish(cmd);
}
void GazeboKinematics::StateCallback(
    const gazebo_msgs::ModelStates::ConstPtr& states) {
  nav_msgs::Odometry odomMsg;
  odomMsg.header.frame_id = "odom";
  odomMsg.header.stamp = ros::Time::now();
  odomMsg.child_frame_id = "base_link";
  int robotIdx = 0;
  bool hasRobot = false;
  for (std::string modelName : states->name) {
    if (modelName == "wheel") {
      hasRobot = true;
      break;
    }
    robotIdx++;
  }
  if (!hasRobot) {
    ROS_ERROR("No robot");
    return;
  }
  odomMsg.pose.pose = states->pose[robotIdx];
  odomMsg.twist.twist = states->twist[robotIdx];
  odomPub_.publish(odomMsg);
}
void GazeboKinematics::Start() {
  ros::AsyncSpinner spinner(2);
  spinner.start();
  ros::waitForShutdown();
}
}  // namespace rei_gazebo_kinematics