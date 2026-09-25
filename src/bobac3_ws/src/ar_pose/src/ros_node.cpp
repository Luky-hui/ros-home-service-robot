#include "PoseAdjust.h"

int main(int argc, char * argv[])
{
  ros::init(argc, argv, "ar_pose");
  rei_ar_pose::ArPoseAdjust arPoseAdjust;
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  double roll, pitch, yaw;
  pnh.param<double>("target_roll", roll, 0.0);
  pnh.param<double>("target_pitch", pitch, 0.0);
  pnh.param<double>("target_yaw", yaw, 0.0);
  arPoseAdjust.Init(nh, true, roll, pitch, yaw);
  ros::MultiThreadedSpinner s(2);
  s.spin();
  ros::shutdown();
  return 0;
}