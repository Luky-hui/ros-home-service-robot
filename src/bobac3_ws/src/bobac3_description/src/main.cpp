#include "gazebo_kinematics.h"

int main(int argc, char * argv[])
{
  ros::init(argc, argv, "gazebo_kinematics_node");
  ros::NodeHandle nh;
  rei_gazebo_kinematics::GazeboKinematics kinematics(nh);
  kinematics.SetCar(0.07, 0.25);
  kinematics.Start();
  return 0;
}
