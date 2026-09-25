#pragma once

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <gazebo_msgs/ModelStates.h>

namespace rei_gazebo_kinematics
{
class GazeboKinematics{
public:
 GazeboKinematics(ros::NodeHandle &nh);
 ~GazeboKinematics();
 void SetCar(float wheelRadius, float separation);
 void Start();

private:
 float wheelRadius_;
 float separation_;
 ros::NodeHandle nh_;
 ros::Subscriber velSub_;
 ros::Subscriber modelStateSub_;
 ros::Publisher odomPub_;
 ros::Publisher frontWheelCmdPub_;
 ros::Publisher leftWheelCmdPub_;
 ros::Publisher rightWheelCmdPub_;
 void VelCallback(const geometry_msgs::Twist::ConstPtr &vel);
 void StateCallback(const gazebo_msgs::ModelStates::ConstPtr &states);
};
}  // namespace rei_gazebo_kinematics
