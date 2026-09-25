#include "RelativeMove.h"

namespace rei_relative_move {
RelativeMove::RelativeMove()
    : motionModel_(0),
      getPoseReady_(false),
      moveFinished_(true),
      robotModel_(-1) {}
RelativeMove::~RelativeMove() {
  if (listenTfThread_.joinable()) {
    listenTfThread_.join();
  }
  geometry_msgs::Twist stopCmd;
  velPub_.publish(stopCmd);
}
int8_t RelativeMove::Init(ros::NodeHandle& nh) {
  tfBuffer_ = std::make_unique<tf2_ros::Buffer>();
  tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);
  xPid_ = std::make_shared<rei_tools::ReiPID>(1.0, 0, 0.0);
  yPid_ = std::make_shared<rei_tools::ReiPID>(1.0, 0, 0.0);
  thetaPid_ = std::make_shared<rei_tools::ReiPID>(2.0, 0, 0.0);
  xPid_->setOutputLimit(0.3, 0.01);
  yPid_->setOutputLimit(0.3, 0.01);
  thetaPid_->setOutputLimit(0.5, 0.08);
  relativeMoveServer_ = nh_.advertiseService(
      "relative_move", &RelativeMove::RelativeMoveCallback, this);
  velPub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 5);
  xErrPub_ = nh_.advertise<std_msgs::Float64>("relative/x_err", 5);
  yErrPub_ = nh_.advertise<std_msgs::Float64>("relative/y_err", 5);
  thetaErrPub_ = nh_.advertise<std_msgs::Float64>("relative/theta_err", 5);
  expectXErr_ = 0.01;
  expectYErr_ = 0.01;
  expectThetaErr_ = 0.05;
  return 0;
}
void RelativeMove::SetXPid(int p, int i, int d) {
  xPid_->setP(p);
  xPid_->setI(i);
  xPid_->setD(d);
}
void RelativeMove::SetYPid(int p, int i, int d) {
  yPid_->setP(p);
  yPid_->setI(i);
  yPid_->setD(d);
}
void RelativeMove::SetThetaPid(int p, int i, int d) {
  thetaPid_->setP(p);
  thetaPid_->setI(i);
  thetaPid_->setD(d);
}
void RelativeMove::ListenTf(std::string frameId, std::string childFrameId) {
  geometry_msgs::TransformStamped transform;
  std::string errMsg;
  if (!tfBuffer_->canTransform(frameId, childFrameId, ros::Time(0),
                               ros::Duration(2.0), &errMsg)) {
    ROS_ERROR("Unable to get pose from TF: %s", errMsg.c_str());
    return;
  }
  {
    std::unique_lock<std::mutex> lock(getFlagMutex_);
    getPoseReady_ = true;
  }
  ros::Rate rate(10.0);
  while (ros::ok() && (!GetFinishFlag())) {
    try {
      transform =
          tfBuffer_->lookupTransform(frameId, childFrameId, ros::Time(0));
      SetTfPose(transform.transform);

    } catch (tf2::TransformException& ex) {
      ROS_WARN("%s", ex.what());
      ros::Duration(0.1).sleep();
      continue;
    }
    rate.sleep();
  }
}
void RelativeMove::SetTfPose(geometry_msgs::Transform& tfPose) {
  std::unique_lock<std::mutex> lock(getTfPoseMutex_);
  tfPose_ = tfPose;
}

void RelativeMove::GetTfPose(geometry_msgs::Transform& trans) {
  std::unique_lock<std::mutex> lock(getTfPoseMutex_);
  trans = tfPose_;
}

geometry_msgs::Pose2D RelativeMove::GetTargetGoal(
    const geometry_msgs::Pose2D& goal) {
  tf2::Quaternion goal_quat;
  goal_quat.setRPY(0, 0, goal.theta);
  geometry_msgs::Transform robotPose;
  GetTfPose(robotPose);
  tf2::Transform robotPoseTrans, goalPoseTrans, goalBaseRobotTrans;
  goalBaseRobotTrans.setOrigin(tf2::Vector3(goal.x, goal.y, 0));
  goalBaseRobotTrans.setRotation(goal_quat);
  geometry_msgs::Transform goalPose;
  tf2::fromMsg(robotPose, robotPoseTrans);
  goalPoseTrans = robotPoseTrans * goalBaseRobotTrans;
  geometry_msgs::Pose2D goalReuslt;
  goalReuslt.x = goalPoseTrans.getOrigin().getX();
  goalReuslt.y = goalPoseTrans.getOrigin().getY();
  tf2::Matrix3x3 mat(goalPoseTrans.getRotation());
  double roll, pitch;
  mat.getRPY(roll, pitch, goalReuslt.theta);
  return goalReuslt;
}
int8_t RelativeMove::GetBaseToGoal(std::string frameId,
                                   const geometry_msgs::Pose2D& inGoal) {
  std::string errMsg;
  if (!tfBuffer_->canTransform(frameId, "base_footprint", ros::Time(0),
                               ros::Duration(2.0), &errMsg)) {
    ROS_ERROR("Unable to get pose from TF: %s", errMsg.c_str());
    return -1;
  }
  try {
    geometry_msgs::TransformStamped transform =
        tfBuffer_->lookupTransform(frameId, "base_footprint", ros::Time(0));
    SetTfPose(transform.transform);

  } catch (tf2::TransformException& ex) {
    ROS_WARN("%s", ex.what());
    ros::Duration(0.1).sleep();
    return -1;
  }

  baseToTargetPose_ = GetTargetGoal(inGoal);
  return 0;
}

int8_t RelativeMove::MovXY(double x, double y) {
  xPid_->reset_integral();
  yPid_->reset_integral();
  geometry_msgs::Pose2D goalReal;
  geometry_msgs::Pose2D targetGoal;
  targetGoal.x = x;
  targetGoal.y = y;
  double xErr, yErr;
  ros::Rate loop(10);
  while (ros::ok()) {
    goalReal = GetTargetGoal(targetGoal);
    // ROS_WARN("goalReal: %lf, %lf", goalReal.x, goalReal.y);
    xErr = .0;
    yErr = .0;
    if (x != 0) xErr = goalReal.x;
    if (y != 0) yErr = goalReal.y;
    std_msgs::Float64 errMsg;
    errMsg.data = xErr;
    xErrPub_.publish(errMsg);

    errMsg.data = yErr;
    yErrPub_.publish(errMsg);

    // ROS_WARN("err: %lf, %lf", xErr, yErr);

    if ((fabs(xErr) < expectXErr_) && (fabs(yErr) < expectYErr_)) {
      velPub_.publish(stopCmd_);
      break;
    } else {
      geometry_msgs::Twist velCmd;
      if (fabs(xErr) > expectXErr_) velCmd.linear.x = xPid_->compute(0.0, xErr);
      if (fabs(yErr) > expectYErr_) velCmd.linear.y = yPid_->compute(0.0, yErr);
      velPub_.publish(velCmd);
    }
    loop.sleep();
  }
  return 0;
}
int8_t RelativeMove::MovTheta(double theta) {
  thetaPid_->reset_integral();
  geometry_msgs::Pose2D goalReal;
  geometry_msgs::Pose2D targetGoal;
  targetGoal.theta = theta;
  double thetaErr;
  ros::Rate loop(10);
  while (ros::ok()) {
    goalReal = GetTargetGoal(targetGoal);
    // ROS_WARN("goalReal: %lf, %lf", goalReal.x, goalReal.y);
    thetaErr = goalReal.theta;

    // ROS_WARN("thetaErr: %lf",thetaErr);

    if (fabs(thetaErr) < expectThetaErr_) {
      velPub_.publish(stopCmd_);
      break;
    } else {
      geometry_msgs::Twist velCmd;
      velCmd.angular.z = thetaPid_->compute(0.0, thetaErr);
      velPub_.publish(velCmd);
    }
    loop.sleep();
  }
  return 0;
}
bool RelativeMove::RelativeMoveCallback(
    relative_move::SetRelativeMove::Request& req,
    relative_move::SetRelativeMove::Response& res) {
  if (listenTfThread_.joinable()) {
    res.message = "last move task still run";
    return false;
  }

  getPoseReady_ = false;
  moveFinished_ = false;
  res.message = "Get tf trans error";
  if (req.global_frame.empty()) req.global_frame = "odom";
  if (GetBaseToGoal(req.global_frame, req.goal) < 0) return true;
  listenTfThread_ =
      std::thread(&RelativeMove::ListenTf, this, "base_footprint", req.global_frame);
  double lastTime = ros::Time::now().toSec();
  ros::Rate loop(10);
  while (ros::ok() && (!GetReadyFlag())) {
    if ((ros::Time::now().toSec() - lastTime) > 2) return true;
    loop.sleep();
  }
  int mode = robotModel_;
  if (robotModel_ == -1) mode = req.is_omni;
  if (mode) {
    MovXY(baseToTargetPose_.x, baseToTargetPose_.y);
    if (req.goal.theta != 0) MovTheta(baseToTargetPose_.theta);
  } else {
    geometry_msgs::Pose2D tmpPose;
    tmpPose = GetTargetGoal(req.goal);
    double tmpTheta = atan2(tmpPose.y, tmpPose.x);
    if (tmpTheta > 3.1415927)
      tmpTheta = tmpTheta - M_PI;
    else if (tmpTheta < -3.1415927)
      tmpTheta = tmpTheta + M_PI;
    MovTheta(tmpTheta);
    MovXY(baseToTargetPose_.x, 0.0);
    MovTheta(baseToTargetPose_.theta);
  }
  SetFinishFlag(true);
  if (listenTfThread_.joinable()) listenTfThread_.join();
  res.message = "";
  res.success = true;
  return true;
}
}  // namespace rei_relative_move