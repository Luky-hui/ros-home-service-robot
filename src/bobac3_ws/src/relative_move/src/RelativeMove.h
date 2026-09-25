#ifndef REI_RELATIVE_MOVE_H__
#define REI_RELATIVE_MOVE_H__
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <pid_lib/pid.h>
#include <relative_move/SetRelativeMove.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_msgs/Float64.h>
#include <memory>
#include <mutex>
#include <thread>
namespace rei_relative_move {
class RelativeMove {
 public:
  RelativeMove();
  ~RelativeMove();
  int8_t Init(ros::NodeHandle& nh);
  void SetXPid(int p, int i, int d);
  void SetYPid(int p, int i, int d);
  void SetThetaPid(int p, int i, int d);
  void SetRobotModel(std::string mode){
    if (mode == "omni") robotModel_ = 1;
    else if(mode == "diff") robotModel_ = 0;
    else
      robotModel_ = -1;
  }
  int8_t MovXY(double x, double y);
  int8_t MovTheta(double theta);
  int8_t GetBaseToGoal(std::string frameId, const geometry_msgs::Pose2D& inGoal);

 private:
  geometry_msgs::Pose2D GetTargetGoal(const geometry_msgs::Pose2D& goal);
  void ListenTf(std::string frameId, std::string childFrameId);
  void GetTfPose(geometry_msgs::Transform& trans);
  void SetTfPose(geometry_msgs::Transform& tfPose);
  bool RelativeMoveCallback(relative_move::SetRelativeMove::Request& req,
                            relative_move::SetRelativeMove::Response& res);
  bool GetReadyFlag() {
    std::unique_lock<std::mutex> lock(getFlagMutex_);
    return getPoseReady_;
  }
  bool GetFinishFlag() {
    std::unique_lock<std::mutex> lock(getFlagMutex_);
    return moveFinished_;
  }
  void SetFinishFlag(bool state) {
    std::unique_lock<std::mutex> lock(getFlagMutex_);
    moveFinished_ = state;
  }

 private:
  ros::NodeHandle nh_;
  int8_t motionModel_;
  bool getPoseReady_;
  bool moveFinished_;
  std::thread listenTfThread_;
  geometry_msgs::Transform tfPose_;
  std::mutex getTfPoseMutex_;
  std::mutex getFlagMutex_;
  std::shared_ptr<rei_tools::ReiPID> xPid_;
  std::shared_ptr<rei_tools::ReiPID> yPid_;
  std::shared_ptr<rei_tools::ReiPID> thetaPid_;
  std::shared_ptr<tf2_ros::TransformListener> tfListener_;
  std::unique_ptr<tf2_ros::Buffer> tfBuffer_;
  ros::ServiceServer relativeMoveServer_;
  ros::Publisher velPub_, xErrPub_, yErrPub_, thetaErrPub_;
  int8_t robotModel_;
  double expectXErr_, expectYErr_, expectThetaErr_;
  geometry_msgs::Pose2D baseToTargetPose_;
  geometry_msgs::Twist stopCmd_;

};
}  // namespace rei_relative_move
#endif