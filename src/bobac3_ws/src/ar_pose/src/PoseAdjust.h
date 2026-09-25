#ifndef REI_AR_POSE_H__
#define REI_AR_POSE_H__
#include <ar_pose/Track.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <relative_move/SetRelativeMove.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <mutex>
#include <thread>
namespace rei_ar_pose {
class ArPoseAdjust {
 public:
  ArPoseAdjust();
  ~ArPoseAdjust();
  int8_t Init(ros::NodeHandle &nh, bool getMarkerFromTopic, double targetRoll, double targetPitch, double targetYaw);
  geometry_msgs::Twist GetTargetAheadPose(
      geometry_msgs::Transform &in_transform);
  geometry_msgs::Transform GetTfPose() {
    std::unique_lock<std::mutex> lock(tf_pose_mtx_);
    return marker_pose_;
  }
  int8_t RelativeMove(double x, double y, double theta);

 private:
  bool TrackCb(ar_pose::Track::Request &req, ar_pose::Track::Response &res);
  void ListenTf(std::string frameId, std::string childFrameId);
  void SetFinishedFlag(bool state) {
    std::unique_lock<std::mutex> lock(flag_mtx_);
    track_finished_ = state;
  }
  bool GetFinishedFlag() {
    std::unique_lock<std::mutex> lock(flag_mtx_);
    return track_finished_;
  }
  bool GetTargetFlag() {
    std::unique_lock<std::mutex> lock(flag_mtx_);
    return get_target_flag_;
  }
  void SetTfPose(const geometry_msgs::Transform &pose) {
    std::unique_lock<std::mutex> lock(tf_pose_mtx_);
    marker_pose_ = pose;
  }
  int8_t FindTarget();
  int8_t GetMarkerPoseFromTopic();
  int8_t GetMarkerPoseFromTf(std::string frameId, std::string childFrameId);
  int8_t GetMarkerPose();

 private:
  ros::NodeHandle nh_;
  geometry_msgs::Transform marker_pose_;
  bool track_finished_;
  bool get_target_flag_;
  std::mutex tf_pose_mtx_;
  std::mutex flag_mtx_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  double expect_x_err_, expect_y_err_, expect_theta_err_;
  ros::ServiceServer track_server_;
  ros::ServiceClient relative_move_client_;
  geometry_msgs::Twist stop_cmd_;
  int target_id_;
  bool get_marker_from_topic_;
  double roll_, pitch_, yaw_;
};
}  // namespace rei_ar_pose

#endif