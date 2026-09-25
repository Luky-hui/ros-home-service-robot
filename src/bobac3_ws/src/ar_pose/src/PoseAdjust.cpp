#include "PoseAdjust.h"

namespace rei_ar_pose {
ArPoseAdjust::ArPoseAdjust()
    : get_target_flag_(false),
      track_finished_(false),
      target_id_(-1),
      get_marker_from_topic_(true),
      roll_(0.0),
      pitch_(0.0),
      yaw_(0.0) {}
ArPoseAdjust::~ArPoseAdjust() {}
int8_t ArPoseAdjust::Init(ros::NodeHandle& nh, bool getMarkerFromTopic, double targetRoll, double targetPitch, double targetYaw) {
  nh_ = nh;
  get_marker_from_topic_ = getMarkerFromTopic;
  track_server_ = nh_.advertiseService("track", &ArPoseAdjust::TrackCb, this);
  relative_move_client_ =
      nh_.serviceClient<relative_move::SetRelativeMove>("relative_move");
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>();
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  expect_x_err_ = 0.01;
  expect_y_err_ = 0.01;
  expect_theta_err_ = 0.05;
  roll_ = targetRoll;
  pitch_ = targetPitch;
  yaw_ = targetYaw;
  return 0;
}
int8_t ArPoseAdjust::GetMarkerPoseFromTopic() {
  ar_track_alvar_msgs::AlvarMarkers::ConstPtr markers_ptr =
      ros::topic::waitForMessage<ar_track_alvar_msgs::AlvarMarkers>(
          "base_camera/ar_pose_marker", ros::Duration(1.0));
  if (markers_ptr != NULL) {
    int idx = 0;
    for (auto marker : markers_ptr->markers) {
      if (marker.id == target_id_) {
        geometry_msgs::Transform pose;
        pose.translation.x = marker.pose.pose.position.x;
        pose.translation.y = marker.pose.pose.position.y;
        pose.translation.z = marker.pose.pose.position.z;
        pose.rotation = marker.pose.pose.orientation;
        SetTfPose(pose);
        if (!GetTargetFlag()) {
          std::unique_lock<std::mutex> lock(flag_mtx_);
          get_target_flag_ = true;
        }
        break;
      }
      idx++;
    }
    if (idx == markers_ptr->markers.size()) {
      std::unique_lock<std::mutex> lock(flag_mtx_);
      get_target_flag_ = false;
      return -1;
    }
    return 0;
  }
  return -1;
}

int8_t ArPoseAdjust::GetMarkerPoseFromTf(std::string frameId,
                                         std::string childFrameId) {
  geometry_msgs::TransformStamped transform;
  try {
    transform = tf_buffer_->lookupTransform(frameId, childFrameId, ros::Time(0),
                                           ros::Duration(1.0));
    if (!GetTargetFlag()) {
      std::unique_lock<std::mutex> lock(flag_mtx_);
      get_target_flag_ = true;
    }
    SetTfPose(transform.transform);

  } catch (tf2::TransformException& ex) {
    ROS_WARN("%s", ex.what());
    {
      std::unique_lock<std::mutex> lock(flag_mtx_);
      get_target_flag_ = false;
    }
    return -1;
  }
  return 0;
}
int8_t ArPoseAdjust::GetMarkerPose() {
  if (get_marker_from_topic_)
    return GetMarkerPoseFromTopic();
  else {
    std::string childFrameId = "ar_marker_" + std::to_string(target_id_);
    return GetMarkerPoseFromTf("base_link", childFrameId);
  }
}
void ArPoseAdjust::ListenTf(std::string frameId, std::string childFrameId) {
  geometry_msgs::TransformStamped transform;
  std::string err_msg;
  if (!tf_buffer_->canTransform(frameId, childFrameId, ros::Time(0),
                               ros::Duration(2.0), &err_msg)) {
    ROS_ERROR("Unable to get pose from TF: %s", err_msg.c_str());
    return;
  }
  {
    std::unique_lock<std::mutex> lock(flag_mtx_);
    get_target_flag_ = true;
  }
  ros::Rate rate(10.0);
  while (nh_.ok() && (!GetFinishedFlag())) {
    try {
      transform =
          tf_buffer_->lookupTransform(frameId, childFrameId, ros::Time(0));
      SetTfPose(transform.transform);

    } catch (tf2::TransformException& ex) {
      ROS_WARN("%s", ex.what());
      ros::Duration(0.1).sleep();
      {
        std::unique_lock<std::mutex> lock(flag_mtx_);
        get_target_flag_ = false;
      }
      continue;
    }
    rate.sleep();
  }
}
int8_t ArPoseAdjust::RelativeMove(double x, double y, double theta) {
  relative_move::SetRelativeMove cmd;
  cmd.request.global_frame = "odom";
  cmd.request.goal.x = x;
  cmd.request.goal.y = y;
  cmd.request.goal.theta = theta;
  relative_move_client_.call(cmd);
  if (!cmd.response.success) {
    ROS_ERROR("RelativeMove error");
    return -1;
  }
  return 0;
}
int8_t ArPoseAdjust::FindTarget() {
  if (RelativeMove(0, 0, 0.25) < 0) {
    ROS_ERROR("RelativeMove error");
    return -2;
  }
  std::this_thread::sleep_for(std::chrono::seconds(1));
  if (GetMarkerPose() == 0) {
    ROS_DEBUG("Find");
    return 0;
  }
  if (RelativeMove(0, 0, -0.5) < 0) {
    ROS_ERROR("RelativeMove error");
    return -2;
  }
  std::this_thread::sleep_for(std::chrono::seconds(1));
  if (GetMarkerPose() == 0) {
    ROS_DEBUG("Find");
    return 0;
  }
  return -1;
}

geometry_msgs::Twist ArPoseAdjust::GetTargetAheadPose(
    geometry_msgs::Transform& in_transform) {
  geometry_msgs::Transform baseToTargte = GetTfPose();
  tf2::Transform base_to_targte_trans, target_to_ahead_trans, base_to_ahead_trans;
  tf2::fromMsg(baseToTargte, base_to_targte_trans);
  tf2::fromMsg(in_transform, target_to_ahead_trans);
  base_to_ahead_trans = base_to_targte_trans * target_to_ahead_trans;
  geometry_msgs::Twist out_pose;
  out_pose.linear.x = base_to_ahead_trans.getOrigin().x();
  out_pose.linear.y = base_to_ahead_trans.getOrigin().y();
  tf2::Matrix3x3 mat(base_to_ahead_trans.getRotation());
  double roll, pitch;
  mat.getRPY(roll, pitch, out_pose.angular.z);
  return out_pose;
}
bool ArPoseAdjust::TrackCb(ar_pose::Track::Request& req,
                           ar_pose::Track::Response& res) {
  geometry_msgs::Transform target_trans;
  target_trans.translation.z = req.goal_dist;
  track_finished_ = false;
  get_target_flag_ = false;
  target_id_ = req.ar_id;
  ros::Rate loop(2);
  int step = 1;
  geometry_msgs::Twist target_pose;
  tf2::Quaternion tmp_quat;
  tmp_quat.setRPY(roll_, pitch_, yaw_);
  while (nh_.ok()) {
    if (GetMarkerPose() < 0) {
      step = 0;
    } else {
      target_trans.rotation.x = tmp_quat.x();
      target_trans.rotation.y = tmp_quat.y();
      target_trans.rotation.z = tmp_quat.z();
      target_trans.rotation.w = tmp_quat.w();
      target_pose = GetTargetAheadPose(target_trans);
    }
    switch (step) {
      case 0:
        if (FindTarget() < 0) {
          res.message = "failed to find target";
          return true;
        }
        step++;
        break;
      case 1:
        if ((fabs(target_pose.linear.x) > (2.0 * req.goal_dist)) ||
            (fabs(target_pose.linear.y) > 0.1) ||
            (fabs(target_pose.angular.z) > 0.1)) {
          if (RelativeMove(target_pose.linear.x, target_pose.linear.y,
                           target_pose.angular.z) < 0) {
            res.message = "RelativeMove error";
            return true;
          }
        }
        step++;
        break;
      case 2:
        if (fabs(target_pose.angular.z) > 0.05)
          step = 4;
        else if ((fabs(target_pose.linear.x) > 0.01) ||
                 (fabs(target_pose.linear.y) > 0.01))
          step = 3;
        else
          step = 5;
        break;
      case 3:
        if (RelativeMove(target_pose.linear.x, target_pose.linear.y, 0) < 0) {
          res.message = "RelativeMove error";
          return true;
        }
        step = 2;
        break;
      case 4:
        if (RelativeMove(0, 0, target_pose.angular.z) < 0) {
          res.message = "RelativeMove error";
          return true;
        }
        step = 2;
        break;
      case 5:
        res.success = true;
        SetFinishedFlag(true);
        return true;
        break;
    }
    loop.sleep();
  }
  return true;
}

}  // namespace rei_ar_pose