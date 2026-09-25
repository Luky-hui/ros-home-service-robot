/*
 * @Author: Maintainer maintainer@example.com
 * @Date: 2022-09-01 16:17:03
 * @LastEditors: Maintainer maintainer@example.com
 * @LastEditTime: 2022-09-05 20:10:55
 * @Gitee: https://gitee.com/REINOVO
 * Copyright (c) 2022 by 深圳市元创兴科技, All Rights Reserved. 
 * @Description: 
 */
#ifndef __REINOVO_LIDAR_FUSE_NODELET_H_
#define __REINOVO_LIDAR_FUSE_NODELET_H_
#include <pluginlib/class_list_macros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <laser_geometry/laser_geometry.h>
#include <memory>
#include <thread>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/transform_listener.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>

namespace rei_lidar_fuse
{
class LidarFuse: public nodelet::Nodelet{
public:
    LidarFuse(){}
    ~LidarFuse();
    /* data */
    virtual void onInit();
    void callback(const sensor_msgs:: LaserScanConstPtr& front_msg,
                    const sensor_msgs:: LaserScanConstPtr& rear_msg);
    
private:
    message_filters::Subscriber<sensor_msgs::LaserScan> front_laser_sub;
    message_filters::Subscriber<sensor_msgs::LaserScan> rear_laser_sub;
    std::string front_frame_id_ = "front_laser_link";
    std::string rear_frame_id_ = "rear_laser_link";
    std::string frame_id_ = "laser_link";
    std::string front_topic_ = "front_lidar/scan";
    std::string rear_topic_ = "rear_lidar/scan";
    float max_angle_ = 180.0f;
    float min_angle_ = -180.0f;
    float max_range_ = 10.0f;
    float min_range_ = 0.05f;
    float angle_increment_ = 0.014f;
    float scan_time_ = 0.014f;
    ros::Publisher fuse_pub_;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan,
          sensor_msgs::LaserScan> MySyncPolicy;
    std::shared_ptr<message_filters::Synchronizer<MySyncPolicy>> sync_;

    laser_geometry::LaserProjection projector_;
    tf::TransformListener tfListener_;
};
} // namespace rei_lidar_fuse


#endif
