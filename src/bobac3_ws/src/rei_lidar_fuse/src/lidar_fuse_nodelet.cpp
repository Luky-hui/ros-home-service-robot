#include<rei_lidar_fuse/lidar_fuse_nodelet.h>
namespace rei_lidar_fuse{
void LidarFuse::onInit(){
    
    tf::TransformListener listen;
    tf::StampedTransform transform;
    ros::NodeHandle& private_nh = getPrivateNodeHandle();
    ros::NodeHandle& nh = getNodeHandle();
    private_nh.param<std::string>("frame_id", frame_id_, "laser_link");
    private_nh.param<std::string>("front_frame_id", front_frame_id_, "front_laser_link");
    private_nh.param<std::string>("rear_frame_id", rear_frame_id_, "rear_laser_link");
    private_nh.param<std::string>("front_topic", front_topic_, "/front_lidar/scan");
    private_nh.param<std::string>("rear_topic", rear_topic_, "/rear_lidar/scan");
    private_nh.param<float>("angle_max", max_angle_, 3.1415);
    private_nh.param<float>("angle_min", min_angle_, -3.1415);
    private_nh.param<float>("range_max", max_range_, 7.0);
    private_nh.param<float>("range_min", min_range_, 0.25);
    private_nh.param<float>("angle_increment", angle_increment_, 0.014);
    private_nh.param<float>("scan_time", scan_time_, 0.02);

    fuse_pub_ = nh.advertise<sensor_msgs::LaserScan>("fuse_scan", 1);
    front_laser_sub.subscribe(nh, front_topic_.c_str(), 1);
    rear_laser_sub.subscribe(nh, rear_topic_.c_str(), 1);
    NODELET_DEBUG("Initializing nodelet...");
    sync_.reset(new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10), front_laser_sub, rear_laser_sub));
    sync_->registerCallback(boost::bind(&LidarFuse::callback, this, _1, _2));
    
}
void LidarFuse::callback(const sensor_msgs:: LaserScanConstPtr& front_msg,
                    const sensor_msgs:: LaserScanConstPtr& rear_msg){
    sensor_msgs::LaserScanPtr laser_msgs(new sensor_msgs::LaserScan);
    laser_msgs->header.stamp = front_msg->header.stamp;
    laser_msgs->header.frame_id = frame_id_;
    laser_msgs->range_max = front_msg->range_max+0.14;
    laser_msgs->range_min = 0.25;
    laser_msgs->angle_increment = std::min(front_msg->angle_increment, rear_msg->angle_increment);
    laser_msgs->time_increment = front_msg->time_increment;
    laser_msgs->scan_time = front_msg->scan_time;
    laser_msgs->angle_max = max_angle_;
    laser_msgs->angle_min = min_angle_;
    sensor_msgs::PointCloud tmpCloud1, tmpCloud2;
    std::vector<sensor_msgs::PointCloud> tmpClouds;

    int ranges_size = std::ceil((laser_msgs->angle_max - laser_msgs->angle_min) / laser_msgs->angle_increment);
    try
	{
		// Verify that TF knows how to transform from the received scan to the destination scan frame
		tfListener_.waitForTransform(front_msg->header.frame_id.c_str(), frame_id_.c_str(), front_msg->header.stamp, ros::Duration(1));
    projector_.transformLaserScanToPointCloud(frame_id_.c_str(), *front_msg, tmpCloud1, tfListener_, laser_geometry::channel_option::Intensity);
    tfListener_.waitForTransform(rear_msg->header.frame_id.c_str(), frame_id_.c_str(), rear_msg->header.stamp, ros::Duration(1));
    projector_.transformLaserScanToPointCloud(frame_id_.c_str(), *rear_msg, tmpCloud2, tfListener_, laser_geometry::channel_option::Intensity);
    }
	catch (tf::TransformException ex)
	{
		return;
	}
    tmpClouds.push_back(tmpCloud1);
    tmpClouds.push_back(tmpCloud2);
    
    laser_msgs->ranges.assign(ranges_size, 0.0);
    laser_msgs->intensities.assign(ranges_size, 0.0);
    double range_min_sq_ = laser_msgs->range_min * laser_msgs->range_min;

    for (size_t j = 0; j < tmpClouds.size(); j++)
    {
        for (size_t i = 0; i < tmpClouds[j].points.size(); i++)
        {
            const float &x = tmpClouds[j].points[i].x;
            const float &y = tmpClouds[j].points[i].y;
            const float &z = tmpClouds[j].points[i].z;

            if (std::isnan(x) || std::isnan(y) || std::isnan(z))
            {
                ROS_DEBUG("rejected for nan in point(%f, %f, %f)\n", x, y, z);
                continue;
            }
            double range_sq = pow(y, 2) + pow(x, 2);

            if (range_sq < range_min_sq_)
            {
                ROS_DEBUG("rejected for range %f below minimum value %f. Point: (%f, %f, %f)", range_sq, range_min_sq_, x, y, z);
                continue;
            }
            double angle = atan2(y, x);

            if (angle < laser_msgs->angle_min || angle > laser_msgs->angle_max)
            {
                ROS_DEBUG("rejected for angle %f not in range (%f, %f)\n", angle, laser_msgs->angle_min, laser_msgs->angle_max);
                continue;
            }
            int index = (angle - laser_msgs->angle_min) / laser_msgs->angle_increment;
            if ((pow(laser_msgs->ranges[index], 2) > range_sq)||(laser_msgs->ranges[index]==0.0)){
                double range = sqrt(range_sq);
                laser_msgs->ranges[index] = range;
                laser_msgs->intensities[index] = tmpClouds[j].channels[0].values[i];
            }

        }
    }
    fuse_pub_.publish(laser_msgs);

}
LidarFuse::~LidarFuse(){}
}
PLUGINLIB_EXPORT_CLASS(rei_lidar_fuse::LidarFuse, nodelet::Nodelet);
