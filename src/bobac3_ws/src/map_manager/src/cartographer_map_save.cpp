#include <ros/ros.h>
#include <cartographer_ros_msgs/FinishTrajectory.h>
#include <cartographer_ros_msgs/WriteState.h>

int main(int argc, char * argv[])
{
  ros::init(argc, argv, "cart_map_saver");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  ros::ServiceClient finishTracjClient = 
    nh.serviceClient<cartographer_ros_msgs::FinishTrajectory>("/finish_trajectory");
  ros::ServiceClient writeStateClient = 
    nh.serviceClient<cartographer_ros_msgs::WriteState>("/write_state");
  std::string mapName = "test";
  int tracjId = 0;
  bool includeUnfinishedSubmaps = false;
  pnh.param<std::string>("map_filename",mapName,mapName);
  pnh.param<int>("trajectory_id",tracjId,tracjId);
  pnh.param<bool>("include_unfinished_submaps",
      includeUnfinishedSubmaps,includeUnfinishedSubmaps);

  cartographer_ros_msgs::FinishTrajectory finishSrv;
  finishSrv.request.trajectory_id = tracjId;
  finishTracjClient.call(finishSrv);
  if(finishSrv.response.status.code!=0){
    ROS_ERROR("%s", finishSrv.response.status.message.c_str());
    return 0;
  }
  ROS_INFO("%s", finishSrv.response.status.message.c_str());

  cartographer_ros_msgs::WriteState writeSrv;
  writeSrv.request.filename = mapName;
  writeSrv.request.include_unfinished_submaps = includeUnfinishedSubmaps;
  writeStateClient.call(writeSrv);
  if(writeSrv.response.status.code!=0){
    ROS_ERROR("%s", writeSrv.response.status.message.c_str());
    return 0;
  }
  ROS_INFO("%s", writeSrv.response.status.message.c_str());
  ros::shutdown();
  return 0;
}
