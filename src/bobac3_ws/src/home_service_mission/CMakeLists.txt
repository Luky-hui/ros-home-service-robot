cmake_minimum_required(VERSION 3.0.2)
project(home_service_mission)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  ar_pose
  face_rec
  geometry_msgs
  move_base_msgs
  raicom_vision
  robot_audio
  rospy
  std_msgs
  std_srvs
  tf
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/home_service_mission_node.py
  scripts/capture_home_waypoint.py
  scripts/home_config_check.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS
  scripts/start_home_task.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY scripts/home_service_modules
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
