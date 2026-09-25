cmake_minimum_required(VERSION 3.0.2)
project(service_group_mission)

find_package(catkin REQUIRED COMPONENTS
  rospy
  actionlib
  gazebo_msgs
  geometry_msgs
  move_base_msgs
  sensor_msgs
  std_msgs
  std_srvs
  tf
  cv_bridge
  face_rec
  robot_audio
  raicom_vision
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/service_group_mission_node.py
  scripts/reset_service_scene.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS
  scripts/clean_service_sim.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
