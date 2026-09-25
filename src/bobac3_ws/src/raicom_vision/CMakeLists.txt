cmake_minimum_required(VERSION 3.0.2)
project(raicom_vision)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  message_generation
  rospy
  sensor_msgs
  std_msgs
)

add_service_files(
  FILES
  DetectObjects.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS cv_bridge message_runtime rospy sensor_msgs std_msgs
)

catkin_install_python(PROGRAMS
  scripts/raicom_vision_node.py
  scripts/vision_smoke_test.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
