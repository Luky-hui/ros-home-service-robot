cmake_minimum_required(VERSION 3.0.2)
project(face_rec)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  std_msgs
  sensor_msgs
)

add_message_files(
  FILES
  face_data.msg
  face_results.msg
)

add_service_files(
  FILES
  recognition_results.srv
)

generate_messages(
  DEPENDENCIES
  sensor_msgs std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
