cmake_minimum_required(VERSION 3.0.2)
project(rei_robot_cruise)

add_compile_options(-std=c++14 -fPIC)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  actionlib
  move_base_msgs
  roscpp
  rospy
  visualization_msgs
  message_generation
  tf
)

add_message_files(
  FILES
  NavGoal.msg
)


add_service_files(
  FILES
  SetGoals.srv
  SetNavOrder.srv
  StartStopNav.srv
  RemoveGoal.srv
  LoadTask.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES rei_robot_cruise
  CATKIN_DEPENDS actionlib move_base_msgs roscpp visualization_msgs message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/cruise.cpp
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  yaml-cpp
)
add_executable(example_cruise test/example.cpp)
add_dependencies(example_cruise ${${PROJECT_NAME}_EXPORTED_TARGETS} ${PROJECT_NAME})
target_link_libraries(example_cruise
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

foreach(dir msg launch params srv)
  install(DIRECTORY ${dir}/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

install(DIRECTORY include/nlohmann
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/..
)

install(FILES
  README.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

