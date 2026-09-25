cmake_minimum_required(VERSION 2.8.3)
project(ar_pose)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS 
  tf2_ros
  message_generation
  roscpp
  relative_move
  ar_track_alvar_msgs
  geometry_msgs
)

add_service_files(
  FILES
  Track.srv
)

generate_messages(
  DEPENDENCIES
)
catkin_package(
  CATKIN_DEPENDS message_runtime
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(pose_adjust src/ros_node.cpp
  src/PoseAdjust.cpp
)
add_dependencies(pose_adjust ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pose_adjust
  ${catkin_LIBRARIES}
)
install(TARGETS pose_adjust
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

foreach(dir launch srv cam_info config)
  install(DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

install(FILES
  readme.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES
  src/PoseAdjust.h
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
