cmake_minimum_required(VERSION 2.8.3)
project(relative_move)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  tf2_ros
  message_generation
  pid_lib
  sensor_msgs
)
add_service_files(
  FILES
  SetRelativeMove.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
)


catkin_package(
 CATKIN_DEPENDS message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
add_executable(${PROJECT_NAME}_node src/ros_node.cpp
  src/RelativeMove.cpp
)

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)


install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
foreach(dir launch srv)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
install(FILES
  readme.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(FILES
  src/RelativeMove.h
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)