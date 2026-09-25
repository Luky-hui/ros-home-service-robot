cmake_minimum_required(VERSION 3.0.2)
project(rei_lidar_fuse)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  message_filters
  nodelet
  roscpp
  sensor_msgs
  tf
  laser_geometry 
)
catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(rei_nodelet_lidar_fuse
  src/lidar_fuse_nodelet.cpp
)

add_dependencies(rei_nodelet_lidar_fuse
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(rei_nodelet_lidar_fuse
  ${catkin_LIBRARIES}
)

install(TARGETS rei_nodelet_lidar_fuse
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
foreach(dir launch)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

install(FILES
  nodelet_lidar_fuse.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
