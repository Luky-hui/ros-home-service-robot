cmake_minimum_required(VERSION 3.0.2)
project(map_manager)

find_package(catkin REQUIRED COMPONENTS
  #cartographer_ros_msgs
  roscpp
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#add_executable(cartographer_map_save_node src/cartographer_map_save.cpp)

#add_dependencies(cartographer_map_save_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#target_link_libraries(cartographer_map_save_node
#  ${catkin_LIBRARIES}
#)

#install(TARGETS cartographer_map_save_node
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

#foreach(dir launch maps)
#  install(DIRECTORY ${dir}/
#    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
#endforeach(dir)
