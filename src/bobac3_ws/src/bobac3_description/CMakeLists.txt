cmake_minimum_required(VERSION 2.8.3)

project(bobac3_description)

find_package(catkin REQUIRED)

catkin_package()

find_package(catkin REQUIRED COMPONENTS
	std_msgs
	roscpp
	geometry_msgs
	nav_msgs
	gazebo_msgs
)
catkin_package()
include_directories(
	${catkin_INCLUDE_DIRS}
)
add_executable(gazebo_kinematics src/main.cpp
	src/gazebo_kinematics.cpp
)
target_link_libraries(gazebo_kinematics
	${catkin_LIBRARIES}
)
foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
