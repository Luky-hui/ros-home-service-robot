cmake_minimum_required(VERSION 3.0.2)
project(rei_voice)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

execute_process(
  COMMAND whoami
  OUTPUT_VARIABLE USERNAME
  OUTPUT_STRIP_TRAILING_WHITESPACE
)
set(AIUI_FILE_PATH "/home/${USERNAME}/.ros/REIAIUI")
if(EXISTS "${AIUI_FILE_PATH}")
  message("AIUI config file exists")
else()
  add_custom_command(OUTPUT COPY_SOURCE_config 
      COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/REIAIUI /home/${USERNAME}/.ros/REIAIUI)
  add_custom_target(Copy_config ALL DEPENDS COPY_SOURCE_config)
endif()

find_package(catkin REQUIRED COMPONENTS
  actionlib
  message_generation
  move_base_msgs
  roscpp
  rospy
  std_msgs
  std_srvs
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  REIResult.msg
  REIResultNlp.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  REITts.srv
  REIPlayer.srv
  UserData.srv
  SendEmail.srv
  REINlp.srv
  AudioRecord.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
 CATKIN_DEPENDS message_runtime
 DEPENDS 
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include/reinovo_ai_voice
  include/jsoncpp
  include/utils
  include
  ${catkin_INCLUDE_DIRS}
)

set(SystemPlatform "x86_64")
if(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "x86_64")
  set(SystemPlatform "x86_64")
elseif(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "aarch64")
  set(SystemPlatform "aarch64")
endif()

link_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/libs/${SystemPlatform}
  ${catkin_LIB_DIRS}
)

file(GLOB_RECURSE JSONCPP_FILE
        "include/jsoncpp/*.cpp")

file(GLOB_RECURSE UTILS_FILE
        "include/utils/*.cpp")

add_executable(rei_voice_node
  ${JSONCPP_FILE}
  ${UTILS_FILE}
  src/rei_voice_node.cpp
)

add_dependencies(rei_voice_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(rei_voice_node
  ${catkin_LIBRARIES}
  reinovo_key_comm
  voice_aiui
  aiui
)

install(TARGETS rei_voice_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

foreach(dir launch srv REIAIUI)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

foreach(dir libs/${SystemPlatform}/)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
endforeach(dir)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)