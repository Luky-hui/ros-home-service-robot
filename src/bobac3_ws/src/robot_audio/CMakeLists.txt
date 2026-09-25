cmake_minimum_required(VERSION 3.0.2)
project(robot_audio)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
add_compile_options(-std=c++11)
execute_process(
  COMMAND whoami
  OUTPUT_VARIABLE USERNAME
  OUTPUT_STRIP_TRAILING_WHITESPACE
)
set(AIUI_FILE_PATH "/home/${USERNAME}/.ros/AIUI")
if(EXISTS "${AIUI_FILE_PATH}")
  message("AIUI config file exists")
else()
  add_custom_command(OUTPUT COPY_SOURCE_config 
      COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/AIUI /home/${USERNAME}/.ros/AIUI)
  add_custom_target(Copy_config ALL DEPENDS COPY_SOURCE_config)
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  ar_pose
  relative_move
  actionlib
  move_base_msgs
  std_srvs
)
add_service_files(
   FILES
   robot_iat.srv
   robot_semanteme.srv
   robot_tts.srv
   Collect.srv
   Awake.srv
   Control.srv
   Nav.srv
   up_sync.srv
 )

 generate_messages(
   DEPENDENCIES
   std_msgs
 )

catkin_package(
 CATKIN_DEPENDS message_runtime
)

include_directories(
  include/robot_audio
  src/jsoncpp
  src/rei_aiui
  src/zbase64
  ${catkin_INCLUDE_DIRS}
)
set(SystemPlatform "x86_64")
if(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "x86_64")
  set(SystemPlatform "x86_64")
elseif(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "aarch64")
  set(SystemPlatform "aarch64")
endif()
link_directories(
  libs/${SystemPlatform}
  ${catkin_LIB_DIRS}
)

add_library(reinovo_voice
  src/rei_aiui/reinovo_voice.cpp
  src/jsoncpp/json_reader.cpp
  src/jsoncpp/json_value.cpp
  src/jsoncpp/json_writer.cpp
  src/zbase64/zbase64.cpp
)

target_link_libraries(reinovo_voice
  ${catkin_LIBRARIES}
  aiui
)

add_executable(voice_aiui_node
  src/robot_aiui.cpp
) 
add_dependencies(voice_aiui_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(voice_aiui_node
  ${catkin_LIBRARIES}
  reinovo_voice
)
############################
##########语音采集###########
############################
add_executable(voice_collect_node 
  src/voice_collect/audio_capture.cpp
  src/voice_collect/vad.cpp
  src/voice_collect/audio_collect.cpp
  src/voice_collect/ros_wrapper.cpp
)

add_dependencies(voice_collect_node
 ${${PROJECT_NAME}_EXPORTED_TARGETS} 
 ${catkin_EXPORTED_TARGETS}
 )

target_link_libraries(voice_collect_node
 ${catkin_LIBRARIES}
 asound
)
############################
##########语音唤醒###########
############################
add_executable(voice_awake_node 
  src/voice_awake/voice_awake.cpp
)

add_dependencies(voice_awake_node
${${PROJECT_NAME}_EXPORTED_TARGETS} 
${catkin_EXPORTED_TARGETS}
)

target_link_libraries(voice_awake_node
${catkin_LIBRARIES}
)

############################
##########语音导航###########
############################
add_executable(voice_nav_node 
src/voice_nav/voice_nav.cpp
)

add_dependencies(voice_nav_node
${${PROJECT_NAME}_EXPORTED_TARGETS} 
${catkin_EXPORTED_TARGETS}
)

target_link_libraries(voice_nav_node
${catkin_LIBRARIES}
)
############################
##########语音控制###########
############################
add_executable(voice_control_node 
src/voice_control/voice_control.cpp
)
add_dependencies(voice_control_node
${${PROJECT_NAME}_EXPORTED_TARGETS} 
${catkin_EXPORTED_TARGETS}
)
target_link_libraries(voice_control_node
${catkin_LIBRARIES}
)

add_executable(voice_interaction_node 
src/interaction.cpp
)
add_dependencies(voice_interaction_node
${${PROJECT_NAME}_EXPORTED_TARGETS} 
${catkin_EXPORTED_TARGETS}
)
target_link_libraries(voice_interaction_node
${catkin_LIBRARIES}
)
install(TARGETS reinovo_voice
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(TARGETS voice_control_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(TARGETS voice_interaction_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(TARGETS voice_nav_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(TARGETS voice_awake_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(TARGETS voice_collect_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(TARGETS voice_aiui_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

foreach(dir launch srv AIUI)
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

install(FILES
  README.txt
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)