cmake_minimum_required(VERSION 3.0.2)
project(local_voice_bridge)

find_package(catkin REQUIRED COMPONENTS
  rei_voice
  robot_audio
  rospy
  std_msgs
  std_srvs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/local_voice_node.py
  scripts/voice_smoke_test.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS
  scripts/start_lessons_6_10.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
