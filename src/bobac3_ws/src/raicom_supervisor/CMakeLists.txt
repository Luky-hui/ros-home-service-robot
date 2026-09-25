cmake_minimum_required(VERSION 3.0.2)
project(raicom_supervisor)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  geometry_msgs
  rospy
  std_msgs
  std_srvs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/raicom_supervisor_node.py
  scripts/raicom_acceptance_check.py
  scripts/mark_national_forbidden_zone.py
  scripts/reset_national_robot_pose.py
  scripts/sim_face_trigger.py
  scripts/trigger_national_face.py
  scripts/test_national_task1_waypoints.py
  scripts/watch_national_vision.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS
  scripts/start_competition_task.sh
  scripts/provincial_preflight.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
