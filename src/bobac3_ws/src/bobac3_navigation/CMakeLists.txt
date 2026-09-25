cmake_minimum_required(VERSION 2.8.3)
project(bobac3_navigation)

catkin_package()

foreach(dir param launch rviz)
  install(DIRECTORY ${dir}/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)