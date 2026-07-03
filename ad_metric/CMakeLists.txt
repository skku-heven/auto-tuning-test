# [transferable-to-heven_ad]
cmake_minimum_required(VERSION 3.0.2)
project(ad_metric)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  nav_msgs
  morai_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/calc_metric.py
  scripts/competition_score.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
