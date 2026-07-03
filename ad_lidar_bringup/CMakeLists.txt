# [transferable-to-heven_ad]
cmake_minimum_required(VERSION 3.0.2)
project(ad_lidar_bringup)

find_package(catkin REQUIRED)
catkin_package()

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
