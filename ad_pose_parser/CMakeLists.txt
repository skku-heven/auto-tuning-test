# [transferable-to-heven_ad]
cmake_minimum_required(VERSION 3.0.2)
project(ad_pose_parser)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  geographic_msgs
  morai_msgs
  tf
  geodesy
)

find_package(PkgConfig REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs geometry_msgs geographic_msgs morai_msgs tf geodesy
)

include_directories(
include
  ${catkin_INCLUDE_DIRS}
)

add_executable(ad_pose_parser
src/ad_pose_parser/main.cpp
src/ad_pose_parser/pose_parser.cpp
)

add_dependencies(ad_pose_parser ${catkin_EXPORTED_TARGETS})

target_link_libraries(ad_pose_parser
  ${catkin_LIBRARIES}
)

install(TARGETS ad_pose_parser
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
