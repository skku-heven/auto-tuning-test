# [transferable-to-heven_ad]
cmake_minimum_required(VERSION 3.0.2)
project(ad_tf2_broadcaster)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  tf2
  tf2_ros
  tf2_msgs
)

catkin_package(
 INCLUDE_DIRS include
  LIBRARIES ad_tf2_broadcaster
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp tf2 tf2_ros tf2_msgs
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(ad_tf2_broadcaster 
  src/ad_tf2_broadcaster/main.cpp
  src/ad_tf2_broadcaster/tf2_broadcaster.cpp
)

target_link_libraries( ad_tf2_broadcaster ${catkin_LIBRARIES}
)

add_executable(ad_tf2_static_broadcaster 
  src/ad_tf2_static_broadcaster/main.cpp
  src/ad_tf2_static_broadcaster/tf2_static_broadcaster.cpp
)

target_link_libraries( ad_tf2_static_broadcaster ${catkin_LIBRARIES}
)
