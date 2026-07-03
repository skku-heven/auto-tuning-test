# [transferable-to-heven_ad]
cmake_minimum_required(VERSION 3.0.2)
project(ad_tracker)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  morai_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs nav_msgs visualization_msgs morai_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(ad_tracker
  src/ad_tracker_node.cpp
  src/gps_tracker.cpp
)

add_dependencies(ad_tracker ${catkin_EXPORTED_TARGETS})

target_link_libraries(ad_tracker
  ${catkin_LIBRARIES}
)

install(TARGETS ad_tracker
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch csv
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
