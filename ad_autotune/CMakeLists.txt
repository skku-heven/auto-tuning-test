cmake_minimum_required(VERSION 3.0.2)
project(ad_autotune)

find_package(catkin REQUIRED)
catkin_package()

catkin_install_python(PROGRAMS
  scripts/autotune_core.py
  scripts/autotune.py
  scripts/run_live_tuning.py
  scripts/make_test_bag.py
  scripts/record_path.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY paths/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/paths
)
