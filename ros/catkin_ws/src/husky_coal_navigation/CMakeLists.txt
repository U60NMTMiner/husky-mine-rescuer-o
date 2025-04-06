cmake_minimum_required(VERSION 2.8.3)
project(husky_coal_navigation)

find_package(catkin REQUIRED roslaunch)

catkin_package()
catkin_add_env_hooks(60.husky_coal_navigation
  SHELLS sh DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)

roslaunch_add_file_check(launch)

install(
  DIRECTORY config launch maps
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

