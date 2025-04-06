cmake_minimum_required(VERSION 3.0.2)
project(husky_gpio_signal)

# Use find_package to get ROS dependencies
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

# Declare a catkin package
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES husky_nodedrop
#  CATKIN_DEPENDS curlcpp roscpp rospy std_msgs
#  DEPENDS system_lib
)
#catkin_add_env_hooks(60.husky_gpio_signal
#  SHELLS sh DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)

###########
## Build ##
###########

# Specify include directories
include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Add your executable and link with necessary libraries
add_executable(${PROJECT_NAME}_node src/husky_gpio_signal.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch)
endif()

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


install(DIRECTORY config launch 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

