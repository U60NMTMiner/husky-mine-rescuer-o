cmake_minimum_required(VERSION 3.0.2)
project(can)

find_package(catkin REQUIRED COMPONENTS roslaunch roscpp sensor_msgs husky_msgs)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp sensor_msgs husky_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(can_node src/joy_to_can.cpp src/can.cpp)
target_link_libraries(can_node ${catkin_LIBRARIES})

install(TARGETS can_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
