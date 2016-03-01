cmake_minimum_required(VERSION 2.8.3)
project(robot)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
)

catkin_package()

## Build LineDetector
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(LineDetector src/LineDetector.cpp)
target_link_libraries(LineDetector ${catkin_LIBRARIES})
add_dependencies(LineDetector robot_generate_messages_cpp)