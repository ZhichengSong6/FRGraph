cmake_minimum_required(VERSION 3.2.0)
project(simple_robot)

set(CMAKE_CXX_FLAGS "-std=c++14 ${CMAKE_CXX_FLAGS} -O3")
set(CMAKE_BUILD_TYPE "Release")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rospy
  sensor_msgs
  std_msgs
  tf
  roscpp
  nav_msgs
  tf
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES simple_robot
 CATKIN_DEPENDS geometry_msgs rospy sensor_msgs std_msgs tf
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/simple_robot.cpp)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)

add_executable(simple_robot_controller
  src/simple_robot_controller.cpp
)

target_link_libraries(simple_robot_controller
  ${catkin_LIBRARIES}
)