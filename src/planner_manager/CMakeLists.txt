cmake_minimum_required(VERSION 3.2.0)
project(planner_manager)

set(CMAKE_CXX_FLAGS "-std=c++14 ${CMAKE_CXX_FLAGS} -O3")
set(CMAKE_BUILD_TYPE "Release")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  pcl_conversions
  pcl_ros
  decomp_ros_msgs
  decomp_ros_utils
  message_generation
)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)
set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})

add_message_files(
  FILES
  GapCandidate.msg
  GapCandidates.msg
)

generate_messages(
  DEPENDENCIES std_msgs geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES planner_manager
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy sensor_msgs std_msgs message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/../third_party/DecompROS/DecompUtil/include
)

add_executable(${PROJECT_NAME}_node
  src/planner_manager.cpp
  src/planner_manager_fsm.cpp
  src/free_regions_graph/free_regions_graph.cpp
  src/gap_extractor/gap_extractor.cpp
)

add_dependencies(${PROJECT_NAME}_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)
