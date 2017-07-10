cmake_minimum_required(VERSION 2.8.3)
project(basicbot_utils)

find_package(catkin REQUIRED COMPONENTS
  rospy
  gazebo_ros
  message_generation
)

find_package(gazebo REQUIRED)

catkin_python_setup()

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(${GAZEBO_LIBRARY_DIRS})