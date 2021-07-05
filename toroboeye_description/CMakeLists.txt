cmake_minimum_required(VERSION 3.0.2)
project(toroboeye_description)
find_package(catkin REQUIRED COMPONENTS
  rospy
  rviz
  xacro
)
catkin_package()
include_directories(
  ${catkin_INCLUDE_DIRS}
)

