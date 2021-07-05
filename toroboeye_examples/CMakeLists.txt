cmake_minimum_required(VERSION 3.0.2)
project(toroboeye_examples)
find_package(catkin REQUIRED COMPONENTS
  rospy
  toroboeye_camera
  toroboeye_msgs
)
catkin_package(
   CATKIN_DEPENDS rospy toroboeye_camera toroboeye_msgs
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

