cmake_minimum_required(VERSION 3.0.2)
project(toroboeye_camera)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  toroboeye_msgs
)

catkin_python_setup()
catkin_package(
   CATKIN_DEPENDS rospy sensor_msgs toroboeye_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
