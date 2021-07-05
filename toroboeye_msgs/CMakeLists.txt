cmake_minimum_required(VERSION 3.0.2)
project(toroboeye_msgs)

find_package(catkin REQUIRED COMPONENTS
  rospy
  message_generation
  std_msgs
  sensor_msgs
)

# Generate messages in the 'msg' folder
 add_message_files(
   FILES
   Device.msg
 )

## Generate services in the 'srv' folder
add_service_files(
  DIRECTORY
  srv
  FILES
  Connect.srv
  Disconnect.srv
  GetCaptureSetting.srv
  GetStatus.srv
  GetRange.srv
  SetCaptureSetting.srv
  Write.srv
  Activate.srv
  Deactivate.srv
  Capture.srv
  WaitForState.srv
  WaitForActive.srv
  WaitForInactive.srv
  WaitForFrame.srv
  Stop.srv
  GetIntrinsics.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

catkin_package(
   CATKIN_DEPENDS rospy message_generation
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

