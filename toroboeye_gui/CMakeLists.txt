cmake_minimum_required(VERSION 3.0.2)
project(toroboeye_gui)
find_package(catkin REQUIRED COMPONENTS
  rospy
  rqt_gui
  rqt_gui_py
  toroboeye_msgs
  toroboeye_camera
)
catkin_python_setup()
catkin_package(
   CATKIN_DEPENDS rospy rqt_gui rqt_gui_py toroboeye_msgs toroboeye_camera
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/toroboeye_gui
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  resource
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(FILES
  plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

