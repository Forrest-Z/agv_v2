cmake_minimum_required(VERSION 3.0.2)
project(agv_main)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

#add_message_files(
#  FILES
#  agv_action.msg
#)
#add_service_files(
#  FILES
#  lift_up.srv
#)
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    rospy
    std_msgs
    message_runtime
)
#catkin_install_python(PROGRAMS scripts/agv_main.py
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)