cmake_minimum_required(VERSION 3.0.2)
project(linefolowing)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
)

add_message_files(
   FILES
   speed_wheel.msg
   MLS_Measurement.msg
#   agv_action.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES linefolowing
#  CATKIN_DEPENDS roscpp rospy std_msgs
   CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib 
)


#linefolowing
add_executable(linefolowersick src/linefolowersick.cpp)
add_dependencies(linefolowersick ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(linefolowersick ${catkin_LIBRARIES})

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)