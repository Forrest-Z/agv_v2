cmake_minimum_required(VERSION 3.0.2)
project(plc_fx5uc)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  diagnostic_msgs
  message_generation
  mbtcp
  fx5u_hardware
  agv_msgs
)

#add_message_files(
#   FILES
#   agv_action.msg
#)
generate_messages(
   DEPENDENCIES
   std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES plc_fx5uc
   CATKIN_DEPENDS roscpp std_msgs
   CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)

#add_executable(example src/example.cpp)
#add_dependencies(example ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(example	${catkin_LIBRARIES})

add_executable(fx5uc_controller src/fx5uc_controller.cpp)
add_dependencies(fx5uc_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(fx5uc_controller	${catkin_LIBRARIES})


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)