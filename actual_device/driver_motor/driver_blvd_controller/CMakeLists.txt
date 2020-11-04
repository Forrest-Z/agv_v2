cmake_minimum_required(VERSION 3.0.2)
project(driver_blvd_controller)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  message_generation
  diagnostic_msgs
  nav_msgs
  mbrtu
  tf
)


## Generate messages in the 'msg' folder
add_message_files(
    FILES
    speed_wheel.msg
    MLS_Measurement.msg
)

generate_messages(
   DEPENDENCIES
   std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES blvd20km_controller
   CATKIN_DEPENDS roscpp std_msgs
   CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)



#blvd20km_controller
add_executable(blvd20km_controller src/blvd20km_controller.cpp)
add_dependencies(blvd20km_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(blvd20km_controller ${catkin_LIBRARIES})

#Navigation_cotrol
add_executable(Navigation_control src/Navigation_control.cpp)
add_dependencies(Navigation_control ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(Navigation_control ${catkin_LIBRARIES})

#odometry
add_executable(odometry src/odometry.cpp)
add_dependencies(odometry ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(odometry ${catkin_LIBRARIES})

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
