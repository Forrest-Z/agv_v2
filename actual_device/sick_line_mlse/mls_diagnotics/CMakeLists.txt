cmake_minimum_required(VERSION 3.0.2)
project(mls_diagnotics)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  diagnostic_msgs
  message_generation
)

add_message_files(
   FILES
   MLS_Measurement.msg
 )

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
)

catkin_package(

   CATKIN_DEPENDS std_msgs roscpp  
   CATKIN_DEPENDS message_runtime

)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

#set(YOUR_LIB_SOURCES
#    #include/${PROJECT_NAME}/mls_mesurement.h
#    src/diagnostic_msg.cpp
#)
#add_library(${PROJECT_NAME} ${YOUR_LIB_SOURCES})

add_executable(diagnostic_msg src/diagnostic_msg.cpp)
add_dependencies(diagnostic_msg ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(diagnostic_msg ${catkin_LIBRARIES})




