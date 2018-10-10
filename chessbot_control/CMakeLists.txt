cmake_minimum_required(VERSION 2.8.3)
project(chessbot_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)

find_package(Boost REQUIRED COMPONENTS system)

 add_service_files(
   FILES
   RG2.srv
   RG2_Grip.srv
 )

 generate_messages(
   DEPENDENCIES
   std_msgs
 )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES chessbot_control
#  CATKIN_DEPENDS roscpp std_msgs
  DEPENDS system_lib
    roscpp
    std_msgs
    message_runtime
)

include_directories(
  include ${catkin_INCLUDE_DIRS}include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)
