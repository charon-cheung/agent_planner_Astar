cmake_minimum_required(VERSION 2.8.3)
project(pos_msg)

find_package(catkin REQUIRED COMPONENTS
    roscpp
  message_generation
  std_msgs
)

 add_message_files(
   FILES
   pos.msg
 )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
 )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pos_msg
  CATKIN_DEPENDS std_msgs message_runtime
#  DEPENDS system_lib
)
