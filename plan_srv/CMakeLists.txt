cmake_minimum_required(VERSION 2.8.3)
project(plan_srv)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  nav_msgs
  pos_msg
)

## Generate services in the 'srv' folder
 add_service_files(
   FILES
   plan.srv
   update.srv
 )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
   nav_msgs
   pos_msg
 )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pos_msg
  CATKIN_DEPENDS std_msgs pos_msg nav_msgs
#  DEPENDS system_lib
)
