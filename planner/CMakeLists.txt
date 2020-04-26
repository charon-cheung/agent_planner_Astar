cmake_minimum_required(VERSION 2.8.3)
project(planner)

## Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  pos_msg
  nav_msgs
)

find_package(Boost COMPONENTS system filesystem REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES planner
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_library(AStar src/AStar.cpp)

add_executable(${PROJECT_NAME}_node src/planner.cpp)
target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
   ${Boost_FILESYSTEM_LIBRARY}
   ${Boost_SYSTEM_LIBRARY}
   AStar
 )

add_executable(server_node src/server.cpp)
target_link_libraries(server_node
   ${catkin_LIBRARIES}
   ${Boost_FILESYSTEM_LIBRARY}
   ${Boost_SYSTEM_LIBRARY}
 )
