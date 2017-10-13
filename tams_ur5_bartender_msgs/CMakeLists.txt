cmake_minimum_required(VERSION 2.8.3)
project(tams_ur5_bartender_msgs)

find_package(catkin REQUIRED
  moveit_msgs
  message_generation
)

add_message_files(
  FILES
  BarCollisionObjectArray.msg
  CocktailList.msg
)

generate_messages(
   DEPENDENCIES
   moveit_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
#  INCLUDE_DIRS include
#  LIBRARIES tams_ur5_bartender_msgs
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)
