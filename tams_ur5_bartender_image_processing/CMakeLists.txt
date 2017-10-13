cmake_minimum_required(VERSION 2.8.3)
project(tams_ur5_bartender_image_processing)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs 
  sensor_msgs
  moveit_msgs
  tams_ur5_bartender_msgs
  image_transport
  find_object_2d
  tf
  visualization_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES tams_ur5_image_processing
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)


## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(bottle_recognition src/bottle_recognition.cpp)
target_link_libraries (bottle_recognition ${catkin_LIBRARIES})
add_dependencies(bottle_recognition ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(bottle_publisher src/bottle_publisher.cpp)
target_link_libraries (bottle_publisher ${catkin_LIBRARIES})
add_dependencies(bottle_publisher ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(demo_bottle_publisher src/demo_bottle_publisher.cpp)
target_link_libraries (demo_bottle_publisher ${catkin_LIBRARIES})
add_dependencies(demo_bottle_publisher ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
