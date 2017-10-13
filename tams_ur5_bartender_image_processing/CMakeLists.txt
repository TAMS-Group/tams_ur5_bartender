cmake_minimum_required(VERSION 2.8.3)
project(project16_image_processing)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs 
  sensor_msgs
  moveit_msgs
  pr2016_msgs
  image_transport
  find_object_2d
  tf
  visualization_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES project16_image_processing
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)


## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(tams_pr16_bar_objects src/tams_pr16_bar_objects.cpp)
target_link_libraries (tams_pr16_bar_objects ${catkin_LIBRARIES})
add_dependencies(tams_pr16_bar_objects ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(marker_publisher src/marker_publisher.cpp)
target_link_libraries (marker_publisher ${catkin_LIBRARIES})
add_dependencies(marker_publisher ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(tams_pr16_demo_bottle_publisher src/tams_pr16_demo_bottle_publisher.cpp)
target_link_libraries (tams_pr16_demo_bottle_publisher ${catkin_LIBRARIES})
add_dependencies(tams_pr16_demo_bottle_publisher ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
