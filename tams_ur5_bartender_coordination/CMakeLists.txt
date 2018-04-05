cmake_minimum_required(VERSION 2.8.3)
project(tams_ur5_bartender_coordination)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  roscpp
  std_msgs 
  sensor_msgs
  moveit_msgs
  tams_ur5_bartender_msgs
  image_transport
  tf
  visualization_msgs
  tams_ur5_bartender_manipulation
)

add_action_files(
  DIRECTORY action
  FILES Cocktail.action
)

generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs  # Or other packages containing msgs
)

catkin_package(
	CATKIN_DEPENDS actionlib_msgs
)

include_directories(
	${catkin_INCLUDE_DIRS}
)


add_executable(bartender_coordination src/bartender_coordination.cpp)
target_link_libraries (bartender_coordination ${catkin_LIBRARIES})
add_dependencies(bartender_coordination ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
