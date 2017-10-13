cmake_minimum_required(VERSION 2.8.3)
project(project16_coordinator)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  roscpp
  std_msgs 
  sensor_msgs
  moveit_msgs
  pr2016_msgs
  image_transport
  tf
  visualization_msgs
  project16_manipulation
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


add_executable(tams_pr16_drink_chooser src/tams_pr16_drink_chooser.cpp)
target_link_libraries (tams_pr16_drink_chooser ${catkin_LIBRARIES})
add_dependencies(tams_pr16_drink_chooser ${catkin_EXPORTED_TARGETS})
