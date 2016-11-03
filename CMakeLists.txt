cmake_minimum_required(VERSION 2.8.3)
project(project16_manipulation)

find_package(catkin REQUIRED COMPONENTS
	roscpp
  rospy
  std_msgs
	moveit_ros_planning_interface
	tf
)

find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)

catkin_package(
)

include_directories(
	${EIGEN_INCLUDE_DIRS}
	${catkin_INCLUDE_DIRS}
)

add_executable(grab_bottle_test src/grasping/grab_bottle_test.cpp)
target_link_libraries(grab_bottle_test ${catkin_LIBRARIES})

add_executable(grab_bottle_demo src/grasping/grab_bottle_demo.cpp)
target_link_libraries(grab_bottle_demo ${catkin_LIBRARIES})

add_executable(grasp_planning_service src/grasping/grasp_planning_service.cpp)
target_link_libraries(grasp_planning_service ${catkin_LIBRARIES})

