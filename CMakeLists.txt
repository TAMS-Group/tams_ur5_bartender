cmake_minimum_required(VERSION 2.8.3)
project(project16_manipulation)

find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)
find_package(catkin REQUIRED genmsg 
	actionlib_msgs 
	actionlib)
add_action_files(DIRECTORY action FILES PourBottle.action)
generate_messages(DEPENDENCIES actionlib_msgs)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	std_msgs
	moveit_ros_planning_interface
	tf
)

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

add_executable(pour_bottle_test src/pouring/pour_bottle_test.cpp)
target_link_libraries(pour_bottle_test ${catkin_LIBRARIES})

add_executable(grab_pour_place src/grab_pour_place.cpp)
target_link_libraries(grab_pour_place ${catkin_LIBRARIES})

add_executable(pour_bottle_action_server src/pour_bottle_action_server.cpp)
target_link_libraries(pour_bottle_action_server ${catkin_LIBRARIES})

add_executable(pour_bottle_action_client src/pour_bottle_action_client.cpp)
target_link_libraries(pour_bottle_action_client ${catkin_LIBRARIES})
