cmake_minimum_required(VERSION 2.8.3)
project(project16_manipulation)

find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)
find_package(catkin REQUIRED COMPONENTS
	genmsg 
	actionlib_msgs 
	actionlib
	roscpp
	rospy
	std_msgs
	moveit_ros_planning_interface
	moveit_visual_tools
	moveit_core
	tf
)

add_action_files(DIRECTORY action FILES PourBottle.action)
generate_messages(DEPENDENCIES actionlib_msgs)

catkin_package(
)

include_directories(
	${EIGEN_INCLUDE_DIRS}
	${catkin_INCLUDE_DIRS}
)

add_executable(grab_bottle_test src/grasping/grab_bottle_test.cpp)
target_link_libraries(grab_bottle_test ${catkin_LIBRARIES})
add_dependencies(grab_bottle_test ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(grab_bottle_demo src/grasping/grab_bottle_demo.cpp)
target_link_libraries(grab_bottle_demo ${catkin_LIBRARIES})

add_executable(grasp_planning_service src/grasping/grasp_planning_service.cpp)
target_link_libraries(grasp_planning_service ${catkin_LIBRARIES})

add_executable(pour_bottle_test src/pouring/pour_bottle_test.cpp)
target_link_libraries(pour_bottle_test ${catkin_LIBRARIES})

add_executable(con_motion_test src/con_motion/con_motion_test.cpp)
target_link_libraries(con_motion_test ${catkin_LIBRARIES})

add_executable(grab_pour_place src/grab_pour_place.cpp)
target_link_libraries(grab_pour_place ${catkin_LIBRARIES})

add_executable(pour_bottle_action_server src/pour_bottle_action_server.cpp)
target_link_libraries(pour_bottle_action_server ${catkin_LIBRARIES})
add_dependencies(pour_bottle_action_server ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(pour_bottle_action_server ${catkin_EXPORTED_TARGETS})

add_executable(pour_bottle_action_client src/pour_bottle_action_client.cpp)
target_link_libraries(pour_bottle_action_client ${catkin_LIBRARIES})
add_dependencies(pour_bottle_action_client ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(pour_bottle_action_client ${catkin_EXPORTED_TARGETS})
