#define _USE_MATH_DEFINES
#include <iostream>
#include <stdio.h>

#include <moveit/move_group_interface/move_group.h>

#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/ApplyPlanningScene.h>
#include <moveit_msgs/Grasp.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <manipulation_msgs/GraspPlanning.h>
 
#include <cmath>
#include <ros/ros.h>
class ConMotion {

};

int main(int argc, char** argv){
    ros::init(argc, argv, "PaPTest");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit::planning_interface::MoveGroup arm("arm");
	arm.setPlannerId("RRTConnectkConfigDefault");
	arm.setPlanningTime(20.0);

   //robot_state::RobotState start_state(*arm.getCurrentState());

   // geometry_msgs::Pose start_pose; 
   // start_pose.orientation.x = arm.getCurrentPose().pose.orientation.x;
   // start_pose.orientation.y = arm.getCurrentPose().pose.orientation.y;
   // start_pose.orientation.z = arm.getCurrentPose().pose.orientation.z;
   // start_pose.orientation.w = arm.getCurrentPose().pose.orientation.w;
   // start_pose.position.x = arm.getCurrentPose().pose.position.x;
   // start_pose.position.y = arm.getCurrentPose().pose.position.y;
   // start_pose.position.z = arm.getCurrentPose().pose.position.z;
	ROS_INFO("Reference frame: %s", arm.getPlanningFrame().c_str());
	ROS_INFO("Reference frame: %s", arm.getEndEffectorLink().c_str());

    geometry_msgs::Pose target_pose; 
    geometry_msgs::Pose glass_pose;
   // target_pose.orientation.x = 0.28;
   // target_pose.orientation.y = -0.7;
   // target_pose.orientation.z = 1.0;
    target_pose.orientation.w = arm.getCurrentPose().pose.orientation.w;
   // target_pose.orientation= tf::createQuaternionMsgFromRollPitchYaw(0,0,M_PI);
    target_pose.position.x =-0.10;
    target_pose.position.y =-0.10;
    target_pose.position.z = 0.15;

    glass_pose.orientation.w = arm.getCurrentPose().pose.orientation.w; 
    glass_pose.position.x = 0.10;
    glass_pose.position.y = 0.10;
    glass_pose.position.z = 0.15;

	arm.setPoseReferenceFrame("table_top");

    //const robot_state::JointModelGroup *joint_model_group = start_state.getJointModelGroup(arm.getName());
    //start_state.setFromIK(joint_model_group, start_pose);
    //arm.setStartState(start_state);

    moveit_msgs::OrientationConstraint ocm;
    ocm.link_name = "s_model_tool0";
    ocm.header.frame_id = "table_top";
    ocm.orientation.w = 1.0;
    ocm.absolute_x_axis_tolerance = 0.05;
    ocm.absolute_y_axis_tolerance = 0.05;
    ocm.absolute_z_axis_tolerance = 2*M_PI;
    ocm.weight = 1.0;
    moveit_msgs::Constraints test_constraints;
    test_constraints.orientation_constraints.push_back(ocm);

	while(true) {
	arm.setPoseTarget(target_pose);
	arm.move();
	sleep(3);

	arm.setPoseTarget(glass_pose);
    	arm.setPathConstraints(test_constraints);
	arm.move();
	sleep(3);

	arm.clearPathConstraints();


	arm.setNamedTarget("folded");
	arm.move();
	sleep(5.0);
}
	//if(arm.plan(plan)) {
	//	sleep(5.0);

	//	ROS_INFO("Visualizing plan");
	//	moveit_msgs::DisplayTrajectory trajectory;
	//	ros::NodeHandle node_handle;
	//	ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
	//	trajectory.trajectory_start = plan.start_state_;
	//	trajectory.trajectory.push_back(plan.trajectory_);
	//	display_publisher.publish(trajectory);
	//	sleep(5.0);
	//}

}


