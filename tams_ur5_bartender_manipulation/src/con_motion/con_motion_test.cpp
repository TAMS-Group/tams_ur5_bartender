/*
Copyright (c) 2017, Daniel Ahlers, Lars Henning Kayser, Jeremias Hartz, Maham Tanveer, Oke Martensen
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the <organization> nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#define _USE_MATH_DEFINES
#include <iostream>
#include <stdio.h>

#include <moveit/move_group_interface/move_group_interface.h>

#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/ApplyPlanningScene.h>
#include <moveit_msgs/Grasp.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <tf/transform_datatypes.h>

#include <cmath>
#include <ros/ros.h>
class ConMotion {

};

int main(int argc, char** argv){
    ros::init(argc, argv, "PaPTest");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit::planning_interface::MoveGroupInterface arm("arm");
	arm.setPlannerId("RRTConnectkConfigDefault");
	arm.setPlanningTime(20.0);

   //robot_state::RobotState start_state(*arm.getCurrentState());

	ROS_INFO("Reference frame: %s", arm.getPlanningFrame().c_str());
	ROS_INFO("End-effector frame: %s", arm.getEndEffectorLink().c_str());

    geometry_msgs::Pose target_pose; 
    geometry_msgs::Pose glass_pose;
    target_pose.orientation= tf::createQuaternionMsgFromRollPitchYaw(0,0,0);
    target_pose.position.x =-0.10;
    target_pose.position.y =-0.10;
    target_pose.position.z = 0.15;

    glass_pose.orientation= tf::createQuaternionMsgFromRollPitchYaw(0,0,0);
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

	while(ros::ok()) {
		ROS_INFO_STREAM("Move to pour_default");
		arm.clearPathConstraints();
		arm.setNamedTarget("pour_default");
		arm.move();

		ROS_INFO_STREAM("Move to target_pose");
		arm.setPoseTarget(target_pose);
		arm.move();

		ROS_INFO_STREAM("Perform constraint motion to glass_pose");
		arm.setPoseTarget(glass_pose);
		arm.setPathConstraints(test_constraints);
		arm.move();
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


