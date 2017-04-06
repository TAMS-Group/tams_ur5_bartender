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
#define USE_MATH_DEFINES

#include <iostream>
#include <algorithm>
#include <stdio.h>


#include <ros/ros.h>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/ApplyPlanningScene.h>
#include <moveit_msgs/Grasp.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit/robot_trajectory/robot_trajectory.h>

#include <math.h>

moveit_msgs::CollisionObject glass_;

class PourBottleTest{
	struct objectDescription {
	    float dim[3];
	    float pos[3]; 
	};


	protected:
		ros::NodeHandle node_handle;
		ros::ServiceClient planning_scene_diff_client;
		ros::ServiceClient grasp_planning_service;

	public:
		std::map<std::string, objectDescription> object_map;

		PourBottleTest()
		{
			objectDescription glass = {{0, 0.03, 0.08}, {-0.2, 0.35, 0.04}};
			object_map["glass"] = glass;
			
			objectDescription bottle = {{0, 0.04, 0.3}, {.01, 0, 0}};// {-0.1, -0.1, 0}};
			object_map["bottle"] = bottle;
			
			planning_scene_diff_client = node_handle.serviceClient<moveit_msgs::ApplyPlanningScene>("apply_planning_scene");
			planning_scene_diff_client.waitForExistence();	
		}
		~PourBottleTest(){
		}


    moveit_msgs::CollisionObject spawnObject(std::string objectID, std::string frameID){
        moveit_msgs::ApplyPlanningScene srv;
        moveit_msgs::PlanningScene planning_scene;
        planning_scene.is_diff = true;
        planning_scene.robot_state.is_diff = true;

        moveit_msgs::CollisionObject object;

        object.header.frame_id = frameID;
        object.id = objectID;
        
        objectDescription objProps = object_map[objectID];
	
        shape_msgs::SolidPrimitive primitive;
        primitive.type = primitive.CYLINDER;
        primitive.dimensions.push_back(objProps.dim[2]);
        primitive.dimensions.push_back(objProps.dim[1]);

        geometry_msgs::Pose pose;
        pose.orientation.w = 1;
        pose.position.x = objProps.pos[0];
        pose.position.y = objProps.pos[1];
	pose.position.z = objProps.pos[2];
        //pose.position.z = primitive.dimensions[0]/2 + 0.0;

        object.primitives.push_back(primitive);
        object.primitive_poses.push_back(pose);

        // add object to scene
        object.operation = object.ADD;
        planning_scene.world.collision_objects.push_back(object);

        // remove attached object in case it is attached
     /*   moveit_msgs::AttachedCollisionObject aco;
        object.operation = object.REMOVE;
        aco.object = object;
        planning_scene.robot_state.attached_collision_objects.push_back(aco);
*/
        srv.request.scene = planning_scene;
        planning_scene_diff_client.call(srv);

        return object;
    }

	float get_bottle_height(moveit_msgs::CollisionObject bottle) {
		float height = 0.0;
		for(int i = 0; i < bottle.primitives.size(); i++) {
			height += bottle.primitives[i].dimensions[0];
		}
		return height;
	}

	std::vector<geometry_msgs::Pose> compute_pouring_waypoints(std::string axis, bool inverse_direction, moveit_msgs::CollisionObject bottle, geometry_msgs::Pose start_pose) {
		//arm.setPoseReferenceFrame("world");
		geometry_msgs::Pose target_pose = start_pose;
		std::vector<geometry_msgs::Pose> waypoints;
		waypoints.push_back(start_pose); // first point of trajectory - necessary?
		//pouring is described in a circular motion
		int dFactor = inverse_direction ? 1 : -1;
		int steps = 10; //number of waypoints
		float pScale = 0.65; //max pouring angle in percent (100% is M_PI)
		float radius =  get_bottle_height(bottle) - bottle.primitives[0].dimensions[0] / 2 ;
		float glass_radius = glass_.primitives[0].dimensions[1] / 2; //glass radius is half the glass width
		float glass_height = glass_.primitives[0].dimensions[0];
		float glass_offset = 0.0; //vertical distance of bottle tip from glass whenn pouring
		float zDistance = 0.3 + radius - glass_height / 2 - glass_offset;
		float startX = start_pose.position.x;
		float startY = start_pose.position.y;
		float startZ = start_pose.position.z;
		for(int i = 0; i <= steps; i++) {
			float angle = pScale * M_PI * i / steps;
			float tilt_factor = pow((float) i / steps, 2);
			float translation = dFactor * (sin(angle) * radius +  tilt_factor * (glass_radius + 0.005));
			//ROS_INFO_STREAM("computing waypoint for angle " << angle);
			if(axis == "Y") {
				target_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(M_PI, -dFactor * angle, 0);
				target_pose.position.x = startX + translation;

			} else { //X Axis
				target_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(M_PI + dFactor * angle, 0, 0);
				target_pose.position.y = startY + translation;
			}
			//	target_pose.position.z = startZ + (cos(M_PI - angle) + 1 - 2 * 1.1 * tilt_factor) * radius;
			//target_pose.position.z = startZ + (cos(M_PI - angle) + 1) * radius - tilt_factor * zDistance;
			target_pose.position.z = startZ + (cos(M_PI - angle) + 1) * radius - ((float) i / steps) * zDistance;
			waypoints.push_back(target_pose);
		}
		return waypoints;
	}

	moveit::planning_interface::MoveGroup::Plan get_pour_bottle_plan(moveit::planning_interface::MoveGroup& arm, moveit::core::RobotState state, geometry_msgs::Pose& start_pose, moveit_msgs::CollisionObject bottle) {
		moveit_msgs::RobotState start_state;
		moveit::core::robotStateToRobotStateMsg(state, start_state);
		arm.setStartState(start_state);
		moveit_msgs::RobotTrajectory trajectory; //Trajectory

		//Trying around Y-Axis (backwards)
		double success_percentage = arm.computeCartesianPath(compute_pouring_waypoints("Y", false, bottle, start_pose), 0.03, 3, trajectory);
		ROS_INFO("Pouring trajectory (%.2f%% achieved)", success_percentage * 100.0);

		//Try alternative, if trajectory does not reach 100%
		if(success_percentage < 0.95) {
			ROS_INFO("Trying next direction!");
			//Trying around X-Axis to the right
			moveit_msgs::RobotTrajectory temp_traj; //Alternative Trajectory
			double next_percentage = arm.computeCartesianPath(compute_pouring_waypoints("X", true, bottle, start_pose), 0.03, 3, temp_traj);
			ROS_INFO("Pouring trajectory (%.2f%% achieved)", next_percentage * 100.0);
			//If better result, take this one
			if(next_percentage > success_percentage) {
				success_percentage = next_percentage;
				trajectory = temp_traj;
			}
			//Try next, if trajectory does not reach 100%
			if(next_percentage < 0.95) {
				ROS_INFO("Trying next direction!");
				//Trying around X-Axis to the left
				next_percentage = arm.computeCartesianPath(compute_pouring_waypoints("X", false, bottle, start_pose), 0.03, 3, temp_traj);
				ROS_INFO("Pouring trajectory (%.2f%% achieved)", next_percentage * 100.0); 
				//If better result, take this one
				if(next_percentage > success_percentage) {
					success_percentage = next_percentage;
					trajectory = temp_traj;
				}
			}
		}
		arm.setStartStateToCurrentState();
		moveit::planning_interface::MoveGroup::Plan result_plan;
		if(success_percentage > 0.95) {
			result_plan.trajectory_ = trajectory;
		}
		return result_plan;
	}


void executePouring() {
}


};




int main(int argc, char** argv){
	ros::init(argc, argv, "PaPTest");
	ros::AsyncSpinner spinner(1);
	spinner.start();

	moveit::planning_interface::MoveGroup gripper("gripper");
	moveit::planning_interface::MoveGroup arm("arm");


	arm.setNamedTarget("folded");
	arm.move();
	sleep(2.0);

	arm.setPlannerId("RRTConnectkConfigDefault");
	arm.setPlanningTime(20.0);

	ROS_INFO("Starting test");
	PourBottleTest testClass;

	ROS_INFO("Spawning glass and bottle");
	glass_ = testClass.spawnObject("glass", "table_top");

	std::string endeffectorLink = arm.getEndEffectorLink().c_str();

	// waypoints for pouring motion
	geometry_msgs::Pose glass_pose;
	glass_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(M_PI, 0, 0);
	glass_pose.position.x = -0.21;
	glass_pose.position.y = 0.35;
	glass_pose.position.z = 0.33;

	arm.setPoseReferenceFrame("table_top");
	// shoulder constraint so that the arm is 'elbow up'
    //moveit_msgs::Constraints constraints;	
	//moveit_msgs::JointConstraint shoulder_const;
	//shoulder_const.joint_name = "ur5_shoulder_pan_joint";
	//shoulder_const.position = 0.0;
	//shoulder_const.tolerance_above = 2 * M_PI;
	//shoulder_const.tolerance_below = 2 * M_PI;
	//shoulder_const.weight = 1.0;
	//constraints.joint_constraints.push_back(shoulder_const);
	//arm.setPathConstraints(constraints);

	arm.setPoseTarget(glass_pose);
	arm.move();
	//arm.clearPathConstraints();
	moveit_msgs::CollisionObject bottle = testClass.spawnObject("bottle", endeffectorLink);
	gripper.attachObject("bottle", endeffectorLink);
	sleep(2);

	arm.setPoseReferenceFrame("world");
	geometry_msgs::Pose start_pose = arm.getCurrentPose().pose;

	/*

	geometry_msgs::Pose target_pose = start_pose;

	waypoints.push_back(start_pose); // first point of trajectory - necessary?

	//pouring is described in a circular motion
	int steps = 10; //number of waypoints
	float pScale = 0.6; //max pouring angle in percent (100% is M_PI)
	float radius = testClass.object_map["bottle"].dim[2] / 2; //radius is half the bottle height
	float startZ = start_pose.position.z; 
	float startY = start_pose.position.y;
	for(int i = 0; i <= steps; i++) {
		float angle = pScale * M_PI * i / steps;
		ROS_INFO_STREAM("computing waypoint for angle " << angle);
		target_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(angle, 0, 0); //rotate around X-axis
		//linear translation described by sine and cosine
		target_pose.position.y = startY + sin(angle) * radius;
		target_pose.position.z = startZ + (cos(M_PI - angle) + 1 - 2 * 0.7 * pow((float) i / steps, 2) ) * radius;
		waypoints.push_back(target_pose);
	}
	*/

	moveit::planning_interface::MoveGroup::Plan pour_forward = testClass.get_pour_bottle_plan(arm, (*arm.getCurrentState()), start_pose, bottle); 
	moveit::planning_interface::MoveGroup::Plan pour_backward;

	//reverse trajectory points
	robot_trajectory::RobotTrajectory rTrajectory(arm.getRobotModel(),"arm");

	rTrajectory.setRobotTrajectoryMsg((*arm.getCurrentState()), pour_forward.trajectory_);
	rTrajectory.reverse();

	//std::reverse(trajectory.joint_trajectory.points.begin(), trajectory.joint_trajectory.points.end());
	//std::reverse(trajectory.multi_dof_joint_trajectory.points.begin(), trajectory.multi_dof_joint_trajectory.points.end());
	//moveit::trajectory_processing::IterativeParabolicTimeParameterization::computeTimeStamps(rTrajectory);
	rTrajectory.getRobotTrajectoryMsg(pour_backward.trajectory_);

	while(ros::ok()) {
		arm.execute(pour_forward);

		sleep(5.0);

		arm.execute(pour_backward);
	}

	arm.setNamedTarget("folded");
	arm.move();

// TODO 
// define frame for the bottle opening
// place over glass using position and angle 
// using waypoints in cartesian paths: http://docs.ros.org/indigo/api/moveit_tutorials/html/doc/pr2_tutorials/planning/src/doc/move_group_interface_tutorial.html

// THINGS TO CONSIDER
// predefined trajectory, but how to control the speed? 
	

/*
	// fails when bottle added as collision object because full closing cannot be performed
	ROS_INFO("Closing gripper");
        gripper.setNamedTarget("basic_closed");
        gripper.move();
        ros::Duration(5.0).sleep();
*/

}

