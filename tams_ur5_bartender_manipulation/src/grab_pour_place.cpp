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
#include <stdio.h>

#include <ros/ros.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/ApplyPlanningScene.h>
#include <moveit_msgs/Grasp.h>

#include <tf/transform_datatypes.h>

#include <cmath>

const std::string ARM_ID = "arm";
const std::string GRIPPER_ID = "gripper";

class GrabPourPlace  {

	struct ObjectDescription {
		float dim[3];
		float pos[3]; 
	};

	moveit::planning_interface::MoveGroupInterface arm;
	moveit::planning_interface::MoveGroupInterface gripper;
	moveit_visual_tools::MoveItVisualToolsPtr mvt;

	protected: 
	ros::NodeHandle node_handle;
	ros::ServiceClient planning_scene_diff_client;
	ros::ServiceClient grasp_planning_service;

	
	public:
	std::map<std::string, ObjectDescription> object_map;

	GrabPourPlace() : arm(ARM_ID), gripper(GRIPPER_ID)
	{
		ObjectDescription glass = {{0, 0.03, 0.08}, {0.10, 0.10, 0.0}};
		object_map["glass"] = glass;

		ObjectDescription bottle = {{0, 0.04, 0.3}, {-0.1, -0.1, 0}};
		object_map["bottle"] = bottle;

		planning_scene_diff_client = node_handle.serviceClient<moveit_msgs::ApplyPlanningScene>("apply_planning_scene");
		planning_scene_diff_client.waitForExistence();

		//arm.setPlannerId("RRTConnectkConfigDefault");
		//arm.setPlannerId("LBKPIECEkConfigDefault");
		arm.setPlannerId("BKPIECEkConfigDefault");

		arm.setPlanningTime(20.0);
		mvt.reset(new moveit_visual_tools::MoveItVisualTools("world","/moveit_visual_markers", arm.getRobotModel()));
		mvt->loadTrajectoryPub("/move_group/display_valid_trajectory");

	}

	void despawnObject(std::string object_id){
		moveit_msgs::ApplyPlanningScene srv;
		moveit_msgs::PlanningScene planning_scene;
		planning_scene.is_diff = true;
		planning_scene.robot_state.is_diff = true;

		moveit_msgs::AttachedCollisionObject aobj;
		aobj.object.id = object_id;
		aobj.object.operation = aobj.object.REMOVE;
		planning_scene.robot_state.attached_collision_objects.push_back(aobj);

		moveit_msgs::CollisionObject object;
		object.id = object_id;
		object.operation = object.REMOVE;
		planning_scene.world.collision_objects.push_back(object);
		srv.request.scene = planning_scene;
		planning_scene_diff_client.call(srv);
	}

	moveit_msgs::CollisionObject spawnObject(std::string objectID){
		moveit_msgs::ApplyPlanningScene srv;
		moveit_msgs::PlanningScene planning_scene;
		planning_scene.is_diff = true;
		planning_scene.robot_state.is_diff = true;

		moveit_msgs::CollisionObject object;

		object.header.frame_id = "table_top";
		object.id = objectID;

		ObjectDescription objProps = object_map[objectID];

		shape_msgs::SolidPrimitive primitive;
		primitive.type = primitive.CYLINDER;
		primitive.dimensions.push_back(objProps.dim[2]);
		primitive.dimensions.push_back(objProps.dim[1]);

		geometry_msgs::Pose pose;
		pose.orientation.w = 1;
		pose.position.x = objProps.pos[0];
		pose.position.y = objProps.pos[1];
		//pose.position.z = objProps.pos[2];
		pose.position.z = objProps.dim[2] / 2;

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

	bool grab_bottle(std::string bottle_id){
		ROS_INFO_STREAM("Starting Grab Bottle with bottle : " << bottle_id);

		//open gripper
		ROS_INFO("open gripper");
		gripper.setNamedTarget("basic_open");
		gripper.move();

		ros::Duration(2.0).sleep();

		ROS_INFO("grab bottle");
		arm.setSupportSurfaceName("table");
		//Pick Bottle
		if (arm.planGraspsAndPick(bottle_id)){
			ROS_INFO("Execute pick succeded");
			return true;
		}
		else {
			ROS_ERROR("Execute pick failed");
			return false;
		}
	}

	void move_bottle(geometry_msgs::Pose pose) {
		//Create orientation constraint - bottle up
		moveit_msgs::Constraints constraints;
		/*
		moveit_msgs::OrientationConstraint ocm;
		ocm.link_name = "s_model_tool0";
		ocm.header.frame_id = "table_top";
		ocm.orientation = tf::createQuaternionMsgFromRollPitchYaw(M_PI, 0, 0);
		ocm.absolute_x_axis_tolerance = 0.05;
		ocm.absolute_y_axis_tolerance = 0.05;
		ocm.absolute_z_axis_tolerance = 2*M_PI;
		ocm.weight = 1.0;
		constraints.orientation_constraints.push_back(ocm);
		*/
		constraints.name = "s_model_tool0:upright";

		//Z offset for pose
		pose.position.z += 0.3;
		pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(M_PI, 0, 0);

		ROS_INFO_STREAM("Moving Bottle to pose: " << pose);

		//set target and constraints
		arm.setPoseReferenceFrame("table_top");
		arm.setPoseTarget(pose);
		arm.setPathConstraints(constraints);

		//move arm
		arm.move();

		//remove constraints after movement
		arm.clearPathConstraints();
	}

	bool pour_bottle(moveit_msgs::CollisionObject bottle) {
		ROS_INFO("Pour Bottle");

		arm.setPoseReferenceFrame("world");
		geometry_msgs::Pose start_pose = arm.getCurrentPose().pose;
		geometry_msgs::Pose target_pose = start_pose;

		std::vector<geometry_msgs::Pose> waypoints;
		waypoints.push_back(start_pose); // first point of trajectory - necessary?
		//pouring is described in a circular motion
		int steps = 10; //number of waypoints
		float pScale = 0.6; //max pouring angle in percent (100% is M_PI)
		float radius = bottle.primitives[0].dimensions[0] / 2; //radius is half the bottle height
		float startZ = start_pose.position.z; 
		float startY = start_pose.position.y;
		for(int i = 0; i <= steps; i++) {
			float angle = pScale * M_PI * i / steps;
			ROS_INFO_STREAM("computing waypoint for angle " << angle);
			target_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(M_PI + angle, 0, 0); //rotate around X-axis
			//linear translation described by sine and cosine
			target_pose.position.y = startY + sin(angle) * radius;
			target_pose.position.z = startZ + (cos(M_PI - angle) + 1 - 2 * 0.7 * pow((float) i / steps, 2) ) * radius;
			waypoints.push_back(target_pose);
		}

		moveit::planning_interface::MoveGroupInterface::Plan pour_forward;
		moveit::planning_interface::MoveGroupInterface::Plan pour_backward;

		moveit_msgs::RobotTrajectory trajectory;
		moveit_msgs::RobotTrajectory trajectory_back;

		// Pour forward trajectory
		double fraction = arm.computeCartesianPath(waypoints, 0.03, 3, trajectory);
		pour_forward.trajectory_ = trajectory;
		ROS_INFO("Pouring trajectory (%.2f%% acheived)", fraction * 100.0);
		arm.execute(pour_forward);

		sleep(5.0);

		// Pour back trajectory
		std::reverse(waypoints.begin(), waypoints.end());
		double fraction_back = arm.computeCartesianPath(waypoints, 0.03, 3, trajectory_back);
		pour_backward.trajectory_ = trajectory_back;
		ROS_INFO("Back movement trajectory (cartesian path) (%.2f%% acheived)", fraction_back * 100.0);
		arm.execute(pour_backward);
		sleep(3.0);
		//Return if trajectories where successfull
		return (fraction == 1.0 && fraction_back == 1.0);
	}

	void cleanup() {
		despawnObject("bottle");
		despawnObject("glass");
		arm.setNamedTarget("pour_default");
		arm.move();
	}

	void move_with_constraint(moveit_msgs::Constraints constraints) {
		arm.setPoseReferenceFrame("table_top");
		geometry_msgs::Pose pose;
		pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(M_PI, 0, 0);
		pose.position.z = 0.3;
		pose.position.x = 0.25;
		pose.position.y = -0.2;
		arm.setPoseTarget(pose);
		ROS_INFO_STREAM("Moving to pose: " << pose);
		moveit::planning_interface::MoveGroupInterface::Plan plan;
	       	arm.plan(plan);
		arm.execute(plan);

		ros::Duration(1.0).sleep();

		//set target and constraints
		pose.position.x = -0.3;
		pose.position.y = 0.3;
		arm.setPoseTarget(pose);
		arm.setPathConstraints(constraints);

		//Z offset for pose
		ROS_INFO_STREAM("Moving with constraint to pose: " << pose);

		//move arm
		arm.plan(plan);
		arm.execute(plan);

		//remove constraints after movement
		//arm.clearPathConstraints();
	}

	void move_to_start_pose(){
		arm.setPoseReferenceFrame("table_top");
		geometry_msgs::Pose pose;
		pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(M_PI, 0, 0);
		pose.position.z = 0.3;
		pose.position.x = 0.25;
		pose.position.y = -0.2;
		arm.setPoseTarget(pose);
		arm.move();
	}

	/**
	 * Plans a constrained trajectory to a fixed pose.
	 * Returns true, if the planned trajectory actually satisfies the constraints, else false.
	 */
	bool plan_with_constraints(moveit_msgs::Constraints constraints) {
		//test pose as planning target (will be set as parameter)
		geometry_msgs::Pose pose;
		pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(M_PI, 0, 0);
		pose.position.z = 0.3;
		pose.position.x = -0.3;
		pose.position.y = 0.3;
		arm.setPoseTarget(pose);

		//plan with constraints 
		arm.setPoseReferenceFrame("table_top");
		arm.setPathConstraints(constraints);
		moveit::planning_interface::MoveGroupInterface::Plan plan;
	       	arm.plan(plan);
		arm.clearPathConstraints();

		//check validity of trajectory
		bool is_valid = trajectory_valid(plan.trajectory_, constraints);
		if(is_valid) {
			ROS_INFO_STREAM("Trajectory is valid! Visualizing trajectory.");
			//visualize trajectory in rviz
			mvt->publishTrajectoryPath(plan.trajectory_, arm.getCurrentState(), false);
			ros::Duration(1.0).sleep();
			//arm.execute(plan);
		} else {
			ROS_ERROR("Trajectory is invalid!");
		}
		return is_valid;
	}

	/**
	 * Tests if a given trajectory meets a specific constraint set.
	 */
	bool trajectory_valid(moveit_msgs::RobotTrajectory trajectory, moveit_msgs::Constraints constraints) {
		kinematic_constraints::KinematicConstraintSet kset(arm.getRobotModel());
		robot_state::Transforms no_transforms(arm.getRobotModel()->getModelFrame());
		kset.add(constraints, no_transforms);
		robot_state::RobotState rstate(arm.getRobotModel());
		std::vector<trajectory_msgs::JointTrajectoryPoint> points = trajectory.joint_trajectory.points;
		//iterative validation of all waypoints in the given trajectory
		for(int i = 0; i < points.size(); i++) {
			//create robot state for waypoint
			for(int j = 0; j < points[i].positions.size(); j++) {
				//TODO: check for valid array id
				//ROS_INFO_STREAM("Checking joint: " <<  trajectory.joint_trajectory.joint_names[j] << " and position " << points[i].positions[j]);		
				rstate.setJointPositions(trajectory.joint_trajectory.joint_names[j], (double *)&points[i].positions[j]);
			}
			rstate.update();
			//return false if robot state does not satisfy constraints
			if(!kset.decide(rstate).satisfied) {
				return false;
			}
		}
		return true;
	}
};

int main(int argc, char** argv) {
	ros::init(argc, argv, "PaPTest");
	ros::AsyncSpinner spinner(1);
	spinner.start();

	GrabPourPlace task_planner;
	task_planner.move_to_start_pose();
	ros::Duration(1.0).sleep();

/*
	//Spawn bottle and glass
	moveit_msgs::CollisionObject bottle = task_planner.spawnObject("bottle");
	moveit_msgs::CollisionObject glass = task_planner.spawnObject("glass");
	geometry_msgs::Pose bottle_pose = bottle.primitive_poses[0];
	geometry_msgs::Pose glass_pose = glass.primitive_poses[0];



	if(task_planner.grab_bottle(bottle.id)) {
		//move bottle to glass
		task_planner.move_bottle(glass_pose);

		//perform pouring motion
		task_planner.pour_bottle(bottle);

		//move bottle back to bottle position
		task_planner.move_bottle(bottle_pose);
	}
	ros::Duration(3.0).sleep();
	//despawn objects and move arm to home
	task_planner.cleanup();
*/


	//constraints with name as specified in the approximation graph database
	moveit_msgs::Constraints constraints;
	moveit_msgs::OrientationConstraint ocm;
	ocm.link_name = "s_model_tool0";
	ocm.header.frame_id = "table_top";
	ocm.orientation = tf::createQuaternionMsgFromRollPitchYaw(M_PI, 0, 0);
	ocm.absolute_x_axis_tolerance = 0.6;
	ocm.absolute_y_axis_tolerance = 0.6;
	ocm.absolute_z_axis_tolerance = M_PI;
	ocm.weight = 1.0;
	constraints.orientation_constraints.push_back(ocm);
	constraints.name = "s_model_tool0:upright:15000:high";

	while(ros::ok()) {
		task_planner.plan_with_constraints(constraints);
		ros::Duration(0.2).sleep();
	}
}
