#define USE_MATH_DEFINES
#include <project16_manipulation/PourBottleAction.h>

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <cmath>
#include <sstream>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf2/exceptions.h>
#include <actionlib/server/simple_action_server.h>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/kinematic_constraints/kinematic_constraint.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/ApplyPlanningScene.h>

#include <pr2016_msgs/BarCollisionObjectArray.h>


const std::string ARM_ID = "arm";
const std::string GRIPPER_ID = "gripper";
bool USE_BOTTLE_PUBLISHER = true;
bool OBJECTS_RECOGNIZED = true;
const pr2016_msgs::BarCollisionObjectArray* collision_objects_ = NULL;

std::map<std::string, moveit_msgs::CollisionObject> bottles_;

const int NUM_RETRIES_AFTER_JAM = 5;
const tf::TransformListener* listener = NULL;
const int planningTime = 30;

class GrabPourPlace  {

	struct ObjectDescription {
		float dim[3];
		float pos[3]; 
	};

	moveit::planning_interface::MoveGroup arm;
	moveit::planning_interface::MoveGroup gripper;

	protected: 
	ros::NodeHandle node_handle;
	ros::ServiceClient planning_scene_diff_client;
	ros::ServiceClient grasp_planning_service;
	ros::Subscriber collision_obj_sub;
	actionlib::SimpleActionServer<project16_manipulation::PourBottleAction>* as_;
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;

	public:
	std::map<std::string, ObjectDescription> object_map;

	GrabPourPlace() : arm(ARM_ID), gripper(GRIPPER_ID)
	{
		//Create dummy objects
		ObjectDescription glass = {{0, 0.04, 0.12}, {-0.20, 0.35, 0.0}};
		object_map["glass"] = glass;

		ObjectDescription bottle = {{0, 0.04, 0.3}, {-0.1, -0.1, 0}};
		object_map["bottle"] = bottle;

		//instantiate ServiceClient
		planning_scene_diff_client = node_handle.serviceClient<moveit_msgs::ApplyPlanningScene>("apply_planning_scene");
		planning_scene_diff_client.waitForExistence();

		//Register action service
		as_ = new actionlib::SimpleActionServer<project16_manipulation::PourBottleAction>(node_handle, "pour_bottle", boost::bind(&GrabPourPlace::execute, this, _1), false);
		as_->start();

		//subscribe collision objects
		collision_obj_sub = node_handle.subscribe("recognizedObjects", 1000, &GrabPourPlace::onObjectsRecognized, this);

		//configure planner
		arm.setPlannerId("RRTConnectkConfigDefault");
		arm.setPlanningTime(planningTime);
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

	moveit_msgs::CollisionObject spawnObject(std::string objectID, float xPos = -1.0, float yPos = -1.0, float zOffset = 0.0){

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
		pose.position.x = xPos != -1.0 ? xPos : objProps.pos[0];
		pose.position.y = yPos != -1.0 ? yPos : objProps.pos[1];
		//pose.position.z = objProps.pos[2];
		pose.position.z = objProps.dim[2] / 2 + zOffset;

		object.primitives.push_back(primitive);
		object.primitive_poses.push_back(pose);

		// add object to scene
		object.operation = object.ADD;

		moveit_msgs::ApplyPlanningScene srv;
		moveit_msgs::PlanningScene planning_scene;
		planning_scene.is_diff = true;
		planning_scene.robot_state.is_diff = true;
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


		ros::Duration(1.0).sleep();

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

	bool execute_plan(moveit::planning_interface::MoveGroup::Plan plan) {
		return bool(arm.execute(plan));
	}

	void set_orientation_constraint() {
		//Create orientation constraint - bottle up
		moveit_msgs::Constraints constraints;
		moveit_msgs::OrientationConstraint ocm;
		ocm.link_name = "s_model_tool0";
		ocm.header.frame_id = "table_top";
		ocm.orientation = tf::createQuaternionMsgFromRollPitchYaw(M_PI, 0, 0);
		ocm.absolute_x_axis_tolerance = M_PI / 5;
		ocm.absolute_y_axis_tolerance = M_PI / 5;
		ocm.absolute_z_axis_tolerance = 2*M_PI;
		ocm.weight = 1.0;
		constraints.orientation_constraints.push_back(ocm);
		arm.setPathConstraints(constraints);
	}

/*  obsolete orientation constraint planning
 
	moveit::planning_interface::MoveGroup::Plan get_move_bottle_plan_cartesian(std::string bottle_id, geometry_msgs::Pose pose, float zOffset) {

		//bottle up constraint
		set_orientation_constraint();

		arm.setPoseReferenceFrame("table_top");
		geometry_msgs::PoseStamped way_pose_stamped;
		try{
			listener->transformPose("table_top", ros::Time(0), arm.getCurrentPose(), "world", way_pose_stamped);
		} catch(tf2::TransformException ex) {
			ROS_ERROR("%s",ex.what()); 
		}
		std::vector<geometry_msgs::Pose> waypoints;
		way_pose_stamped.pose.position.x -= 0.2;
		way_pose_stamped.pose.position.z += zOffset / 2;
		waypoints.push_back(way_pose_stamped.pose);
		pose.position.z += zOffset;
		waypoints.push_back(pose);

		moveit_msgs::RobotTrajectory trajectory; //Trajectory
		double success = arm.computeCartesianPath(waypoints, 0.03, 3, trajectory);
		ROS_INFO("Move to glass trajectory (%.2f%% achieved)", success * 100.0);

		arm.clearPathConstraints();
		ros::Duration(2.0).sleep();

		moveit::planning_interface::MoveGroup::Plan plan;
		if(success == 1.0) {
			plan.trajectory_ = trajectory;
		}
		return plan;
	}

	moveit::planning_interface::MoveGroup::Plan get_move_bottle_plan_fixed(std::string bottle_id, std::string target) {

		//bottle up constraint
		set_orientation_constraint();

		//set target and constraints
		arm.setPoseReferenceFrame("table_top");
		arm.setNamedTarget(target);

		//arm.setPlannerId("PRMstarkConfigDefault");
		arm.setPlanningTime(planningTime);

		//move arm
		moveit::planning_interface::MoveGroup::Plan plan;
		std::vector<std::string> ids;
		ids.push_back(bottle_id);
		std::map<std::string, moveit_msgs::AttachedCollisionObject> attached_objects = planning_scene_interface_.getAttachedObjects(ids);
		moveit_msgs::AttachedCollisionObject attached_bottle = attached_objects[bottle_id];
		float xPos = -1.0;
		float yPos = -1.0;
		if(bottles_.count(bottle_id) == 1) {
			xPos = bottles_[bottle_id].primitive_poses[0].position.x;
			yPos = bottles_[bottle_id].primitive_poses[0].position.y;
		}	

		moveit_msgs::CollisionObject bottle = attached_bottle.object;
		moveit_msgs::CollisionObject stub_bottle = spawnObject("bottle", xPos, yPos, .05);

		gripper.detachObject(bottle_id);
		despawnObject(bottle_id);
		attached_bottle.object = stub_bottle;
		planning_scene_interface_.applyAttachedCollisionObject(attached_bottle);

		ros::Duration(5.0).sleep();
		arm.plan(plan);

		gripper.detachObject("bottle");
		despawnObject("bottle");
		attached_bottle.object = bottle;
		planning_scene_interface_.applyAttachedCollisionObject(attached_bottle);

		for(int i = 0; i<attached_bottle.touch_links.size(); i++) {
			ROS_INFO_STREAM("Touch link" << attached_bottle.touch_links[i]);
		}
		arm.clearPathConstraints();
		ros::Duration(2.0).sleep();
		return plan;
	}

	moveit::planning_interface::MoveGroup::Plan get_move_bottle_plan(std::string bottle_id, geometry_msgs::Pose pose, float zOffset) {

		//bottle up constraint
		set_orientation_constraint();

		//Z offset for pose
		pose.position.z += zOffset;

		ROS_INFO_STREAM("Moving Bottle to pose: " << pose);

		//set target and constraints
		arm.setPoseReferenceFrame("table_top");
		arm.setPoseTarget(pose);

		//arm.setPlannerId("PRMstarkConfigDefault");
		arm.setPlanningTime(planningTime);

		//move arm
		moveit::planning_interface::MoveGroup::Plan plan;
		std::vector<std::string> ids;
		ids.push_back(bottle_id);
		std::map<std::string, moveit_msgs::AttachedCollisionObject> attached_objects = planning_scene_interface_.getAttachedObjects(ids);
		moveit_msgs::AttachedCollisionObject attached_bottle = attached_objects[bottle_id];
		float xPos = -1.0;
		float yPos = -1.0;
		if(bottles_.count(bottle_id) == 1) {
			xPos = bottles_[bottle_id].primitive_poses[0].position.x;
			yPos = bottles_[bottle_id].primitive_poses[0].position.y;
		}	

		moveit_msgs::CollisionObject bottle = attached_bottle.object;
		moveit_msgs::CollisionObject stub_bottle = spawnObject("bottle", xPos, yPos, .05);

		gripper.detachObject(bottle_id);
		despawnObject(bottle_id);
		attached_bottle.object = stub_bottle;
		planning_scene_interface_.applyAttachedCollisionObject(attached_bottle);

		ros::Duration(5.0).sleep();
		arm.plan(plan);

		gripper.detachObject("bottle");
		despawnObject("bottle");
		attached_bottle.object = bottle;
		planning_scene_interface_.applyAttachedCollisionObject(attached_bottle);

		for(int i = 0; i<attached_bottle.touch_links.size(); i++) {
			ROS_INFO_STREAM("Touch link" << attached_bottle.touch_links[i]);
		}
		arm.clearPathConstraints();
		ros::Duration(2.0).sleep();
		return plan;
	}
*/

	moveit::planning_interface::MoveGroup::Plan get_move_bottle_plan(std::string bottle_id, geometry_msgs::Pose pose, moveit_msgs::CollisionObject glass, float zOffset) {

		//bottle up constraint (this constraint has a precomputed configuration space approximation)
		//look into moveit/moveit_planners/ompl/.../demo_construct_state_database.cpp
		//The database is located at the 'constraints_approximation_database' folder.
		//To use the database, add following param to node 'move_group' in file 'move_group.launch' inside package 'tams_ur5_setup_moveit_config'.
		//    <param name="constraint_approximations_path" value="$(find project16_manipulation)/constraints_approximation_database" />
		//
		
		//orientation constraints
		moveit_msgs::OrientationConstraint ocm;
		ocm.link_name = "s_model_tool0";
		ocm.header.frame_id = "table_top";
		ocm.orientation = tf::createQuaternionMsgFromRollPitchYaw(M_PI, 0, 0);
		ocm.absolute_x_axis_tolerance = 0.6;
		ocm.absolute_y_axis_tolerance = 0.6;
		ocm.absolute_z_axis_tolerance = M_PI;
		ocm.weight = 1.0;

		moveit_msgs::Constraints constraints;
		constraints.name = "s_model_tool0:upright:15000:high";
		constraints.orientation_constraints.push_back(ocm);
		arm.setPathConstraints(constraints);

		//Z offset for pose
		pose.position.z += zOffset;
		ROS_INFO_STREAM("Moving Bottle to pose: " << pose);

		//set target and constraints
		arm.setPoseReferenceFrame("table_top");
		//using fixed pouring position to guarantee successfull pouring
		//arm.setNamedTarget("pour_above_glass_1");
		arm.setPoseTarget(pose);

		/*
		//replace bottle collision object with primitive cylinder
		std::vector<std::string> ids;
		ids.push_back(bottle_id);
		std::map<std::string, moveit_msgs::AttachedCollisionObject> attached_objects = planning_scene_interface_.getAttachedObjects(ids);
		moveit_msgs::AttachedCollisionObject attached_bottle = attached_objects[bottle_id];
		float xPos = -1.0;
		float yPos = -1.0;
		if(bottles_.count(bottle_id) == 1) {
			xPos = bottles_[bottle_id].primitive_poses[0].position.x;
			yPos = bottles_[bottle_id].primitive_poses[0].position.y;
		}	

		moveit_msgs::CollisionObject bottle = attached_bottle.object;
		moveit_msgs::CollisionObject stub_bottle = spawnObject("bottle", xPos, yPos, .05);
		gripper.detachObject(bottle_id);
		despawnObject(bottle_id);
		attached_bottle.object = stub_bottle;
		planning_scene_interface_.applyAttachedCollisionObject(attached_bottle);
		*/

		ros::Duration(1.0).sleep();

		//start planning
		moveit::planning_interface::MoveGroup::Plan best_plan;
		bool succeeded = false;
		for(int i = 0; i < 10; i++) {
			moveit::planning_interface::MoveGroup::Plan plan;
			arm.plan(plan);
			if(trajectory_valid(plan.trajectory_, constraints)) {
				if(!succeeded) {
					best_plan = plan;
					succeeded = true;
				} else if(plan.trajectory_.joint_trajectory.points.size() < best_plan.trajectory_.joint_trajectory.points.size()){
					best_plan = plan;
				}
			}
		}
		ROS_INFO_STREAM("Planning move to glass " << (succeeded ? "SUCCEEDED!" : "FAILED!"));

		arm.clearPathConstraints();

		/*
		//reattach original bottle collision object
		gripper.detachObject("bottle");
		despawnObject("bottle");
		attached_bottle.object = bottle;
		planning_scene_interface_.applyAttachedCollisionObject(attached_bottle);
		*/

		ros::Duration(0.5).sleep();
		return best_plan;
	}


	/**
	 * Tests if a given trajectory meets a specific constraint set and is not empty
	 */
	bool trajectory_valid(moveit_msgs::RobotTrajectory trajectory, moveit_msgs::Constraints constraints) {
		std::vector<trajectory_msgs::JointTrajectoryPoint> points = trajectory.joint_trajectory.points;
		if(points.empty()) {
			return false;
		} else {
		kinematic_constraints::KinematicConstraintSet kset(arm.getRobotModel());
		robot_state::Transforms no_transforms(arm.getRobotModel()->getModelFrame());
		kset.add(constraints, no_transforms);
		moveit::core::RobotState rstate(arm.getRobotModel());
		//iterative validation of all waypoints in the given trajectory
		for(int i = 0; i < points.size(); i++) {
			//create robot state for waypoint
			for(int j = 0; j < points[i].positions.size(); j++) {
				rstate.setJointPositions(trajectory.joint_trajectory.joint_names[j], (double *)&points[i].positions[j]);
			}	
			//problems with this function?
			//moveit::core::jointTrajPointToRobotState(trajectory.joint_trajectory, (size_t) i, rstate);
			rstate.update();
			//return false if robot state does not satisfy constraints
			if(!kset.decide(rstate).satisfied) {
				return false;
			}
		}
		return true;
		}
	}

	bool can_pour(trajectory_msgs::JointTrajectory joint_trajectory, geometry_msgs::Pose pose, moveit_msgs::CollisionObject bottle, moveit_msgs::CollisionObject glass) {
		moveit::core::RobotState rstate(arm.getRobotModel());
		moveit::core::jointTrajPointToRobotState(joint_trajectory, (size_t) (joint_trajectory.points.size() - 1), rstate);
		moveit_msgs::Constraints constraints = arm.getPathConstraints();
		arm.clearPathConstraints();
		moveit::planning_interface::MoveGroup::Plan plan = get_pour_bottle_plan(rstate, pose, bottle, glass);
		arm.setPathConstraints(constraints);
		return !plan.trajectory_.joint_trajectory.points.empty();
	}

	moveit::planning_interface::MoveGroup::Plan get_pour_bottle_plan(moveit::core::RobotState state, geometry_msgs::Pose& start_pose, moveit_msgs::CollisionObject bottle, moveit_msgs::CollisionObject glass) {
		moveit_msgs::RobotState start_state;
		moveit::core::robotStateToRobotStateMsg(state, start_state);
		arm.setStartState(start_state);
		moveit_msgs::RobotTrajectory trajectory; //Trajectory

		//Trying around Y-Axis (backwards)
		double success_percentage = arm.computeCartesianPath(compute_pouring_waypoints("Y", false, bottle, glass, start_pose), 0.03, 3, trajectory);
		ROS_INFO("Pouring trajectory (%.2f%% achieved)", success_percentage * 100.0);

		//Try alternative, if trajectory does not reach 100%
		if(success_percentage < 1.0) {
			ROS_INFO("Trying next direction!");
			//Trying around X-Axis to the right
			moveit_msgs::RobotTrajectory temp_traj; //Alternative Trajectory
			double next_percentage = arm.computeCartesianPath(compute_pouring_waypoints("X", true, bottle, glass, start_pose), 0.03, 3, temp_traj);
			ROS_INFO("Pouring trajectory (%.2f%% achieved)", next_percentage * 100.0);
			//If better result, take this one
			if(next_percentage > success_percentage) {
				success_percentage = next_percentage;
				trajectory = temp_traj;
			}
			//Try next, if trajectory does not reach 100%
			if(next_percentage < 1.0) {
				ROS_INFO("Trying next direction!");
				//Trying around X-Axis to the left
				next_percentage = arm.computeCartesianPath(compute_pouring_waypoints("X", false, bottle, glass, start_pose), 0.03, 3, temp_traj);
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

	bool pour_bottle_in_glass(moveit_msgs::CollisionObject bottle, moveit_msgs::CollisionObject glass, float portion_size) {
		ROS_INFO("Pour Bottle");
		geometry_msgs::Pose start_pose = arm.getCurrentPose().pose;
		moveit::planning_interface::MoveGroup::Plan pour_forward = get_pour_bottle_plan((*arm.getCurrentState()), start_pose, bottle, glass); 
		bool success = !pour_forward.trajectory_.joint_trajectory.points.empty();
		if(success) {
			//reverse pouring trajectory
			moveit::planning_interface::MoveGroup::Plan pour_backward;
			reverse_trajectory(pour_forward.trajectory_, pour_backward.trajectory_);
			ros::Duration(0.5).sleep();

			if(!arm.execute(pour_forward)) {
				//TODO: handle failure
				return false;
			}

			sleep(std::max((float)0.0, std::min(portion_size, (float)5.0)));

			if(!arm.execute(pour_backward)) {
				//TODO: handle failure
				return false;
			}
		}			
		return success;
	}

	std::vector<geometry_msgs::Pose> compute_pouring_waypoints(std::string axis, bool inverse_direction, moveit_msgs::CollisionObject bottle, moveit_msgs::CollisionObject glass, geometry_msgs::Pose start_pose) {
		arm.setPoseReferenceFrame("world");
		geometry_msgs::Pose target_pose = start_pose;
		std::vector<geometry_msgs::Pose> waypoints;
		waypoints.push_back(start_pose); // first point of trajectory - necessary?
		//pouring is described in a circular motion
		int dFactor = inverse_direction ? 1 : -1;
		int steps = 10; //number of waypoints
		float pScale = 0.65; //max pouring angle in percent (100% is M_PI)
		float radius =  get_bottle_height(bottle.id) - bottle.primitives[0].dimensions[0] / 2 ;
		float glass_radius = glass.primitives[0].dimensions[1] / 2; //glass radius is half the glass width
		float glass_height = glass.primitives[0].dimensions[0]; 
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

	void cleanup() {
		ROS_INFO("Cleanup");
		for(std::map<std::string, moveit_msgs::CollisionObject>::iterator it = bottles_.begin(); it != bottles_.end(); ++it) {
			despawnObject(it->first);
		}
		bottles_.clear(); 
		despawnObject("glass");
		despawnObject("bottle");
		gripper.setNamedTarget("basic_open");
		gripper.move();
		arm.setNamedTarget("pour_default");
		arm.move();
	}

	bool placeBottle(std::string bottle_id) {
		ROS_INFO("Placing Bottle");

		geometry_msgs::PoseStamped place_pose_stamped;
		try{
			listener->transformPose("table_top", ros::Time(0), arm.getCurrentPose(), "world", place_pose_stamped);
		} catch(tf2::TransformException ex) {
			ROS_ERROR("%s",ex.what()); 
			return false;
		}

		arm.setPoseReferenceFrame("table_top");
		geometry_msgs::Pose place_pose = place_pose_stamped.pose;
		moveit_msgs::CollisionObject bottle = bottles_[bottle_id];
		float grasp_height = bottle.primitives[0].dimensions[0] / 2;
		place_pose.position.z = grasp_height + 0.005; // table distance and bottle height offset
		ROS_INFO_STREAM(place_pose);

		//move bottle down
		std::vector<geometry_msgs::Pose> waypoints;
		waypoints.push_back(place_pose);
		moveit::planning_interface::MoveGroup::Plan plan;
		moveit_msgs::RobotTrajectory trajectory;
		double fraction = arm.computeCartesianPath(waypoints, 0.03, 3, trajectory);
		ROS_INFO("Place Down trajectory (%.2f%% acheived)", fraction * 100.0);
		plan.trajectory_ = trajectory;
		if(fraction!=1.0 || !bool(arm.execute(plan))) {
			return false;
		}

		ros::Duration(0.5).sleep();

		//open gripper
		gripper.setNamedTarget("basic_open");
		ROS_INFO("Setting gripper to target: basic_open");
		if(!gripper.move()) {
			return false;
		}

		ros::Duration(0.5).sleep();

		//release object
		despawnObject(bottle_id);
		if(bottles_.count(bottle_id) == 1) {
			ROS_INFO("Respawning bottle");
			bottle = bottles_[bottle_id];
			for(int i = 0; i < bottle.primitive_poses.size(); i++) {
				bottle.primitive_poses[i].position.x = place_pose.position.x;
				bottle.primitive_poses[i].position.y = place_pose.position.y;
			}
			bottle.operation = bottle.ADD;
			planning_scene_interface_.applyCollisionObject(bottle); 
		} else {
			bottle = spawnObject(bottle_id);
		}

		ros::Duration(0.5).sleep();

		//move to retreat position
		geometry_msgs::Pose post_place_pose = place_pose;
		post_place_pose.position.x -= 0.15;
		post_place_pose.position.z += 0.15;
		//post_place_pose.position.z += 0.1;
		ROS_INFO_STREAM("Retreating pose" << post_place_pose);
		std::vector<geometry_msgs::Pose> retreat_waypoints;
		retreat_waypoints.push_back(post_place_pose);
		moveit::planning_interface::MoveGroup::Plan retreat_plan;
		moveit_msgs::RobotTrajectory retreat_trajectory;
		double retreat_fraction = arm.computeCartesianPath(retreat_waypoints, 0.03, 3, retreat_trajectory);
		retreat_plan.trajectory_ = retreat_trajectory;
		if(retreat_fraction > 0.3)
			return bool(arm.execute(retreat_plan));
		else
			return false;
	}

	float get_bottle_height(std::string bottle_id) {
		moveit_msgs::CollisionObject bottle = bottles_[bottle_id];
		float height = 0.0;
		for(int i = 0; i < bottle.primitives.size(); i++) {
			height += bottle.primitives[i].dimensions[0];
		}
		return height;
	}

	bool move_back() {
		arm.setNamedTarget("pour_default");
		return bool(arm.move());
	}

	void reverse_trajectory(moveit_msgs::RobotTrajectory &forward, moveit_msgs::RobotTrajectory &backward){
		robot_trajectory::RobotTrajectory rTrajectory(arm.getRobotModel(),"arm");
		rTrajectory.setRobotTrajectoryMsg((*arm.getCurrentState()), forward); 
		rTrajectory.reverse();
		trajectory_processing::IterativeParabolicTimeParameterization iptp;
		iptp.computeTimeStamps(rTrajectory);
		rTrajectory.getRobotTrajectoryMsg(backward);
	}

	void onObjectsRecognized(const pr2016_msgs::BarCollisionObjectArray& objArray)
	{
		if(!OBJECTS_RECOGNIZED) {
			OBJECTS_RECOGNIZED = true;
			bottles_.clear();
			ROS_INFO("creating collision objects");
			for(int i = 0; i<objArray.objects.size(); i++) {
				// add object to scene
				moveit_msgs::CollisionObject object = objArray.objects[i];
				object.operation = object.ADD;
				ROS_INFO_STREAM("Bottle " << object.type.key);
				object.id = object.type.key;
				planning_scene_interface_.applyCollisionObject(object); 
				bottles_[object.id] = object;
			}
		}
	}

	void recognizeBottles() 
	{
		OBJECTS_RECOGNIZED = false;
		int retrys = 20;
		while(!OBJECTS_RECOGNIZED && retrys > 0) {
			ros::Duration(1.0).sleep();
			retrys--;
		}
	}

	void execute(const project16_manipulation::PourBottleGoalConstPtr& goal)
	{
		ROS_INFO("Running pour bottle!");

		cleanup();

		std::string bottle_id = goal->bottle_id;
		float portion_size = goal->portion_size;

		moveit_msgs::CollisionObject bottle;

		project16_manipulation::PourBottleFeedback feedback;
		if(USE_BOTTLE_PUBLISHER) {
			recognizeBottles();
			ros::Duration(1.0).sleep();
			if(bottles_.count(bottle_id) == 1) {
				bottle = bottles_[bottle_id];
			}
			else {
				feedback.task_state = std::string("Bottle " + bottle.id + " does not exist!");
				as_->publishFeedback(feedback);
				as_->setAborted();
				return;
			}
		} else {
			//Spawn bottle
			bottle = spawnObject("bottle");
		}
		ROS_INFO_STREAM("Bottles Recognized");
		geometry_msgs::Pose bottle_pose = bottle.primitive_poses[0];
		bottle_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(M_PI, 0, -M_PI/2);
		moveit_msgs::CollisionObject glass = spawnObject("glass");
		geometry_msgs::Pose glass_pose = glass.primitive_poses[0];
		glass_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(M_PI, 0, 0);
		feedback.task_state = "Objects spawned";
		as_->publishFeedback(feedback);

		project16_manipulation::PourBottleResult result;
		result.success = pouringStateMachine( bottle, glass, glass_pose , portion_size);

		if(result.success) {
			as_->setSucceeded(result,"Grasping, constrained motion and pouring succeeded");
		} else {
			as_->setAborted();
		}
	}

	bool pouringStateMachine( moveit_msgs::CollisionObject bottle, moveit_msgs::CollisionObject glass, geometry_msgs::Pose glass_pose, float portion_size) 
	{
		int CurrentAttempt = 1;
		enum stateSpace { pickBottleUp, moveBottleToGlass, pourIntoGlass, moveBottleBack, placeBottleDown, moveArmToDefault, run_cleanup, Exit, OK };
		stateSpace state = pickBottleUp;
		stateSpace failedAtState = OK;

		moveit::planning_interface::MoveGroup::Plan move_bottle_back;
		moveit::planning_interface::MoveGroup::Plan move_bottle;
		
		do {
			switch(state) {

				case pickBottleUp:
					ROS_INFO_STREAM("--- STATE 1: pickBottleUp; Attempt " << CurrentAttempt);
					// if SUCCESS, move on to next state, else check number of attempts and either retry state or abort and skip states
					if ( grab_bottle(bottle.id) ) {
						ROS_INFO_STREAM("pickBottleUp succeeded with attempt " << CurrentAttempt);
						ros::Duration(0.5).sleep();
						state = moveBottleToGlass;
						CurrentAttempt = 1;
					} else {
						if ( CurrentAttempt++ == NUM_RETRIES_AFTER_JAM+1 ) {
							ROS_INFO_STREAM("XXX STATE 1: pickBottleUp FAILED after all " << CurrentAttempt << " attempts. Moving on to cleanup.");
							state = run_cleanup;
							failedAtState = pickBottleUp;
						}
					}
					break;


					/*
				case moveBottleToGlass:
					ROS_INFO_STREAM("--- STATE 2: moveBottleToGlass; (planningTime: " << planningTime << ") ; Attempt " << CurrentAttempt);
					// Try the empirically found "good" joint configurations first, so that pouring is more likely to work
					switch(CurrentAttempt){
						case 1: 
							move_bottle = get_move_bottle_plan_cartesian(bottle.id, glass_pose, .3); break;
						case 2:
							move_bottle = get_move_bottle_plan_fixed(bottle.id, "pour_above_glass_1"); break;
						case 3:
							move_bottle = get_move_bottle_plan_fixed(bottle.id, "pour_above_glass_2"); break;
						default:
							move_bottle = get_move_bottle_plan(bottle.id, glass_pose, .3);
					}
					// if path was found and if executing it was a SUCCESS, move on to next state
					if ( !move_bottle.trajectory_.joint_trajectory.points.empty() ) {
						if( execute_plan(move_bottle) ) {
							if (CurrentAttempt<3) {
								ROS_INFO_STREAM("moveBottleToGlass succeeded with attempt " << CurrentAttempt << " (PREDEFINED JOINT CONFIGURATION!!)");
							} else ROS_INFO_STREAM("moveBottleToGlass succeeded with attempt " << CurrentAttempt);
							reverse_trajectory(move_bottle.trajectory_, move_bottle_back.trajectory_); // for later use in moving the bottle back
							ros::Duration(1.0).sleep();
							state = pourIntoGlass;
							CurrentAttempt = 1;
							break;
						} else {
							ROS_INFO_STREAM("moveBottleToGlass (executing path!) FAILED (in attempt " << CurrentAttempt << ")");
						}
					}
					// when plan was found & executed, this isn't reached, else check number of attempts and either retry state or abort and skip states
					if (CurrentAttempt++ == NUM_RETRIES_AFTER_JAM) {
						ROS_INFO_STREAM("XXX STATE 2: moveBottleToGlass FAILED after all " << CurrentAttempt << " attempts. Moving on to placeBottleDown.");
						state = placeBottleDown;
						failedAtState = moveBottleToGlass;
						CurrentAttempt = 1;
					}
					break;
					*/

				case moveBottleToGlass:
					ROS_INFO_STREAM("--- STATE 2: moveBottleToGlass; (planningTime: " << planningTime << ") ; Attempt " << CurrentAttempt);
					move_bottle = get_move_bottle_plan(bottle.id, glass_pose, glass, .3);
					if(!move_bottle.trajectory_.joint_trajectory.points.empty()) {
						if(execute_plan(move_bottle)) {
							ROS_INFO_STREAM("moveBottleToGlass succeeded with attempt " << CurrentAttempt);
							reverse_trajectory(move_bottle.trajectory_, move_bottle_back.trajectory_); // for later use in moving the bottle back
							ros::Duration(0.5).sleep();
							state = pourIntoGlass;
							CurrentAttempt = 1;
							break;
						} else ROS_INFO_STREAM("moveBottleToGlass (executing path!) FAILED (in attempt " << CurrentAttempt << ")");
					}
					// when plan was found & executed, this isn't reached, else check number of attempts and either retry state or abort and skip states
					if (CurrentAttempt++ == NUM_RETRIES_AFTER_JAM) {
						ROS_INFO_STREAM("XXX STATE 2: moveBottleToGlass FAILED after all " << CurrentAttempt << " attempts. Moving on to placeBottleDown.");
						state = placeBottleDown;
						failedAtState = moveBottleToGlass;
						CurrentAttempt = 1;
					}
					break;

				case pourIntoGlass:
					state = moveBottleBack;
					if ( !pour_bottle_in_glass(bottle, glass, portion_size)){
						failedAtState = pourIntoGlass;
					} else ROS_INFO_STREAM("pourIntoGlass succeeded");
					ros::Duration(0.5).sleep();
					break;
	 

				case moveBottleBack:
					ROS_INFO_STREAM("--- STATE 4: moveBottleBack; Attempt " << CurrentAttempt);
					// if SUCCESS, move on to next state, else check number of attempts and either retry state or abort and skip states
					if ( execute_plan(move_bottle_back) ) {
						ROS_INFO_STREAM("moveBottleBack succeeded with attempt " << CurrentAttempt);
						state = placeBottleDown;
						CurrentAttempt = 1;
						ros::Duration(0.5).sleep();
					} else if ( CurrentAttempt++ == NUM_RETRIES_AFTER_JAM ){
						ROS_INFO_STREAM("XXX STATE 4: moveBottleBack FAILED after all " << CurrentAttempt << " attempts. Moving on to ABORT FOR GOOD.");
						state = Exit; // just stay still and abort if moving back didn't work because the hand is still holding the bottle..
						failedAtState = moveBottleBack;
					}
					break;


				case placeBottleDown:
					ROS_INFO_STREAM("--- STATE 5: placeBottleDown; Attempt " << CurrentAttempt);
					if ( placeBottle(bottle.id) ) {
						ROS_INFO_STREAM("placeBottleDown succeeded with attempt " << CurrentAttempt);
						state = moveArmToDefault;
						CurrentAttempt = 1;
						ros::Duration(0.5).sleep();
					} else if ( CurrentAttempt++ == NUM_RETRIES_AFTER_JAM ) {
						ROS_INFO_STREAM("XXX STATE 5: placeBottleDown FAILED after all " << CurrentAttempt << " attempts. Moving on to ABORT FOR GOOD.");
						state = Exit;
						failedAtState = placeBottleDown;
					}
					break;


				case moveArmToDefault:
					CurrentAttempt = 1;
					ROS_INFO_STREAM("--- STATE 6: moveArmToDefault; Attempt " << CurrentAttempt);
					if ( move_back() ) {
						ROS_INFO_STREAM("moveArmToDefault succeeded with attempt " << CurrentAttempt);
						ros::Duration(1.0).sleep();
						state = run_cleanup;
					} else if ( CurrentAttempt++ == NUM_RETRIES_AFTER_JAM ) {
						ROS_INFO_STREAM("XXX STATE 6: moveArmToDefault FAILED after all " << CurrentAttempt << " attempts. Moving on to cleanup.");
						state = run_cleanup;
						failedAtState = moveArmToDefault;
					}
					break;


				case run_cleanup:
					ROS_INFO_STREAM("--- STATE 7: run_cleanup: despawn objects and move arm to home (if not already happened)");
					cleanup();
					// result.success = failedAtState == OK;
					state = Exit;
					break;
			}
		} while(state != Exit);

		return failedAtState == OK;
	}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "pour_bottle_server");
	listener = new (tf::TransformListener);
	GrabPourPlace gpp;
	ros::AsyncSpinner spinner(1);
	spinner.start();
	ros::spin();
	return 0;
}
