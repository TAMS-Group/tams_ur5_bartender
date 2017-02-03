#define USE_MATH_DEFINES
#include <project16_manipulation/PourBottleAction.h>

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <cmath>
#include <sstream>

#include <actionlib/server/simple_action_server.h>
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/ApplyPlanningScene.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <tf/transform_listener.h>
#include <tf2/exceptions.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <pr2016_msgs/BarCollisionObjectArray.h>


const std::string ARM_ID = "arm";
const std::string GRIPPER_ID = "gripper";
bool USE_BOTTLE_PUBLISHER = true;
bool OBJECTS_RECOGNIZED = true;
const pr2016_msgs::BarCollisionObjectArray* collision_objects_ = NULL;

std::map<std::string, moveit_msgs::CollisionObject> bottles_;

const int NUM_RETRIES_AFTER_JAM = 5;
const tf::TransformListener* listener = NULL;


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
		arm.setPlanningTime(20.0);
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

		//setShoulderPanConstraint();

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

	void setShoulderPanConstraint() {
		moveit_msgs::Constraints constraints = arm.getPathConstraints();
		moveit_msgs::JointConstraint jc;
		jc.joint_name = "ur5_shoulder_pan_joint";
		jc.position = 0.0;
		jc.tolerance_above = M_PI / 2;
		jc.tolerance_below = M_PI / 2;
		constraints.joint_constraints.push_back(jc);
		arm.setPathConstraints(constraints);
	}

	//	bool move_bottle(geometry_msgs::Pose pose, float zOffset) {
	//		//Create orientation constraint - bottle up
	//		//Create orientation constraint - bottle up
	//		moveit_msgs::Constraints constraints;
	//		moveit_msgs::OrientationConstraint ocm;
	//		ocm.link_name = "s_model_tool0";
	//		ocm.header.frame_id = "table_top";
	//		ocm.orientation = tf::createQuaternionMsgFromRollPitchYaw(M_PI, 0, 0);
	//		ocm.absolute_x_axis_tolerance = M_PI / 9;
	//		ocm.absolute_y_axis_tolerance = M_PI / 9;
	//		ocm.absolute_z_axis_tolerance = 2*M_PI;
	//		ocm.weight = 1.0;
	//		constraints.orientation_constraints.push_back(ocm);
	//
	//		//Z offset for pose
	//		pose.position.z += zOffset;
	//
	//		ROS_INFO_STREAM("Moving Bottle to pose: " << pose);
	//
	//		//set target and constraints
	//		arm.setPoseReferenceFrame("table_top");
	//		arm.setPoseTarget(pose);
	//		arm.setPathConstraints(constraints);
	//
	//		//arm.setPlannerId("PRMstarkConfigDefault");
	//		arm.setPlanningTime(90.0);
	//
	//		//move arm
	//		moveit::planning_interface::MoveItErrorCode errorCode = arm.move();
	//
	//		//remove constraints after movement
	//		arm.clearPathConstraints();
	//		return bool(errorCode); 
	//	}

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

	moveit::planning_interface::MoveGroup::Plan get_move_bottle_plan_fixed(std::string bottle_id, std::string target) {

		//bottle up constraint
		set_orientation_constraint();

		//set target and constraints
		arm.setPoseReferenceFrame("table_top");
		arm.setNamedTarget(target);

		//arm.setPlannerId("PRMstarkConfigDefault");
		arm.setPlanningTime(40.0);

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
		arm.setPlanningTime(40);

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

	bool pour_bottle_in_glass(moveit_msgs::CollisionObject bottle, moveit_msgs::CollisionObject glass) {
		ROS_INFO("Pour Bottle");

		moveit_msgs::RobotTrajectory trajectory; //Trajectory

		//Trying around Y-Axis (backwards)
		double success_percentage = arm.computeCartesianPath(compute_pouring_waypoints("Y", false, bottle, glass), 0.03, 3, trajectory);
		ROS_INFO("Pouring trajectory (%.2f%% achieved)", success_percentage * 100.0);

		//Try alternative, if trajectory does not reach 100%
		if(success_percentage < 1.0) {
			ROS_INFO("Trying next direction!");
			//Trying around X-Axis to the right
			moveit_msgs::RobotTrajectory temp_traj; //Alternative Trajectory
			double next_percentage = arm.computeCartesianPath(compute_pouring_waypoints("X", true, bottle, glass), 0.03, 3, temp_traj);
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
				next_percentage = arm.computeCartesianPath(compute_pouring_waypoints("X", false, bottle, glass), 0.03, 3, temp_traj);
				ROS_INFO("Pouring trajectory (%.2f%% achieved)", next_percentage * 100.0); 
				//If better result, take this one
				if(next_percentage > success_percentage) {
					success_percentage = next_percentage;
					trajectory = temp_traj;
				}
			}
		}
		bool success = success_percentage > 0.95;

		if(success) {
			// Plan pouring trajectory (forward)
			moveit::planning_interface::MoveGroup::Plan pour_forward;
			pour_forward.trajectory_ = trajectory;

			//reverse pouring trajectory
			moveit::planning_interface::MoveGroup::Plan pour_backward;
			reverse_trajectory(trajectory, pour_backward.trajectory_);
			ros::Duration(5.0).sleep();

			if(!arm.execute(pour_forward)) {
				//TODO: handle failure
				return false;
			}

			//maybe parameterize pouring time and/or speed
			sleep(2.0);

			if(!arm.execute(pour_backward)) {
				//TODO: handle failure
				return false;
			}
		}			
		return success;
	}

	std::vector<geometry_msgs::Pose> compute_pouring_waypoints(std::string axis, bool inverse_direction, moveit_msgs::CollisionObject bottle, moveit_msgs::CollisionObject glass) {
		arm.setPoseReferenceFrame("world");
		geometry_msgs::Pose start_pose = arm.getCurrentPose().pose;
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
		float glass_offset = 0.04; //vertical distance of bottle tip from glass whenn pouring
		float zDistance = 0.3 + radius - glass_height / 2 - glass_offset;
		float startX = start_pose.position.x;
		float startY = start_pose.position.y;
		float startZ = start_pose.position.z; 
		for(int i = 0; i <= steps; i++) {
			float angle = pScale * M_PI * i / steps;
			float tilt_factor = pow((float) i / steps, 2);
			float translation = dFactor * (sin(angle) * radius +  tilt_factor * glass_radius);
			ROS_INFO_STREAM("computing waypoint for angle " << angle);
			if(axis == "Y") {
				target_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(M_PI, -dFactor * angle, 0);
				target_pose.position.x = startX + translation;

			} else { //X Axis
				target_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(M_PI + dFactor * angle, 0, 0);
				target_pose.position.y = startY + translation;
			}
			//	target_pose.position.z = startZ + (cos(M_PI - angle) + 1 - 2 * 1.1 * tilt_factor) * radius;
			target_pose.position.z = startZ + (cos(M_PI - angle) + 1) * radius - tilt_factor * zDistance;
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
		gripper.setNamedTarget("basic_open");
		gripper.move();
		ros::Duration(1.0).sleep();
		arm.setNamedTarget("pour_default");
		arm.move();
	}

	bool placeBottleDown(std::string bottle_id) {
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

		ros::Duration(1.0).sleep();

		//open gripper
		gripper.setNamedTarget("basic_open");
		ROS_INFO("Setting gripper to target: basic_open");
		if(!gripper.move()) {
			return false;
		}

		ros::Duration(1.0).sleep();

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

		ros::Duration(1.0).sleep();

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
		//Define which steps should be run (should be scoped enum)
		bool run_grab_bottle = true; 
		bool run_move_to_glass = true; 
		bool run_pour_bottle = false; 
		bool run_move_back = false; 
		bool run_place_bottle = false;
		bool run_back_to_default = false;

		cleanup();

		std::string bottle_id = goal->bottle_id;
		moveit_msgs::CollisionObject bottle;

		project16_manipulation::PourBottleFeedback feedback;
		if(USE_BOTTLE_PUBLISHER) {
			recognizeBottles();
			ros::Duration(2.0).sleep();
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

		// State Machine
		int CurrentAttempt;
		enum state { run_grab_bottle2, run_move_to_glass2, run_pour_bottle2, run_move_back2, run_place_bottle2, run_back_to_default2, run_cleanup, Exit, OK };
		state currentState = run_grab_bottle2;
		state failedAtState = OK;

		moveit::planning_interface::MoveGroup::Plan move_bottle_back;
		moveit::planning_interface::MoveGroup::Plan move_bottle;

		project16_manipulation::PourBottleResult result;
		bool failed = false;
		do {
			switch(currentState) {

				case run_grab_bottle2:
					CurrentAttempt = 1;
					currentState = run_move_to_glass2;
					ROS_INFO_STREAM("STATE 1: run_grab_bottle2; Attempt " << CurrentAttempt);
					while (!grab_bottle(bottle.id)) 
					{
						if (CurrentAttempt++ == NUM_RETRIES_AFTER_JAM) 
						{
							ROS_INFO_STREAM("run_grab_bottle2 bottle still failed after all " << CurrentAttempt << " attempts..");
							currentState = run_cleanup;
							failedAtState = run_grab_bottle2;
							break;
						}
						ROS_INFO_STREAM("STATE 1: run_grab_bottle2; Attempt " << CurrentAttempt);
					}
					if (failedAtState==OK)
					{ 
						ROS_INFO_STREAM("run_grab_bottle2 succeeded with attempt " << CurrentAttempt);
					}
					ros::Duration(1.0).sleep();
					break;


				case run_move_to_glass2:
					CurrentAttempt = 1;
					currentState = run_pour_bottle2;
					failed = false;
					ROS_INFO_STREAM("STATE 2: run_move_to_glass2; Attempt " << CurrentAttempt);
					//Try move to glass for x attempts
					do {
						move_bottle = get_move_bottle_plan(bottle.id, glass_pose, .3);
						if (CurrentAttempt++ == NUM_RETRIES_AFTER_JAM)
						{	
							failed = true;
							break;
						}
						ROS_INFO_STREAM("STATE 2: run_move_to_glass2; Attempt " << CurrentAttempt);
					} while (move_bottle.trajectory_.joint_trajectory.points.empty());

					//If that failed, try to go to fixed robot state 'pour_above_glass'
					if (failed) {
						ROS_INFO("Planning failed, planning with fixed robot state!");
						move_bottle = get_move_bottle_plan_fixed(bottle.id, "pour_above_glass_1");
						failed = move_bottle.trajectory_.joint_trajectory.points.empty(); 
					}

					//If that failed, try to go to fixed robot state 'pour_above_glass'
					if (failed) {
						ROS_INFO("Planning failed, planning with fixed robot state!");
						move_bottle = get_move_bottle_plan_fixed(bottle.id, "pour_above_glass_2");
						failed = move_bottle.trajectory_.joint_trajectory.points.empty(); 
					}

					//If we have a valid plan, execute and create reverse trajectory
					if (!failed) {
						if(execute_plan(move_bottle)) {
							if(CurrentAttempt == NUM_RETRIES_AFTER_JAM) {
								ROS_INFO("Failed executing motion, using fixed robot state 'pour_above_glass'");
							} else {
								ROS_INFO_STREAM("run_move_to_glass2 succeeded with attempt " << CurrentAttempt);
							}
							reverse_trajectory(move_bottle.trajectory_, move_bottle_back.trajectory_);
							ros::Duration(1.0).sleep();
							break;
						}
					}

					//Here we failed!
					currentState = run_place_bottle2;
					failedAtState = run_move_to_glass2;
					break;


				case run_pour_bottle2:
					currentState = run_move_back2;
					if (!pour_bottle_in_glass(bottle, glass)) {
						failedAtState = run_pour_bottle2;
					} else ROS_INFO_STREAM("run_pour_bottle2 succeeded");
					ros::Duration(1.0).sleep();
					break;

					/*
					   if(false){
					   CurrentAttempt = 1;
					   ROS_INFO_STREAM("STATE 3: run_pour_bottle2; Attempt " << CurrentAttempt);
					   while(!pour_bottle_in_glass(bottle, glass)) 
					   {
					   if (!CurrentAttempt++<NUM_RETRIES_AFTER_JAM)
					   {
					   ROS_INFO_STREAM("run_pour_bottle2 still failed after all " << CurrentAttempt << " attempts..");
					   failedAtState = run_pour_bottle2;
					   currentState = run_move_back2; // TODO what do here? -> bottle still in hand, possibly stuck somewhere in the middle of the pouring motion..
					   break;
					   }
					   ROS_INFO_STREAM("STATE 3: run_pour_bottle2; Attempt " << CurrentAttempt);
					   }
					   if (failedAtState==OK)
					   { 
					   ROS_INFO_STREAM("run_pour_bottle2 succeeded with attempt " << CurrentAttempt);
					   currentState = run_move_back2;
					   ros::Duration(1.0).sleep();
					   }
					   break;
					   }
					 */

				case run_move_back2:
					CurrentAttempt = 1;
					ROS_INFO_STREAM("STATE 4: run_move_back2; Attempt " << CurrentAttempt);
					while(!execute_plan(move_bottle_back))
					{
						if (CurrentAttempt++ == NUM_RETRIES_AFTER_JAM)
						{
							currentState = Exit;
							failedAtState = run_move_back2;
							break;
						}
						ROS_INFO_STREAM("STATE 4: run_move_back2; Attempt " << CurrentAttempt);
					}
					if (failedAtState!=run_move_back2)
					{ 
						ROS_INFO_STREAM("run_move_back2 succeeded with attempt " << CurrentAttempt);
						currentState = run_place_bottle2;
					}
					ros::Duration(1.0).sleep();
					break;


				case run_place_bottle2:
					currentState = run_back_to_default2;
					CurrentAttempt = 1;
					ROS_INFO_STREAM("STATE 5: run_place_bottle2; Attempt " << CurrentAttempt);
					while(!placeBottleDown(bottle.id)) 
					{
						if (CurrentAttempt++ == NUM_RETRIES_AFTER_JAM)
						{
							failedAtState = run_place_bottle2;
							break;
						}
						ROS_INFO_STREAM("STATE 5: run_place_bottle2; Attempt " << CurrentAttempt);
					}
					if (failedAtState == run_place_bottle2)
					{ 
						ROS_INFO_STREAM("run_place_bottle2 succeeded with attempt " << CurrentAttempt);
						currentState = Exit; //TODO: handle failure
					} 
					ros::Duration(1.0).sleep();
					break;

				case run_back_to_default2:
					CurrentAttempt = 1;
					ROS_INFO_STREAM("STATE 6: run_back_to_default2; Attempt " << CurrentAttempt);
					while(!move_back()) 
					{
						if (CurrentAttempt++ == NUM_RETRIES_AFTER_JAM)
						{
							ROS_INFO_STREAM("run_back_to_default2 failed..");
							failedAtState = run_back_to_default2;
							break;
						}
						ROS_INFO_STREAM("STATE 6: run_back_to_default2; Attempt " << CurrentAttempt);
					}
					currentState = run_cleanup;
					if (failedAtState != run_back_to_default2)
					{ 
						ROS_INFO_STREAM("run_back_to_default2 succeeded with attempt " << CurrentAttempt);
					}
					ros::Duration(3.0).sleep();
					break;


				case run_cleanup:
					ROS_INFO_STREAM("STATE 7: run_cleanup");
					//despawn objects and move arm to home (if not already happened)
					cleanup();
					result.success = failedAtState == OK;
					currentState = Exit;
					break;
			}
		} while(currentState != Exit);

		if(result.success) {
			as_->setSucceeded(result,"Grasping, constrained motion and pouring suceeded");
		} else {
			as_->setAborted();
		}
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
