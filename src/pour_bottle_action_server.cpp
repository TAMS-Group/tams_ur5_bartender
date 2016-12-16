#define USE_MATH_DEFINES
#include <project16_manipulation/PourBottleAction.h>
#include <actionlib/server/simple_action_server.h>

#include <iostream>
#include <stdio.h>

#include <moveit/move_group_interface/move_group.h>

#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/ApplyPlanningScene.h>
#include <moveit_msgs/Grasp.h>

#include <manipulation_msgs/GraspPlanning.h>

#include <cmath>
#include <ros/ros.h>

const std::string ARM_ID = "arm";
const std::string GRIPPER_ID = "gripper";

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

	
	public:
	std::map<std::string, ObjectDescription> object_map;

	GrabPourPlace() : arm(ARM_ID), gripper(GRIPPER_ID)
	{
		ObjectDescription glass = {{0, 0.03, 0.08}, {0.20, 0.10, 0.0}};
		object_map["glass"] = glass;

		ObjectDescription bottle = {{0, 0.04, 0.3}, {-0.1, -0.1, 0}};
		object_map["bottle"] = bottle;

		planning_scene_diff_client = node_handle.serviceClient<moveit_msgs::ApplyPlanningScene>("apply_planning_scene");
		planning_scene_diff_client.waitForExistence();

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

	bool move_bottle(geometry_msgs::Pose pose) {
		//Create orientation constraint - bottle up
		moveit_msgs::Constraints constraints;
		moveit_msgs::OrientationConstraint ocm;
		ocm.link_name = "s_model_tool0";
		ocm.header.frame_id = "table_top";
		ocm.orientation = tf::createQuaternionMsgFromRollPitchYaw(M_PI, 0, 0);
		ocm.absolute_x_axis_tolerance = 0.1;
		ocm.absolute_y_axis_tolerance = 0.1;
		ocm.absolute_z_axis_tolerance = 2*M_PI;
		ocm.weight = 1.0;
		constraints.orientation_constraints.push_back(ocm);

		//Z offset for pose
		pose.position.z += 0.3;
		pose.orientation = tf:rcreateQuaternionMsgFromRollPitchYaw(M_PI, 0, 0);
		rCode

		ROS_INFO_STREAM("Moving Bottle to pose: " << pose);

		//set target and constraints
		arm.setPoseReferenceFrame("table_top");
		arm.setPoseTarget(pose);
		arm.setPathConstraints(constraints);

		//move arm
		moveit::planning_interface::MoveItErrorCode errorCode = arm.move();

		//remove constraints after movement
		arm.clearPathConstraints();
		return bool(errorCode); 
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

		moveit::planning_interface::MoveGroup::Plan pour_forward;
		moveit::planning_interface::MoveGroup::Plan pour_backward;

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
		arm.setNamedTarget("folded");
		arm.move();
	}
};

typedef actionlib::SimpleActionServer<project16_manipulation::PourBottleAction> Server;

void execute(const project16_manipulation::PourBottleGoalConstPtr& goal, Server* as)
{
	GrabPourPlace task_planner;
	task_planner.cleanup();
	project16_manipulation::PourBottleFeedback feedback;

	//Spawn bottle and glass
	moveit_msgs::CollisionObject bottle = task_planner.spawnObject("bottle");
	moveit_msgs::CollisionObject glass = task_planner.spawnObject("glass");
	geometry_msgs::Pose bottle_pose = bottle.primitive_poses[0];
	geometry_msgs::Pose glass_pose = glass.primitive_poses[0];
	feedback.task_state = "Objects spawned";
	as->publishFeedback(feedback);

	bool success = true;

	if(success = task_planner.grab_bottle(bottle.id)) {
		feedback.task_state = "Bottle grasped" + bottle.id;
		as->publishFeedback(feedback);
		ros::Duration(1.0).sleep();
		//move bottle to glass
		if( success = task_planner.move_bottle(glass_pose)) {
			feedback.task_state = "Bottle moved to glass";
			as->publishFeedback(feedback);
			ros::Duration(1.0).sleep();
			//perform pouring motion
			if (success = task_planner.pour_bottle(bottle)) {
				feedback.task_state = "Pouring succeeded";
				as->publishFeedback(feedback);
				//move bottle back to bottle position
				if (success = task_planner.move_bottle(bottle_pose)) {
					feedback.task_state = "Bottle moved back";
					as->publishFeedback(feedback);
				}
			}
		}
	}

	ros::Duration(3.0).sleep();
	//despawn objects and move arm to home
	task_planner.cleanup();
	// Do lots of awesome groundbreaking robot stuff here
	project16_manipulation::PourBottleResult result;
	result.success = success;
	as->setSucceeded(result,"Grasping, constrained motion and pouring suceeded");
	
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "pour_bottle_server");
	ros::NodeHandle n;
	Server server(n, "pour_bottle", boost::bind(&execute, _1, &server), false);
	server.start();
	ros::spin();
	return 0;
}
