#include <iostream>
#include <stdio.h>


#include <ros/ros.h>

#include <moveit/move_group_interface/move_group.h>

#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/ApplyPlanningScene.h>
#include <moveit_msgs/Grasp.h>
#include <moveit_msgs/DisplayTrajectory.h>


#include <manipulation_msgs/GraspPlanning.h>
#include <math.h>

class PourBottleTest{
	struct objectDescription {
	    float dim[3];
	    float pos[3]; 
	};


	protected:
		ros::NodeHandle node_handle;
		ros::ServiceClient planning_scene_diff_client;
		ros::ServiceClient grasp_planning_service;
		std::map<std::string, objectDescription> object_map;

	public:
		PourBottleTest()
		{
			objectDescription glas = {{0, 0.03, 0.12}, {0.1, 0.1, 0.05}};
			object_map["glas"] = glas;
			
			objectDescription bottle = {{0, 0.04, 0.3}, {0, 0, 0}};// {-0.1, -0.1, 0}};
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

	ROS_INFO("Spawning glas and bottle");
	moveit_msgs::CollisionObject glas = testClass.spawnObject("glas", "table_top");

	std::string endeffectorLink = arm.getEndEffectorLink().c_str();
	//testClass.spawnObject("bottle", endeffectorLink);
	//gripper.attachObject("bottle", endeffectorLink);
	sleep(2.0);

	// waypoints for pouring motion
	std::vector<geometry_msgs::Pose> waypoints;
	geometry_msgs::Pose glass_pose;
	glass_pose.orientation.w = 1.0;
	glass_pose.position.x = 0.1;
	glass_pose.position.y = 0.1;
	glass_pose.position.z = 0.2;

	arm.setPoseReferenceFrame("table_top");
	arm.setPoseTarget(glass_pose);
	arm.move();
	sleep(2);

	arm.setPoseReferenceFrame("world");
	geometry_msgs::Pose start_pose = arm.getCurrentPose().pose;
	geometry_msgs::Pose target_pose = start_pose;
	waypoints.push_back(start_pose);
	for(int i = 1; i < 10; i++) {
		float angle = M_PI * i / 10;
		ROS_INFO_STREAM("computing waypoint for angle " << angle);
		target_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(angle, 0, 0);
		target_pose.position.x += 0.02;
		waypoints.push_back(target_pose);
	}

	//for(int i = 0; i < waypoints.size(); i++) {
	//	arm.setPoseTarget(waypoints[i]);
	//	arm.move();
	//	sleep(2.0);
	//}
	moveit_msgs::RobotTrajectory trajectory;
	double fraction = arm.computeCartesianPath(waypoints, 0.01, 1000, trajectory);
	ROS_INFO("Visualizing plan 4 (cartesian path) (%.2f%% acheived)", fraction * 100.0);
	sleep(5.0);

	moveit::planning_interface::MoveGroup::Plan pouring_plan;
	pouring_plan.trajectory_ = trajectory;
	arm.execute(pouring_plan);

// TODO 
// define frame for the bottle opening
// place over glas using position and angle 
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

