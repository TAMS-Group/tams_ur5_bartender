#include <iostream>
#include <stdio.h>

#include <moveit/move_group_interface/move_group.h>

#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/ApplyPlanningScene.h>
#include <moveit_msgs/Grasp.h>

#include <manipulation_msgs/GraspPlanning.h>

class PourBottleTest{
	struct object_description {
	    float dim[3];
	    float pos[3];
	};


	protected:
		ros::NodeHandle node_handle;
		ros::ServiceClient planning_scene_diff_client;
		ros::ServiceClient grasp_planning_service;
		std::map<std::string, object_description> object_map;

	public:
		PourBottleTest()
		{
			object_description glas = {{0, 0.03, 0.12}, {0.1, 0.1, 0}};
			object_map["glas"] = glas;
			object_description bottle = {{0, 0.04, 0.3}, {-0.1, -0.1, 0}};
			object_map["bottle"] = bottle;
			
			planning_scene_diff_client = node_handle.serviceClient<moveit_msgs::ApplyPlanningScene>("apply_planning_scene");
			planning_scene_diff_client.waitForExistence();	
		}
		~PourBottleTest(){
		}


    moveit_msgs::CollisionObject spawnObject(std::string objectID){
        moveit_msgs::ApplyPlanningScene srv;
        moveit_msgs::PlanningScene planning_scene;
        planning_scene.is_diff = true;
        planning_scene.robot_state.is_diff = true;

        moveit_msgs::CollisionObject object;

        object.header.frame_id = "table_top";
        object.id = objectID;
        
        object_description objProps = object_map[objectID];
	
        shape_msgs::SolidPrimitive primitive;
        primitive.type = primitive.CYLINDER;
        primitive.dimensions.push_back(objProps.dim[2]);
        primitive.dimensions.push_back(objProps.dim[1]);

        geometry_msgs::Pose pose;
        pose.orientation.w = 1;
        pose.position.x = objProps.pos[0];
        pose.position.y = objProps.pos[1];
        pose.position.z = primitive.dimensions[0]/2 + 0.0;

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

	PourBottleTest testClass;

	testClass.spawnObject("bottle");
	testClass.spawnObject("glas");

}

