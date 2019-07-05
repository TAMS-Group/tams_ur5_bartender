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
#include <iostream>
#include <stdio.h>

#include <moveit/move_group_interface/move_group_interface.h>

#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/ApplyPlanningScene.h>
#include <moveit_msgs/Grasp.h>

bool GRIPPER_TEST = false;

class GrabBottleTest{

protected:
    ros::NodeHandle node_handle;
    ros::ServiceClient planning_scene_diff_client;
    ros::ServiceClient grasp_planning_service;

public:
    GrabBottleTest()
    {
        planning_scene_diff_client = node_handle.serviceClient<moveit_msgs::ApplyPlanningScene>("apply_planning_scene");
        planning_scene_diff_client.waitForExistence();
    }

    ~GrabBottleTest(){
    }

    bool executePick(moveit::planning_interface::MoveGroupInterface& arm){
        arm.setSupportSurfaceName("table");
        return bool(arm.planGraspsAndPick("object"));
    }
/*
    bool executePick(moveit_msgs::CollisionObject &object){
        arm.setPlanningTime(20.0);
        arm.setSupportSurfaceName("table");
        return arm.planGraspsAndPick(object);
    }

    void executePlace(){

        moveit::planning_interface::MoveGroup gripper("gripper");

        std::vector<moveit_msgs::PlaceLocation> location;

        moveit_msgs::PlaceLocation place_location;
        place_location.id = "place_location";

        jointValuesToJointTrajectory(gripper.getNamedTargetValues("open"), place_location.post_place_posture);

        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = "table_top";
        pose.pose.orientation.w = 1;
        pose.pose.position.x = -0.03;
        pose.pose.position.y = 0.2;
        pose.pose.position.z = 0.1;
        place_location.place_pose = pose;

        place_location.pre_place_approach.min_distance = 0.02;
        place_location.pre_place_approach.desired_distance = 0.1;
        place_location.pre_place_approach.direction.header.frame_id = "tool0";
        place_location.pre_place_approach.direction.vector.x = 1.0;

        place_location.post_place_retreat.min_distance = 0.02;
        place_location.post_place_retreat.desired_distance = 0.1;
        place_location.post_place_retreat.direction.header.frame_id = "table_top";
        place_location.post_place_retreat.direction.vector.z = 1.0;

        location.push_back(place_location);

        arm.setSupportSurfaceName("table");
        arm.place("object", location);
    }
*/

    void despawnObject(){
        moveit_msgs::ApplyPlanningScene srv;
        moveit_msgs::PlanningScene planning_scene;
        planning_scene.is_diff = true;
        planning_scene.robot_state.is_diff = true;

        moveit_msgs::AttachedCollisionObject aobj;
        aobj.object.id = "object";
        aobj.object.operation = aobj.object.REMOVE;
        planning_scene.robot_state.attached_collision_objects.push_back(aobj);

        moveit_msgs::CollisionObject object;
        object.id = "object";
        object.operation = object.REMOVE;
        planning_scene.world.collision_objects.push_back(object);
        srv.request.scene = planning_scene;
        planning_scene_diff_client.call(srv);
    }
          
    moveit_msgs::CollisionObject spawnObject(){
        moveit_msgs::ApplyPlanningScene srv;
        moveit_msgs::PlanningScene planning_scene;
        planning_scene.is_diff = true;
        planning_scene.robot_state.is_diff = true;

        moveit_msgs::CollisionObject object;

        object.header.frame_id = "table_top";
        object.id = "object";

        shape_msgs::SolidPrimitive primitive;
        primitive.type = primitive.CYLINDER;
        primitive.dimensions.push_back(0.3);
        primitive.dimensions.push_back(0.04);

        geometry_msgs::Pose pose;
        pose.orientation.w = 1;
        pose.position.x = -0.1;
        pose.position.y = -0.1;
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

    void jointValuesToJointTrajectory(std::map<std::string, double> target_values, trajectory_msgs::JointTrajectory &grasp_pose){
       grasp_pose.joint_names.reserve(target_values.size());
       grasp_pose.points.resize(1);
       grasp_pose.points[0].positions.reserve(target_values.size());

       for(std::map<std::string, double>::iterator it = target_values.begin(); it != target_values.end(); ++it){
           grasp_pose.joint_names.push_back(it->first);
           grasp_pose.points[0].positions.push_back(it->second);
       }
    }
/*
    void testPose(){

        geometry_msgs::PoseStamped pose;

        pose.header.frame_id = "table_top";
        pose.pose.orientation.x = 0.5;
        pose.pose.orientation.y = 0.5;
        pose.pose.orientation.z = -0.5;
        pose.pose.orientation.w = 0.5;
        pose.pose.position.x = 0.05;
        pose.pose.position.y = 0.2;
        pose.pose.position.z = 0.344;

        arm.setPoseTarget(pose, "tool0");
        arm.move();
    }
*/
};

int main(int argc, char** argv){
    ros::init(argc, argv, "PaPTest");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit::planning_interface::MoveGroupInterface gripper("gripper");
    moveit::planning_interface::MoveGroupInterface arm("arm");

    //arm.setPlannerId(arm.getDefaultPlannerId("arm"));
    arm.setPlannerId("RRTConnectkConfigDefault");
    arm.setPlanningTime(20.0);

    if(GRIPPER_TEST) {
        gripper.setNamedTarget("basic_closed");
        gripper.move();
        ros::Duration(5.0).sleep();
        gripper.setNamedTarget("basic_open");
        gripper.move();
        return 0;
    }

    ROS_INFO("Starting Test");
    GrabBottleTest testClass;

    testClass.despawnObject();

    gripper.setNamedTarget("basic_open");
    gripper.move();
    arm.setNamedTarget("folded");
    arm.move();

    ros::Duration(5.0).sleep();
    ROS_INFO("Spawn Object");
    testClass.spawnObject();

    ROS_INFO("Grab Bottle");
    bool succeeded = testClass.executePick(arm);
    if (succeeded){
      ROS_INFO("Execute pick succeded");
      return 0;
    }
    else {
      ROS_ERROR("Execute pick failed");
      return 1;
    }
}
