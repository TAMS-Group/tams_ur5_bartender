#include <iostream>
#include <stdio.h>

#include <moveit/move_group_interface/move_group.h>

#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/ApplyPlanningScene.h>
#include <moveit_msgs/Grasp.h>
#include <moveit_msgs/GetPlanningScene.h>

#include <tf/transform_listener.h>

class GrabBottleDemo{

protected:
    ros::NodeHandle node_handle;
    ros::ServiceClient planning_scene_diff_client;
    ros::ServiceClient get_planning_scene_client;

    moveit::planning_interface::MoveGroup arm;
    moveit::planning_interface::MoveGroup gripper;

    tf::TransformListener tf_listener;
    tf::StampedTransform transform;

    double objectHeight;

    geometry_msgs::PoseStamped place_pose;

public:
    GrabBottleDemo() : 
        arm("arm"), 
        gripper("gripper")
    {
        planning_scene_diff_client = node_handle.serviceClient<moveit_msgs::ApplyPlanningScene>("apply_planning_scene");
        planning_scene_diff_client.waitForExistence();
        get_planning_scene_client = node_handle.serviceClient<moveit_msgs::GetPlanningScene>("get_planning_scene");
        get_planning_scene_client.waitForExistence();

    }

    ~GrabBottleDemo(){
    }

    bool executePick(){
        arm.setSupportSurfaceName("table");
        return arm.planGraspsAndPick("can");
    }

    
    //PLACE DOES NOT WORK YET!

/*    void executePlace(){

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

    bool spawnObject(){
        try{
            ros::Time time = ros::Time::now();
            tf_listener.waitForTransform("table_top", "object", time, ros::Duration(2.0));
            tf_listener.lookupTransform("table_top", "object", time, transform);
        }catch(tf::TransformException ex){
            return false;
        }

        objectHeight = transform.getOrigin().getZ();
        moveit_msgs::ApplyPlanningScene srv;
        moveit_msgs::PlanningScene planning_scene;
        planning_scene.is_diff = true;

        moveit_msgs::CollisionObject object;
        object.header.frame_id = "object";
        object.id = "can";

        shape_msgs::SolidPrimitive primitive;
        primitive.type = primitive.CYLINDER;
        primitive.dimensions.push_back(objectHeight); //height
        primitive.dimensions.push_back(0.04); //radius
        object.primitives.push_back(primitive);

        geometry_msgs::Pose pose;
        pose.orientation.w = 1.0;
        pose.position.z = -objectHeight/2;
        object.primitive_poses.push_back(pose);

        place_pose.header.frame_id = "table_top";
        place_pose.pose.orientation.x = 0.5;
        place_pose.pose.orientation.y = 0.5;
        place_pose.pose.orientation.z = -0.5;
        place_pose.pose.orientation.w = 0.5;
        place_pose.pose.position.x = transform.getOrigin().getX();
        place_pose.pose.position.y = transform.getOrigin().getY();
        place_pose.pose.position.z = transform.getOrigin().getZ() + 0.005;

        object.operation = object.ADD;
        planning_scene.world.collision_objects.push_back(object);

        srv.request.scene = planning_scene;
        planning_scene_diff_client.call(srv);

        return true;
    }

    bool moveToStart(){
        moveit_msgs::GetPlanningScene::Request request;
        moveit_msgs::GetPlanningScene::Response response;

        request.components.components = request.components.ROBOT_STATE;

        get_planning_scene_client.call(request, response);

        if(!similarJointStates(response.scene.robot_state.joint_state, arm.getNamedTargetValues("folded"))){
            ROS_INFO("bewege zu startzustand");
            arm.setNamedTarget("folded");
            if(arm.move()){
                ros::Duration(5.0).sleep();
                return true;
            }
            else
                return false;
        }
        return true;
    }

    bool placeStub(){
        arm.setPoseTarget(place_pose,"s_model_tool0");
        return arm.move();
        return false;
    }

    bool releaseObject(){

        gripper.setNamedTarget("open");
        return gripper.move();
    }

    void resetPlacePose(){
        place_pose = geometry_msgs::PoseStamped();
    }

    void detachObject(){
        moveit_msgs::PlanningScene planning_scene;

        moveit_msgs::AttachedCollisionObject attached_collision_object;
        attached_collision_object.object.id = "can";
        attached_collision_object.object.operation = attached_collision_object.object.REMOVE;

        planning_scene.robot_state.attached_collision_objects.push_back(attached_collision_object);
        planning_scene.is_diff = true;
        planning_scene.robot_state.is_diff = true;
        moveit_msgs::ApplyPlanningScene srv;
        srv.request.scene = planning_scene;
        planning_scene_diff_client.call(srv);
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

    bool similarJointStates(sensor_msgs::JointState joint_states, std::map<std::string, double> joint_values){
        for(int i=0; i < joint_states.name.size(); i++){
            if(joint_values.find(joint_states.name[i]) != joint_values.end()){
                double rad_diff = fabs(joint_states.position[i] - joint_values.at(joint_states.name[i]));
                if(rad_diff > 0.01){
                    return false;
                }
            }
        }
        return true;
    }


};

int main(int argc, char** argv){
    ros::init(argc, argv, "PaPDemo");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    GrabBottleDemo demo;

    int count = 0;
    while(true){
        count++;
        ROS_INFO("Arm moves to start position");
        if(!demo.moveToStart()){
            ROS_ERROR("Move to start pose failed");
            continue;
        }
        demo.resetPlacePose();
        ROS_INFO_STREAM("Pick number " << count);
        if(!demo.spawnObject()){
            ROS_WARN("No object detected, please put object in the next 10 seconds on the table");
            ros::Duration(5.0).sleep();
            continue;
        }

        if(!demo.executePick()){
            ROS_INFO("Pick failed, retry in 5 seconds");
            ros::Duration(5.0).sleep();
            continue;
        }
        ros::Duration(1.0).sleep();

        if(!demo.placeStub()){
            ROS_ERROR("Place failed");
            break;
        }

        if(!demo.releaseObject()){
            ROS_ERROR("Release object failed");
            break;
        }

        demo.detachObject();

    }
    return 0;
}

