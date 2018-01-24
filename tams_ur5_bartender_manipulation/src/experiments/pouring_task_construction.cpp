/*
Copyright (c) 2017, Lars Henning Kayser
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

/*
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
*/

#include <ros/ros.h>
#include <tf_conversions/tf_eigen.h>

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/AttachedCollisionObject.h>

// moveit task constructor includes
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/gripper.h>
#include <moveit/task_constructor/stages/move.h>
#include <moveit/task_constructor/stages/generate_grasp_pose.h>
#include <moveit/task_constructor/stages/generate_pose.h>
#include <moveit/task_constructor/stages/cartesian_position_motion.h>
#include <moveit/task_constructor/stages/pouring_motion.h>

void spawnCylinder(moveit::planning_interface::PlanningSceneInterface& psi, const std::string& obj_id, double height, double radius, double x_pos, double y_pos){
	moveit_msgs::CollisionObject o;
	o.id= obj_id;
	o.header.frame_id= "table_top";
	o.primitive_poses.resize(1);
	o.primitive_poses[0].position.x= x_pos;
	o.primitive_poses[0].position.y= y_pos;
	o.primitive_poses[0].position.z= 0.5 * height;
	o.primitive_poses[0].orientation.w= 1.0;
	o.primitives.resize(1);
	o.primitives[0].type= shape_msgs::SolidPrimitive::CYLINDER;
	o.primitives[0].dimensions.resize(2);
	o.primitives[0].dimensions[0]= height;
	o.primitives[0].dimensions[1]= radius;
	psi.applyCollisionObject(o);
}

int constructPouringTask(){
	using namespace moveit::task_constructor;

	moveit::planning_interface::PlanningSceneInterface psi;
	std::string bottle = "bottle";
	std::string glass = "glass";
	spawnCylinder(psi, bottle, 0.25, 0.035, 0.2, -0.2);
	spawnCylinder(psi, glass, 0.12, 0.03, -0.2, 0.3);

	std::vector<std::string> ids = { bottle };

	std::map<std::string, geometry_msgs::Pose> poses = psi.getObjectPoses(ids);
	geometry_msgs::PoseStamped bottle_pose;
	bottle_pose.header.frame_id = "world";
	bottle_pose.pose = poses[bottle];

		/*
	if(ps.knowsFrameTransform(bottle)) {
		Eigen::Affine3d& bottle_pose_affine = ps.getFrameTransform(bottle);
		tf::poseEigenToMsg(bottle_pose_affine, bottle_pose);
	} else {
		ROS_ERROR_STREAM("Bottle frame not found!");
		return 1;
	}
	*/
	
	Task t;

	{
		auto move = std::make_unique<stages::CurrentState>("current state");
		t.add(std::move(move));
	}


	{
		auto move = std::make_unique<stages::Gripper>("open gripper");
		move->setEndEffector("gripper");
		move->setTo("basic_open");
		t.add(std::move(move));
	}

	{
		auto move = std::make_unique<stages::Move>("move to pre-grasp");
		move->setGroup("arm");
		move->setPlannerId("RRTConnectkConfigDefault");
		move->setTimeout(8.0);
		t.add(std::move(move));
	}

	{
		auto move = std::make_unique<stages::CartesianPositionMotion>("proceed to grasp pose");
		move->addSolutionCallback(std::ref(t.introspection()));
		move->setGroup("arm");
		move->setLink("s_model_tool0");
		move->setMinMaxDistance(.03, 0.1);
		move->setCartesianStepSize(0.02);

		geometry_msgs::PointStamped target;
		target.header.frame_id= bottle;
		move->towards(target);
		t.add(std::move(move));
	}

	{
		auto gengrasp = std::make_unique<stages::GenerateGraspPose>("generate grasp pose");
		gengrasp->setEndEffector("gripper");
		//gengrasp->setGroup("arm");
		gengrasp->setGripperGraspPose("basic_open");
		gengrasp->setObject(bottle);
		gengrasp->setGraspOffset(.03);
		gengrasp->setAngleDelta(-.2);
		gengrasp->setMaxIKSolutions(8);
		t.add(std::move(gengrasp));
	}

	{
		auto move = std::make_unique<stages::Gripper>("grasp");
		move->setEndEffector("gripper");
		move->setTo("basic_closed");
		move->graspObject(bottle);
		t.add(std::move(move));
	}

	{
		auto move = std::make_unique<stages::CartesianPositionMotion>("lift object");
		move->setGroup("arm");
		move->setLink("s_model_tool0");
		move->setMinMaxDistance(0.03, 0.05);
		move->setCartesianStepSize(0.01);

		geometry_msgs::Vector3Stamped direction;
		direction.header.frame_id= "table_top";
		direction.vector.z= 1.0;
		move->along(direction);
		t.add(std::move(move));
	}

	{
		auto move = std::make_unique<stages::Move>("move to glass");
		move->setGroup("arm");
		move->setPlannerId("RRTConnectkConfigDefault");
		move->setTimeout(8.0);
		t.add(std::move(move));
	}

	/*
	{
		auto genpose = std::make_unique<stages::GeneratePose>("generate pose above bottle");
		genpose->setGroup("arm");
		genpose->setLink("s_model_tool0");
		geometry_msgs::PoseStamped pose;
		pose.header.frame_id = glass;
		pose.pose.orientation.w = 1;
		pose.pose.position.z = 0.3;
		genpose->fromPose(pose);
		genpose->setMaxIKSolutions(8);
		t.add(std::move(genpose));
	}
	*/

	{
		auto genpose = std::make_unique<stages::GeneratePose>("generate pose above glass");
		genpose->setGroup("arm");
		genpose->setLink("s_model_tool0");

		geometry_msgs::PoseStamped pose_stamped;
		pose_stamped.header.frame_id= glass;
		pose_stamped.pose.orientation.w = 1;
		pose_stamped.pose.position.z = 0.3;
		genpose->fromPose(pose_stamped);

		/*
		moveit_msgs::Constraints constraints;
		moveit_msgs::PositionConstraint pc;
		pc.header.frame_id = glass;
		pc.link_name = "s_model_tool0";
		shape_msgs::SolidPrimitive primitive;
		primitive.type = shape_msgs::SolidPrimitive::SPHERE;
		primitive.dimensions.push_back(0.001);
		pc.constraint_region.primitives.push_back(primitive);
		geometry_msgs::Pose pose;
		pose.orientation.w = 1;
		pose.position.z = 0.3;
		pc.constraint_region.primitive_poses.push_back(pose);
		pc.weight=1.0;
		constraints.position_constraints.push_back(pc);
		moveit_msgs::OrientationConstraint oc;
		oc.link_name = "s_model_tool0";
		oc.header.frame_id = glass;
		oc.orientation.w = 1.0;
		oc.absolute_x_axis_tolerance = 0.001;
		oc.absolute_y_axis_tolerance = 0.001;
		oc.absolute_z_axis_tolerance = 2*M_PI;
		oc.weight = 1.0;
		constraints.orientation_constraints.push_back(oc);
		genpose->fromConstraints(constraints);
		*/
		genpose->setMaxIKSolutions(8);
		t.add(std::move(genpose));
	}


	{
		auto move = std::make_unique<stages::PouringMotion>("pouring motion");
		move->setGroup("arm");
		move->setGlass(glass);
		move->setCartesianStepSize(0.01);
		t.add(std::move(move));
	}

	
	{
		auto move = std::make_unique<stages::Move>("move to bottle position");
		move->setGroup("arm");
		move->setPlannerId("RRTConnectkConfigDefault");
		move->setTimeout(8.0);
		t.add(std::move(move));
	}

	{
		auto move = std::make_unique<stages::CartesianPositionMotion>("lower object");
		move->setGroup("arm");
		move->setLink("s_model_tool0");
		move->setMinMaxDistance(0.03, 0.05);
		move->setCartesianStepSize(0.01);

		geometry_msgs::Vector3Stamped direction;
		direction.header.frame_id= "table_top";
		direction.vector.z= -1.0;
		move->along(direction);
		t.add(std::move(move));
	}

	{
		//TODO: check for correct offset etc (maybe implement grasp pose equivalent)
		auto genpose = std::make_unique<stages::GeneratePose>("generate old bottle pose");
		genpose->setGroup("arm");
		genpose->setLink("s_model_tool0");
		genpose->fromPose(bottle_pose);
		genpose->setTimeout(0.5);
		t.add(std::move(genpose));
	}


	{
		auto move = std::make_unique<stages::Gripper>("open gripper");
		move->setEndEffector("gripper");
		move->setTo("basic_open");
		//TODO: implement detach object
		move->graspObject(bottle);
		t.add(std::move(move));
	}

	{
		auto move = std::make_unique<stages::Move>("move to default configuration");
		move->setGroup("arm");
		move->setPlannerId("RRTConnectkConfigDefault");
		move->setTimeout(8.0);
		t.add(std::move(move));
	}

	{
		//TODO: implement pose generation for named target states
		auto genpose = std::make_unique<stages::GeneratePose>("generate default configuration");
		genpose->setGroup("arm");
		genpose->setLink("s_model_tool0");
		geometry_msgs::PoseStamped pose = bottle_pose;
		pose.pose.position.z += 0.04;
		genpose->fromPose(pose);
		genpose->setTimeout(0.5);
		t.add(std::move(genpose));
	}
	
	t.plan();
	t.publishAllSolutions();

	return 0;
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "pour_bottle_server");
	ros::AsyncSpinner spinner(4);
	spinner.start();
	return constructPouringTask();
}
