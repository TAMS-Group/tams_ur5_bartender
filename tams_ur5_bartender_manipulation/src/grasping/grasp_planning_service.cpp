/*Copyright (c) 2017, Daniel Ahlers, Lars Henning Kayser, Jeremias Hartz, Maham Tanveer, Oke Martensen
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
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.*/
#include <ros/ros.h>
#include <moveit_msgs/GraspPlanning.h>

#include <moveit/move_group_interface/move_group_interface.h>

#include <tf/transform_datatypes.h>

void jointValuesToJointTrajectory(std::map<std::string, double> target_values, ros::Duration duration, 
        trajectory_msgs::JointTrajectory &grasp_pose)
{
  grasp_pose.joint_names.reserve(target_values.size());
  grasp_pose.points.resize(1);
  grasp_pose.points[0].positions.reserve(target_values.size());

  for(std::map<std::string, double>::iterator it = target_values.begin(); it != target_values.end(); ++it){
    grasp_pose.joint_names.push_back(it->first);
    grasp_pose.points[0].positions.push_back(it->second);
  }
  grasp_pose.points[0].time_from_start = duration;
}

bool serviceCB(moveit_msgs::GraspPlanning::Request &req, moveit_msgs::GraspPlanning::Response &res)
{
  moveit::planning_interface::MoveGroupInterface move_group(req.group_name);
  moveit::planning_interface::MoveGroupInterface gripper(move_group.getRobotModel()->getEndEffectors()[0]->getName());

  moveit_msgs::Grasp grasp;
  grasp.id = "grasp";

  jointValuesToJointTrajectory(gripper.getNamedTargetValues("basic_open"), ros::Duration(3.0), grasp.pre_grasp_posture);
  jointValuesToJointTrajectory(gripper.getNamedTargetValues("basic_closed"), ros::Duration(3.0), grasp.grasp_posture);

  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = req.target.id;
  //gripper needs to roll around 180°, so that plugs are at the top
  pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, 0.0);
  pose.pose.position.z = 0.0;
  pose.pose.position.x = -0.015;
  grasp.grasp_pose = pose;

  grasp.pre_grasp_approach.min_distance = 0.05;
  grasp.pre_grasp_approach.desired_distance = 0.18;
  grasp.pre_grasp_approach.direction.header.frame_id = "tool0";
  grasp.pre_grasp_approach.direction.vector.x = 1.0;

  grasp.post_grasp_retreat.min_distance = 0.05;
  grasp.post_grasp_retreat.desired_distance = 0.1;
  grasp.post_grasp_retreat.direction.header.frame_id = move_group.getPlanningFrame();
  grasp.post_grasp_retreat.direction.vector.z = 1.0;

  res.grasps.push_back(grasp);
  res.error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;

  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "plan_grasps_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("plan_grasps", serviceCB);
  ros::spin();

  return 0;
}

