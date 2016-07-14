import rospy

from math import *
import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from shape_msgs.msg import SolidPrimitive

import ompl
import robotiq_s_model_control import *
import actionlib
import tf

ROBOT_ATTACHED = False

def grab_bottle(position):
	robot = moveit_commander.RobotCommander()
	scene = moveit_commander.PlanningSceneInterface()
	group = moveit_commander.MoveGroupCommander("arm")

	display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory)
	rospy.sleep(10)

	#Create and add bottle collision object
	bottle, pose_bottle = get_bottle()
	scene.world.collision_objects.append(bottle)
	
	#Compute pre-grasp position for bottle
	pose_target = geometry_msgs.msg.PoseStamped()
	pose_target.header.frame_id = "table_top"
	pose_target.position.x = pose_bottle.position.x - 10
	pose_target.position.y = pose_bottle.position.y
	pose_target.position.z = pose_bottle.position.z + 3 
	#set orientation so that z axis of gripper points downwards
	# TODO: Create euler angles and convert to quaternions
	#pose_target.orientation.x = 1
	#pose_target.orientation.y = 0
	#pose_target.orientation.z = 0
	#pose_target.orientation.w = pi

	#set target position
	group.set_pose_target(pose_target)
	plan1 = group.plan()
	# Uncomment below line when working with a real robot
	# group.go(wait=True)
	rospy.sleep(5)

def get_bottle():
	bottle_height = 15.0

	pose_bottle = geometry_msgs.msg.Pose()
	pose_bottle.orientation.w = 1;
	pose_bottle.position.x = 0;
	pose_bottle.position.y = 0;
	pose_bottle.position.z = bottle_height / 2.0

	bottle = moveit_msgs.msg.CollisionObject()
	bottle.header.frame_id = 'table_top'
	bottle.id = "bottle"

	cylinder = SolidPrimitive()
	cylinder.type = cylinder.CYLINDER
	cylinder.dimensions = (0.076, 0.15)
	bottle.primitives.append(cylinder)
	bottle.primitive_poses.append(pose_bottle)
	return bottle, pose_bottle

if __name__="__main__":
	moveit_commander.roscpp_initialize(sys.argv)
	rospy.init_node('manipulation', anonymous=True)
	rospy.loginfo('Starting Manipulation Action Server')
	ManipulationAction(rospy.get_name())
	rospy.spin()



