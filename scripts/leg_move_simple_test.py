#! /usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import geometry_msgs.msg
import trajectory_msgs.msg
import tf

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander("leg_1")
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory, queue_size=1)

pose_target = geometry_msgs.msg.Pose()
pose_target.position.x = 0.3
pose_target.position.y = 0.3
pose_target.position.z = 0.2
group.set_pose_target(pose_target)

rospy.loginfo("Planning trajectory to pose 1")
plan1 = group.plan()
rospy.loginfo("DONE planning trajectory to pose 1")
rospy.loginfo("Executing trajectory to pose 1")
group.go(wait=True)
rospy.loginfo("Done Executing Trajectory to Pose 1")

pose_target = geometry_msgs.msg.Pose()
roll = 0.0
pitch = 0.0
yaw = 0.0
quaternion = tf.transformations.quaternion_from_euler(roll,pitch,yaw)
pose_target.orientation.x = quaternion[0]
pose_target.orientation.y = quaternion[1]
pose_target.orientation.z = quaternion[2]
pose_target.position.x = -0.3
pose_target.position.y = -0.3
pose_target.position.z = 0.2
group.set_pose_target(pose_target)

rospy.loginfo("Planning trajectory to pose 2")
plan1 = group.plan()
rospy.loginfo("DONE planning trajectory to pose 2")
rospy.loginfo("Executing trajectory to pose 2")
group.go(wait=True)
rospy.loginfo("Done Executing Trajectory to Pose 2")

moveit_commander.roscpp_shutdown()