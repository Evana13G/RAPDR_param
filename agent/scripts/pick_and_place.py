#!/usr/bin/env python
import rospy
import moveit_commander
import argparse
import struct
import sys
import copy
import time
import random
import os
import subprocess, signal
import tf

import rospkg

from gazebo_msgs.srv import (
    SpawnModel,
    DeleteModel,
    GetModelState,
    GetLinkState,
)

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)

from std_msgs.msg import (
    Header,
    Empty,
)

from trajectory_msgs.msg import (
    JointTrajectory,
    JointTrajectoryPoint,
)

from moveit_msgs.msg import (
    DisplayRobotState,
    RobotState,
    DisplayTrajectory,
    Grasp,
    Constraints,
    JointConstraint,
    PositionConstraint,
    OrientationConstraint,
    VisibilityConstraint,
    GripperTranslation,
    JointLimits,
    LinkPadding,
)


robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
arm_group = moveit_commander.MoveGroupCommander("manipulator")
grp_group = moveit_commander.MoveGroupCommander("gripper")

constraints = Constraints()
constraints.name = "general_constraints"


CupPose = None
CoverPose = None

def setPoseCup(data):
    global CupPose
    CupPose = data

def setPoseCover(data):
    global CoverPose
    CoverPose = data

def move_to_start():
    #arm_group.set_named_target('up')
    initial_joints = [1.6288508606798757e-05, -1.5999785607582009, -8.699362263797639e-05, -6.581909286129672e-05, -1.570796, 4.259259461747433e-05]

    arm_group.set_joint_value_target(initial_joints)
    arm_group.go(wait=True)
    print("Moved to start")

def gripper_open():
    grp_group.set_joint_value_target([9.800441184282249e-05, -9.800441184282249e-05, 9.800441184282249e-05, 9.800441184282249e-05, -9.800441184282249e-05, 9.800441184282249e-05])
    grp_group.go(wait=True)

def gripper_close():
    grp_group.set_joint_value_target([0.8039005131791948, -0.8039005131791948, 0.8039005131791948, 0.8039005131791948, -0.8039005131791948, 0.8039005131791948])
    grp_group.go(wait=True)

def move_gripper(value):
    # Value is from 0 to 1, where 0 is an open gripper, and 1 is a closed gripper
    jointAngles = [(1*value), (-1*value), (1*value), (1*value), (-1*value), (1*value)]
    grp_group.set_joint_value_target(jointAngles)
    grp_group.go(wait=True)

def move_to_pose(pose):
    arm_group.set_pose_target(pose)
    arm_group.go(wait=True)

def approach(pose):
    approachPose = copy.deepcopy(pose)
    hover_distance = 0.15
    approachPose.position.z = approachPose.position.z + hover_distance
    arm_group.set_pose_target(approachPose)
    arm_group.go(wait=True)
    
def pick(pose):
    #gripper_open()
    print("Open Gripper")
    move_gripper(0)
    print("Approach block")
    approach(pose)
    #print("Move to pose")
    #move_to_pose(pose)
    rospy.sleep(1)
    print("Close gripper")
    move_gripper(0.33)
    #gripper_close()
    rospy.sleep(1)
    print("Move to start")
    move_to_start()
    #approach(pose)

def place(pose):
    #Needs work probably
    approach(pose)
    move_to_pose(pose)
    gripper_open()
    approach(pose)

def printState():
    print("Robot State:")
    print(robot.get_current_state())


def init_constraint():
    joint_constraint = JointConstraint()
    joint_constraint.joint_name = "shoulder_pan_joint"
    joint_constraint.position = 0
    joint_constraint.tolerance_above = 0.7854
    joint_constraint.tolerance_below = 0.7854
    joint_constraint.weight = 1
    constraints.joint_constraints.append(joint_constraint)

def init_push_constraint():
    shoulder_constraint = JointConstraint()
    shoulder_constraint.joint_name = "shoulder_lift_joint"
    shoulder_constraint.position = -0.37754034809990955 #Obtained from looking at a good run in Gazebo
    shoulder_constraint.tolerance_above = 0.7853 #45 degrees
    shoulder_constraint.tolerance_below = 0.7853
    shoulder_constraint.weight = 1
    constraints.joint_constraints.append(shoulder_constraint)

    elbow_constraint = JointConstraint()
    elbow_constraint.joint_name = "elbow_joint"
    elbow_constraint.position = 0.6780465480943496
    elbow_constraint.tolerance_above = 0.7853
    elbow_constraint.tolerance_below = 0.7853
    elbow_constraint.weight = 1
    constraints.joint_constraints.append(elbow_constraint)

    # wrist_constraint = JointConstraint()
    # wrist_constraint.joint_name = "wrist_3_joint"
    # wrist_constraint.position = 0
    # wrist_constraint.tolerance_above = 0.1745
    # wrist_constraint.tolerance_below = 0.1745
    # wrist_constraint.weight = 1
    # constraints.joint_constraints.append(wrist_constraint)


def enable_base_constraint():
    init_constraint()
    arm_group.set_path_constraints(constraints)

def disable_base_constraint():
    arm_group.set_path_constraints(None)

# def init_push_sideways_constraints():

#     upper_arm_link_orientation = tf.transformations.quaternion_from_euler(-0.000501, 1.193249, 0.255100)
#     upper_arm_link_pose = Pose()
#     upper_arm_link_pose.position.x = -0.834252
#     upper_arm_link_pose.position.y = 0.131472
#     upper_arm_link_pose.position.z = 0.689158
#     upper_arm_link_pose.orientation.x = upper_arm_link_orientation[0]
#     upper_arm_link_pose.orientation.y = upper_arm_link_orientation[1]
#     upper_arm_link_pose.orientation.z = upper_arm_link_orientation[2]
#     upper_arm_link_pose.orientation.w = upper_arm_link_orientation[3]

#     upper_arm_push_constraint = OrientationConstraint()
#     upper_arm_push_constraint.header = Header()
#     upper_arm_push_constraint.header.frame_id = "/world"
#     #orientation_constraint.link_name = self.arm_group.get_end_effector_link()
#     upper_arm_push_constraint.link_name = "upper_arm_link"
#     upper_arm_push_constraint.orientation = upper_arm_link_pose.orientation
#     upper_arm_push_constraint.absolute_x_axis_tolerance = 0.4
#     upper_arm_push_constraint.absolute_y_axis_tolerance = 1
#     upper_arm_push_constraint.absolute_z_axis_tolerance = 0.2
#     #orientation_constraint.absolute_z_axis_tolerance = 3.14 #ignore this axis
#     upper_arm_push_constraint.weight = 1

#     constraints.orientation_constraints.append(upper_arm_push_constraint)

#     forearm_link_orientation = tf.transformations.quaternion_from_euler(-3.141569, 1.270296, -2.885950)
#     forearm_link_pose = Pose()
#     forearm_link_pose.position.x = -0.421766
#     forearm_link_pose.position.y = 0.115586
#     forearm_link_pose.position.z = 0.845854
#     forearm_link_pose.orientation.x = forearm_link_orientation[0]
#     forearm_link_pose.orientation.y = forearm_link_orientation[1]
#     forearm_link_pose.orientation.z = forearm_link_orientation[2]
#     forearm_link_pose.orientation.w = forearm_link_orientation[3]

    
#     forearm_link_pose = Pose(position=Point(x=-0.421766, y= 0.115586, z=0.845854), orientation=Quaternion(x=forearm_link_orientation[0], y=forearm_link_orientation[1], z=forearm_link_orientation[2], w=forearm_link_orientation[3]))
#     forearm_push_constraint = OrientationConstraint()
#     forearm_push_constraint.header = Header()
#     forearm_push_constraint.header.frame_id = "/world"
#     forearm_push_constraint.link_name = "forearm_link"
#     forearm_push_constraint.orientation = forearm_link_pose.orientation
#     forearm_push_constraint.absolute_x_axis_tolerance = 0.4
#     forearm_push_constraint.absolute_y_axis_tolerance = 1
#     forearm_push_constraint.absolute_z_axis_tolerance = 0.2
#     upper_arm_push_constraint.weight = 1
#     constraints.orientation_constraints.append(forearm_push_constraint)



def enable_push_sideways_constraints():
    #init_push_sideways_constraints()
    init_push_constraint
    arm_group.set_path_constraints(constraints)

def disable_push_sideways_constraints():
    arm_group.set_path_constraints(None)


def push_from_side(startPose, endPose):
    disable_push_sideways_constraints()
    print("Pushing from side")
    print("Moving to first pose")
    move_to_pose(startPose)
    enable_push_sideways_constraints()
    print("Moving to end pose")
    move_to_pose(endPose)
    disable_push_sideways_constraints()


def main():
    rospy.init_node('pick_and_place_node', anonymous=True)

    rospy.Subscriber("cover_pose", PoseStamped, setPoseCover)
    rospy.Subscriber("cup_pose", PoseStamped, setPoseCup)


    while (CoverPose == None):
        rospy.sleep(1)


    #move_to_start()

    testPose = copy.deepcopy(CoverPose.pose)
    testPose.position.x = testPose.position.x - 0.4
    testPose.position.z = testPose.position.z + 0.2
    testPose.orientation.w = 1


    testPoseLinear = copy.deepcopy(testPose)
    testPose.position.x = testPose.position.x + 0.5


    item_to_pickup = copy.deepcopy(CoverPose.pose)
    item_to_pickup.position.z = item_to_pickup.position.z #+ 0.3
    #item_to_pickup.position.y = item_to_pickup.position.y + 0.3
    downOrientation = tf.transformations.quaternion_from_euler(0, 3.1415/2, 0)
    item_to_pickup.orientation.x = downOrientation[0]
    item_to_pickup.orientation.y = downOrientation[1]
    item_to_pickup.orientation.z = downOrientation[2]
    item_to_pickup.orientation.w = downOrientation[3]



    sidePose1 = copy.deepcopy(CoverPose.pose)
    sidePose1.position.y = item_to_pickup.position.y - 0.3

    sidePose2 = copy.deepcopy(CoverPose.pose)
    sidePose2.position.y = item_to_pickup.position.y + 0.3

    print(item_to_pickup)

    # for i in range(0,3):

    #     if(not rospy.is_shutdown()):
    #         print("Picking up block")
    #         pick(item_to_pickup)
    #         placePose = addnoise_pose()
    #         print("Placing block")
    #         place(placePose)
    #         gripper_open()
    #         move_to_start()
    #     else:
    #         break   

    #printState()

    if not rospy.is_shutdown():

        #move_to_pose(testPose)
        #print("Moving Linearly")
        #move_to_pose(testPoseLinear)
        enable_base_constraint()
        #pick(item_to_pickup)
        push_from_side(sidePose1, sidePose2)
        #printState()

    # rospy.spin()
    return 0

if __name__ == '__main__':
    sys.exit(main())




# Desired joint angles for pushing from side to side
# [shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint,
#  wrist_3_joint, gripper_finger1_inner_knuckle_joint, gripper_finger1_finger_tip_joint,
#  gripper_finger1_joint, gripper_finger2_inner_knuckle_joint, gripper_finger2_finger_tip_joint,
#  gripper_finger2_joint]
#  position: [0.2548506868586795, -0.37754034809990955, 0.6780465480943496, -0.29937058415319306, 1.8228079943929831, -3.1409085487141875, 0.0006364657950719277, -0.0006364657950719277, 0.0006364657950719277, 0.0006364657950719277, -0.0006364657950719277, 0.0006364657950719277]

#shoulder lift joint
#elbow joint 

#links: upper_arm_link
#forearm_link

# def init_push_sideways_constraints():

#     upper_arm_link_orientation = tf.transformations.quaternion_from_euler(-0.000501, 1.193249, 0.255100)
#     upper_arm_link_pose = Pose(position=Point(x=-0.834252, y=0.131472, z=0.689158), orientation=Quaternion(x=upper_arm_link_orientation[0], 
#         y=upper_arm_link_orientation[1]), z = upper_arm_link_orientation[2], w = upper_arm_link_orientation[3])

#     upper_arm_push_constraint = OrientationConstraint()
#     #orientation_constraint.header = pose.header
#     #orientation_constraint.link_name = self.arm_group.get_end_effector_link()
#     upper_arm_push_constraint.link_name = "upper_arm_link"
#     upper_arm_push_constraint.orientation = upper_arm_link_pose.orientation
#     upper_arm_push_constraint.absolute_x_axis_tolerance = 0.4
#     upper_arm_push_constraint.absolute_y_axis_tolerance = 1
#     upper_arm_push_constraint.absolute_z_axis_tolerance = 0.2
#     #orientation_constraint.absolute_z_axis_tolerance = 3.14 #ignore this axis
#     upper_arm_push_constraint.weight = 1

#     constraints.orientation_constraints.append(upper_arm_push_constraint)

#     forearm_link_orientation = tf.transformations.quaternion_from_euler(-3.141569, 1.270296, -2.885950)
#     forearm_link_pose = Pose(position=Point(x=-0.421766, y= 0.115586, z=0.845854), orientation=Quaternion(x=forearm_link_orientation[0],
#         y=forearm_link_orientation[1], z=forearm_link_orientation[2], w=forearm_link_orientation[3]))
#     forearm_push_constraint = OrientationConstraint()
#     forearm_push_constraint.link_name = "forearm_link"
#     forearm_push_constraint.orientation = forearm_link_pose.orientation
#     forearm_push_constraint.absolute_x_axis_tolerance = 0.4
#     forearm_push_constraint.absolute_y_axis_tolerance = 1
#     forearm_push_constraint.absolute_z_axis_tolerance = 0.2
#     upper_arm_push_constraint.weight = 1
#     constraints.orientation_constraints.append(forearm_push_constraint)



# def enable_push_sideways_constraints():
#     arm_group.set_path_constraints(push_sideways_constraints)

# def disable_push_sideways_constraints():
#     arm_group.set_path_constraints(None)





#upper_arm_link pose: 
#x: -0.834252
#y: 0.131472
#z: 0.689158
#roll: -0.000501
#pitch: 1.193249
#yaw: 0.255100

#forearm_link pose:
#x: -0.421766
#y: 0.115586
#z: 0.845854
#roll: -3.141569
#pitch: 1.270296
#yaw: -2.885950


