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

robot = moveit_commander.RobotCommander()
arm_group = moveit_commander.MoveGroupCommander("manipulator")
grp_group = moveit_commander.MoveGroupCommander("gripper")


CupPose = None
CoverPose = None

def setPoseCup(data):
    global CupPose
    CupPose = data

def setPoseCover(data):
    global CoverPose
    CoverPose = data
    

def addnoise_pose():
    overhead_orientation = Quaternion(
                            x=-0.0249590815779,
                            y=0.999649402929,
                            z=0.00737916180073,
                            w=0.00486450832011)
    pose = Pose(position= Point(x=-0.3, y=0, z=0.8), orientation=overhead_orientation)
    x = random.uniform(-0.09, 0.09)
    y = random.uniform(-0.09, 0.09)
    pose.position.x = pose.position.x + x
    pose.position.y = pose.position.y + y
    return pose

def move_to_start():
    arm_group.set_named_target('up')
    arm_group.go(wait=True)
    print("Moved to start")

def gripper_open():
    grp_group.set_joint_value_target([9.800441184282249e-05, -9.800441184282249e-05, 9.800441184282249e-05, 9.800441184282249e-05, -9.800441184282249e-05, 9.800441184282249e-05])
    grp_group.go(wait=True)

def gripper_close():
    grp_group.set_joint_value_target([0.8039005131791948, -0.8039005131791948, 0.8039005131791948, 0.8039005131791948, -0.8039005131791948, 0.8039005131791948])
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
    gripper_open()
    approach(pose)
    move_to_pose(pose)
    gripper_close()
    approach(pose)

def place(pose):
    approach(pose)
    move_to_pose(pose)
    gripper_open()
    approach(pose)

def main():
    rospy.init_node('pick_and_place_node', anonymous=True)
    
    filename = '0'
    num_of_run = 1  

    rospy.Subscriber("cover_pose", PoseStamped, setPoseCover)
    rospy.Subscriber("cup_pose", PoseStamped, setPoseCup)


    while(CoverPose == None):
        rospy.sleep(1)

    item_to_pickup = CoverPose.pose

    for i in range(0,3):

        if(not rospy.is_shutdown()):
            print("Picking up block")
            pick(item_to_pickup)
            placePose = addnoise_pose()
            print("Placing block")
            place(placePose)
            gripper_open()
            move_to_start()
        else:
            break   


    # rospy.spin()
    return 0

if __name__ == '__main__':
    sys.exit(main())