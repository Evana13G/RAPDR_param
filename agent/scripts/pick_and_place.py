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

def addnoise_pose():
    overhead_orientation = Quaternion(
                            x=-0.0249590815779,
                            y=0.999649402929,
                            z=0.00737916180073,
                            w=0.00486450832011)
    #pose = Pose(position= Point(x=0.7, y=0.15, z=-0.129), orientation=overhead_orientation)
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
	
def pick(pose, filename):
    fn = "ur5_grasp_model_" + filename
    filename = "ur5_pick_model_" + filename
    #rps = start_rosbag_recording(fn)
    time.sleep(0.5)
    gripper_open()
    approach(pose)
    move_to_pose(pose)
    gripper_close()
    #stop_rosbag_recording(rps)
    #rosbag_process = start_rosbag_recording(filename)
    time.sleep(0.5)
    approach(pose)
    #stop_rosbag_recording(rosbag_process)

def place(pose, filename):
    fn = "ur5_place_model" + filename
    #rosbag_process = start_rosbag_recording(fn)
    approach(pose)
    move_to_pose(pose)
    gripper_open()
    #stop_rosbag_recording(rosbag_process)
    approach(pose)

robot = moveit_commander.RobotCommander()
arm_group = moveit_commander.MoveGroupCommander("manipulator")
grp_group = moveit_commander.MoveGroupCommander("gripper")


def main():
    rospy.init_node('pick_and_place_node', anonymous=True)
    
    filename = '0'
    num_of_run = 1  

    move_to_start()
    init()

    # block_pose = Pose(position=Point(x=-0.2, y= 0, z=0.6))

    # for iBlock in range(0,12):

    #     print("Block " + str(iBlock))
    #     filename = str(iBlock)

    #     for run in range(0, num_of_run):
    #         if(not rospy.is_shutdown()):
    #             print("Picking up block")
    #             pick(block_pose, filename)
    #             placePose = addnoise_pose()
    #             print("Placing block")
    #             place(placePose, filename)
    #             gripper_open()
    #             move_to_start()


    #             #delete_gazebo_block()
    #             #load_gazebo_block(filename)
    #         else:
    #             break   
    rospy.spin()
    return 0

if __name__ == '__main__':
    sys.exit(main())