#!/usr/bin/env python

import argparse
import struct
import sys
import copy
import numpy as np
import rospy
import rospkg
import time
import random
import os
import subprocess, signal
import moveit_commander
import moveit_msgs.msg


from math import pi
from moveit_commander.conversions import pose_to_list

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
    String
)

# from tf.transformations import *

##################################################################

class PhysicalAgent(object):
    def __init__(self, hover_distance = 0.15, verbose=False):
        self._hover_distance = hover_distance # in meters
        self._verbose = verbose # bool
        self._robot = moveit_commander.RobotCommander()
        self._arm_group = moveit_commander.MoveGroupCommander("manipulator")
        self._grp_group = moveit_commander.MoveGroupCommander("gripper")

####################################################################################################

    def pick(self, pose):
        self.gripper_open()
        self.approach(pose)
        self.move_to_pose(pose)
        self.gripper_close()
        self.approach(pose)

    def place(self, pose):
        self.approach(pose)
        self.move_to_pose(pose)
        self.gripper_open()
        self.approach(pose)

####################################################################################################

    def gripper_open(self, position=0):
        self._grp_group.set_joint_value_target([9.800441184282249e-05, -9.800441184282249e-05, 9.800441184282249e-05, 9.800441184282249e-05, -9.800441184282249e-05, 9.800441184282249e-05])
        self._grp_group.go(wait=True)
        if self._verbose:
            print("Gripper opened")
        return 1

    def gripper_close(self, position=1):
        self._grp_group.set_joint_value_target([0.8039005131791948, -0.8039005131791948, 0.8039005131791948, 0.8039005131791948, -0.8039005131791948, 0.8039005131791948])
        self._grp_group.go(wait=True)
        if self._verbose:
            print("Gripper closed")
        return 1

    def move_to_start(self): 
        self._arm_group.set_named_target('up') # See UR5 config
        self._arm_group.go(wait=True)
        if self._verbose:
            print("Moved to start")
        return 1






    def move_to_pose(self, pose):
        self._arm_group.set_pose_target(pose)
        self._arm_group.go(wait=True)

    def approach(self, pose):
        approachPose = copy.deepcopy(pose)
        hover_distance = 0.15
        approachPose.position.z = approachPose.position.z + hover_distance
        self._arm_group.set_pose_target(approachPose)
        self._arm_group.go(wait=True)

# ####################################################################################################

    def _addnoise_pose(self):
        overhead_orientation = Quaternion(
                                x=-0.0249590815779,
                                y=0.999649402929,
                                z=0.00737916180073,
                                w=0.00486450832011)
        pose = Pose(position= Point(x=-0.3, y=0, z=0.7), orientation=overhead_orientation)
        x = random.uniform(-0.09, 0.09)
        y = random.uniform(-0.09, 0.09)
        pose.position.x = pose.position.x + x
        pose.position.y = pose.position.y + y
        return pose


