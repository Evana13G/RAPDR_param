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
import tf

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


##################################################################

class PhysicalAgent(object):
    def __init__(self, hover_distance = 0.15, verbose=False):
        self._hover_distance = hover_distance # in meters
        self._verbose = verbose # bool
        self._robot = moveit_commander.RobotCommander()
        self._arm_group = moveit_commander.MoveGroupCommander("manipulator")
        self._grp_group = moveit_commander.MoveGroupCommander("gripper")
        self._scene = moveit_commander.PlanningSceneInterface()
        
        self._base_constraint = Constraints()
        self._gripper_down_orientation = tf.transformations.quaternion_from_euler(0, 3.1415/2, 0)


####################################################################################################



####################################################################################################

    def gripper_open(self, position=0):
        self._move_gripper(position)
        if self._verbose:
            print("Gripper opened")
        return 1

    def gripper_close(self, position=1):
        self._move_gripper(position)
        if self._verbose:
            print("Gripper closed")
        return 1

    def move_to_start(self): 
        # self._arm_group.set_named_target('up') # See UR5 config
        initial_joints = [1.6288508606798757e-05, -1.5999785607582009, -8.699362263797639e-05, -6.581909286129672e-05, -1.570796, 4.259259461747433e-05]
        self._arm_group.set_joint_value_target(initial_joints)
        self._arm_group.go(wait=True)
        print("Moved to start")
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

#####################################################################################################
######################### Internal Functions
    def _move_gripper(value):
        # Value is from 0 to 1, where 0 is an open gripper, and 1 is a closed gripper
        jointAngles = [(1*value), (-1*value), (1*value), (1*value), (-1*value), (1*value)]
        self._grp_group.set_joint_value_target(jointAngles)
        self._grp_group.go(wait=True)

    def _enable_base_constraint():
        arm_group.set_path_constraints(self._base_constraint)

    def _disable_constraint():
        arm_group.set_path_constraints(None)

    def _init_base_constraint():
        joint_constraint = JointConstraint()
        joint_constraint.joint_name = "shoulder_pan_joint"
        joint_constraint.position = 0
        joint_constraint.tolerance_above = 0.7854
        joint_constraint.tolerance_below = 0.7854
        joint_constraint.weight = 1
        self._base_constraint.joint_constraints.append(joint_constraint)