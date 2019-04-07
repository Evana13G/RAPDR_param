#!/usr/bin/env python

import argparse
import struct
import sys
import copy
import numpy as np

import rospy
import rospkg

from gazebo_msgs.srv import (
    SpawnModel,
    DeleteModel,
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

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

from tf.transformations import *

import baxter_interface

from agent.srv import *
from util.physical_agent import PhysicalAgent


CupPose = None

def setPoseCup(data):
    global CupPose
    CupPose = data
    


def hoverOverPose(poseStmpd):
	newPose = copy.deepcopy(poseStmpd)
	newPose.pose.position.z += 0.25
	return newPose

def handle_pushObject(req):
    
    appr = rospy.ServiceProxy('approach_srv', ApproachSrv)

    limb = req.limb
    poseTo = CupPose
    hover_distance = 0.15


    if limb == 'left_gripper':
      appr('left', hoverOverPose(poseTo))
      appr('left', poseTo)
      appr('left', hoverOverPose(poseTo))
    else:
      appr('right', hoverOverPose(poseTo))
      appr('right', poseTo)
      appr('right', hoverOverPose(poseTo))
    
    return PressButtonSrvResponse(1)


def main():
    rospy.init_node("press_button_node")
    rospy.wait_for_message("/robot/sim/started", Empty)

    rospy.wait_for_service('approach_srv', timeout=60)

    rospy.Subscriber("cover_pose", PoseStamped, setPoseCup)
 
    s = rospy.Service("push_object_srv", PushObjectSrv, handle_pushObject)

    rospy.spin()
    
    return 0

if __name__ == '__main__':
    sys.exit(main())
