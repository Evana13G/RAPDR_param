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

from agent.srv import *
from util.physical_agent import PhysicalAgent

CupPose = None
CoverPose = None

SRVPROXY_move_to_start = rospy.ServiceProxy('move_to_start_srv', MoveToStartSrv)
SRVPROXY_open_gripper = rospy.ServiceProxy('open_gripper_srv', OpenGripperSrv)
SRVPROXY_close_gripper = rospy.ServiceProxy('close_gripper_srv', CloseGripperSrv)
SRVPROXY_approach = rospy.ServiceProxy('approach_srv', ApproachSrv)

def setPoseCup(data):
    global CupPose
    CupPose = data
    
def setPoseCover(data):
    global CoverPose
    CoverPose = data

def handle_pushObject(req):

    obj = req.objectName

    if obj == 'cup': 
        poseTo = CupPose
    elif obj == 'cover':
        poseTo = CoverPose
    else:
        poseTo = CoverPose

    SRVPROXY_approach(poseTo)

    return PushObjectSrvResponse(1)


def main():
    rospy.init_node("push_button_node")

    rospy.Subscriber("cover_pose", PoseStamped, setPoseCover)
    rospy.Subscriber("cup_pose", PoseStamped, setPoseCup)
 
    s = rospy.Service("push_object_srv", PushObjectSrv, handle_pushObject)

    rospy.spin()
    
    return 0

if __name__ == '__main__':
    sys.exit(main())
