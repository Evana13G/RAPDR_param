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

# SRVPROXY_move_to_start = rospy.ServiceProxy('move_to_start_srv', MoveToStartSrv)
# SRVPROXY_open_gripper = rospy.ServiceProxy('open_gripper_srv', OpenGripperSrv)
# SRVPROXY_close_gripper = rospy.ServiceProxy('close_gripper_srv', CloseGripperSrv)
# SRVPROXY_approach = rospy.ServiceProxy('approach_srv', ApproachSrv)

SRVPROXY_push = rospy.ServiceProxy('push_srv', PushSrv)

def setPoseCup(data):
    global CupPose
    CupPose = data
    
def setPoseCover(data):
    global CoverPose
    CoverPose = data

def handle_pushObject(req):

    obj = req.objectName

    if obj == 'cup': 
        poseTo = CupPose.pose
    elif obj == 'cover':
        poseTo = CoverPose.pose
    else:
        poseTo = CoverPose.pose

    startPose = copy.deepcopy(poseTo)
    startPose.position.y = poseTo.position.y - 0.3

    endPose = copy.deepcopy(poseTo)
    endPose.position.y = poseTo.position.y + 0.3

    SRVPROXY_push(startPose, endPose)

    return PushObjectSrvResponse(1)


def main():
    rospy.init_node("push_object_node")

    rospy.Subscriber("cover_pose", PoseStamped, setPoseCover)
    rospy.Subscriber("cup_pose", PoseStamped, setPoseCup)
 
    s = rospy.Service("push_object_srv", PushObjectSrv, handle_pushObject)

    rospy.spin()
    
    return 0

if __name__ == '__main__':
    sys.exit(main())
