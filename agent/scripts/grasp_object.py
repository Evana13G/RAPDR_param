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

from tf.transformations import *

from agent.srv import *

CupPose = None
CoverPose = None

SRVPROXY_move_to_start = rospy.ServiceProxy('move_to_start_srv', MoveToStartSrv)
SRVPROXY_open_gripper = rospy.ServiceProxy('open_gripper_srv', OpenGripperSrv)
SRVPROXY_close_gripper = rospy.ServiceProxy('close_gripper_srv', CloseGripperSrv)
SRVPROXY_approach = rospy.ServiceProxy('approach_srv', ApproachSrv)

def setPoseCup(data):
    global CupPose
    CupPose = data.pose

def setPoseCover(data):
    global CoverPose
    CoverPose = data.pose

def handle_graspObject(req):
    
    global object_name
    object_name = req.objectName
    poseTo = None
    
    if object_name == "cup":
        poseTo = CupPose
    elif object_name == "cover":
        poseTo = CoverPose
    else:
        poseTo = CoverPose

    SRVPROXY_move_to_start()
    SRVPROXY_open_gripper()
    SRVPROXY_approach(poseTo)
    SRVPROXY_close_gripper()
    
    return GraspObjectSrvResponse(1)


def main():
    rospy.init_node("grasp_object_node")

    rospy.Subscriber("cover_pose", PoseStamped, setPoseCover)
    rospy.Subscriber("cup_pose", PoseStamped, setPoseCup)

    s = rospy.Service("grasp_object_srv", GraspObjectSrv, handle_graspObject)

    rospy.spin()
    
    return 0

if __name__ == '__main__':
    sys.exit(main())
