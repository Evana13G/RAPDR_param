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

# SRVPROXY_move_to_start = rospy.ServiceProxy('move_to_start_srv', MoveToStartSrv)
# SRVPROXY_open_gripper = rospy.ServiceProxy('open_gripper_srv', OpenGripperSrv)
# SRVPROXY_close_gripper = rospy.ServiceProxy('close_gripper_srv', CloseGripperSrv)
# SRVPROXY_approach = rospy.ServiceProxy('approach_srv', ApproachSrv)

SRVPROXY_grasp = rospy.ServiceProxy('grasp_srv', GraspSrv)

def setPoseCup(data):
    global CupPose
    CupPose = data

def setPoseCover(data):
    global CoverPose
    CoverPose = data

def handle_graspObject(req):
    
    global object_name
    object_name = req.objectName
    poseTo = None
    
    if object_name == "cup":
        poseTo = CupPose.pose
    elif object_name == "cover":
        poseTo = CoverPose.pose
    else:
        poseTo = CoverPose.pose

    graspPose = copy.deepcopy(poseTo)
    # graspPose.position.x = poseTo.position.x - 0.17
    # graspPose.position.y = poseTo.position.y - 0.16
    graspPose.position.z = poseTo.position.z + 0.1

    SRVPROXY_grasp(graspPose)
    
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
