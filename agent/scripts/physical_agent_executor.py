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
from util.physical_agent import PhysicalAgent

pa = None

def move_to_start(req):
    return MoveToStartSrvResponse(pa.move_to_start())

def open_gripper(req):
    return OpenGripperSrvResponse(pa.gripper_open(req.position))
    
def close_gripper(req):
    return CloseGripperSrvResponse(pa.gripper_close(req.position))

def approach(req):
    return ApproachSrvResponse(pa.approach(req.pose))

def push(req):
    return PushSrvResponse(pa.push_from_side(req.startPose, req.endPose))

def main():
    rospy.init_node("physical_agent_node")

    global pa
    pa = PhysicalAgent()
    s_1 = rospy.Service("move_to_start_srv", MoveToStartSrv, move_to_start)
    s_2 = rospy.Service("open_gripper_srv", OpenGripperSrv, open_gripper)
    s_2 = rospy.Service("close_gripper_srv", CloseGripperSrv, close_gripper)
    s_3 = rospy.Service("approach_srv", ApproachSrv, approach)

    # Action Primitives
    s_4 = rospy.Service("push_srv", PushSrv, push)

    rospy.spin()

    return 0 
####################################################################################################

if __name__ == "__main__":
    main()
