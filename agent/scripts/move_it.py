#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

def move_it_test(req):
    return MoveItTestSrvResponse(1)

def main():
    rospy.init_node("move_it_node")
    rospy.wait_for_message("/robot/sim/started", Empty)

    s = rospy.Service("move_it_srv", MoveToStartSrv, move_it_test)

    rospy.spin()

    return 0 
####################################################################################################

if __name__ == "__main__":
    main()
