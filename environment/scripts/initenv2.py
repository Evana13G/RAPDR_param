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

from agent.srv import *

pub = True

pub_bowl_pose = rospy.Publisher('bowl_pose', PoseStamped, queue_size = 10)
pub_cover_pose = rospy.Publisher('cover_pose', PoseStamped, queue_size = 10)
pub_block_pose = rospy.Publisher('block_pose', PoseStamped, queue_size = 10)

def load_gazebo_models(bowl_pose=Pose(position=Point(x=0.0, y=0.0, z=0.6)),
                       block_pose=Pose(position=Point(x=0.0, y=0.0, z=0.6)),
                       cover_pose=Pose(position=Point(x=0.0, y=0.0, z=0.6)),
                       reference_frame="world"):
    
    # Get Models' Path
    print("Loading Gazebo Models")

    model_path = rospkg.RosPack().get_path('environment')+"/models/"
    
    #bowl
    bowl_xml = ''
    with open (model_path + "bowl/bowl.urdf", "r") as bowl_file:
        bowl_xml=bowl_file.read().replace('\n', '')
    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    try:
        spawn_urdf = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        resp_urdf = spawn_urdf("bowl", bowl_xml, "/",
                               bowl_pose, reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn URDF service call failed: {0}".format(e))

    #block
    block_xml = ''
    with open (model_path + "block/model.urdf", "r") as block_file:
        block_xml=block_file.read().replace('\n', '')
    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    try:
        spawn_urdf = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        resp_urdf = spawn_urdf("block", block_xml, "/",
                               block_pose, reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn URDF service call failed: {0}".format(e))


    #bowl cover 
    cover_xml = ''
    with open (model_path + "bowl/cover.urdf", "r") as cover_file:
        cover_xml=cover_file.read().replace('\n', '')
    rospy.wait_for_service('/gazebo/spawn_urdf_model')    

    try:
        spawn_urdf = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        resp_urdf = spawn_urdf("cover", cover_xml, "/",
                               cover_pose, reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn URDF service call failed: {0}".format(e))




def delete_gazebo_models():
    try:
        delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        resp_delete = delete_model("bowl")
        resp_delete = delete_model("block")
        resp_delete = delete_model("cover")
    except rospy.ServiceException, e:
        rospy.loginfo("Delete Model service call failed: {0}".format(e))


def init():

    SRVPROXY_move_to_start = rospy.ServiceProxy('move_to_start_srv', MoveToStartSrv)
    
    SRVPROXY_move_to_start()

    load_gazebo_models()
    frameid_var = "/world"


    while pub == True:

        rate = rospy.Rate(10) # 10hz
        rospy.wait_for_service('/gazebo/get_model_state')
        rospy.wait_for_service('/gazebo/get_link_state')

        #### Get plastic_bowl pose
        try:
            cover_ms = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            resp_cover_ms = cover_ms("cover", "");
            pose_cover = resp_cover_ms.pose
            header_cover = resp_cover_ms.header
            header_cover.frame_id = frameid_var
            poseStamped_cover = PoseStamped(header=header_cover, pose=pose_cover)
            pub_cover_pose.publish(poseStamped_cover)
        except rospy.ServiceException, e:
            rospy.logerr("get_model_state for block service call failed: {0}".format(e))


        try:
            bowl_ms = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            resp_bowl_ms = bowl_ms("bowl", "");
            pose_bowl = resp_bowl_ms.pose
            header_bowl = resp_bowl_ms.header
            header_bowl.frame_id = frameid_var
            poseStamped_bowl = PoseStamped(header=header_bowl, pose=pose_bowl)
            pub_bowl_pose.publish(poseStamped_bowl)
        except rospy.ServiceException, e:
            rospy.logerr("get_model_state for block service call failed: {0}".format(e))

        try:
            block_ms = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            resp_block_ms = block_ms("block", "");
            pose_block = resp_block_ms.pose
            header_block = resp_block_ms.header
            header_block.frame_id = frameid_var
            poseStamped_block = PoseStamped(header=header_block, pose=pose_block)
            pub_block_pose.publish(poseStamped_block)
        except rospy.ServiceException, e:
            rospy.logerr("get_model_state for block service call failed: {0}".format(e))

def main():
    rospy.init_node('initialize_environment_node', anonymous=True)
    # rospy.wait_for_message("/robot/sim/started", Empty) # causing the assimp error 

    rospy.on_shutdown(delete_gazebo_models)
    rospy.wait_for_service('move_to_start_srv', timeout=60)

    init()

    rospy.spin()
    return 0

if __name__ == '__main__':
    sys.exit(main())