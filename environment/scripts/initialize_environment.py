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


pub = True

pub_cup_pose = rospy.Publisher('cup_pose', PoseStamped, queue_size = 10)
pub_cover_pose = rospy.Publisher('cover_pose', PoseStamped, queue_size = 10)


def load_gazebo_models(table_pose=Pose(position=Point(x=1.0, y= 0.0, z=0.0)),
                       table_reference_frame="world",
                       # block_pose=Pose(position=Point(x=0.6725, y= 0.1265, z=0.7825)),
                       # cup_pose=Pose(position=Point(x=0.9, y=0.0185, z=0.8)),
                       # cover_pose=Pose(position=Point(x=0.9, y=0.0185, z=1.0)),
                       cup_pose=Pose(position=Point(x=0.0, y=0.0, z=0.7)),
                       cover_pose=Pose(position=Point(x=0.0, y=0.0, z=0.7)),
                       reference_frame="world"):
    
    # Get Models' Path
    print("Loading Gazebo Models")

    model_path = rospkg.RosPack().get_path('environment')+"/models/"
    
    # Load Table SDF
    # table_xml = ''
    # with open (model_path + "cafe_table/model.sdf", "r") as table_file:
    #     table_xml=table_file.read().replace('\n', '')
    
    # Load Blocks URDF
    # block_xml = ''
    # block_path = "block/model.urdf"
    # with open (model_path + block_path, "r") as block_file:
    #     block_xml=block_file.read().replace('\n', '')
  
   # Spawn Table SDF
    # rospy.wait_for_service('/gazebo/spawn_sdf_model')
    # try:
    #     spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    #     resp_sdf = spawn_sdf("cafe_table", table_xml, "/",
    #                          table_pose, table_reference_frame)
    # except rospy.ServiceException, e:
    #     rospy.logerr("Spawn SDF service call failed: {0}".format(e))
   
   # Spawn Block URDF
    # rospy.wait_for_service('/gazebo/spawn_urdf_model')
    # try:
    #     spawn_urdf = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
    #     resp_urdf = spawn_urdf("block", block_xml, "/",
    #                            block_pose, block_reference_frame)
    # except rospy.ServiceException, e:
    #     rospy.logerr("Spawn URDF service call failed: {0}".format(e))


    cup_xml = ''
    with open (model_path + "cup_with_cover/cup_model.urdf", "r") as cup_file:
        cup_xml=cup_file.read().replace('\n', '')
    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    try:
        spawn_urdf = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        resp_urdf = spawn_urdf("cup", cup_xml, "/",
                               cup_pose, reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn URDF service call failed: {0}".format(e))


    cover_xml = ''
    with open (model_path + "cup_with_cover/cover_model.urdf", "r") as cover_file:
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
        # resp_delete = delete_model("cafe_table")
        # resp_delete = delete_model("block")
        resp_delete = delete_model("cup")
        resp_delete = delete_model("cover")
    except rospy.ServiceException, e:
        rospy.loginfo("Delete Model service call failed: {0}".format(e))

def addnoise_pose():
    overhead_orientation = Quaternion(
                            x=-0.0249590815779,
                            y=0.999649402929,
                            z=0.00737916180073,
                            w=0.00486450832011)
    #pose = Pose(position= Point(x=0.7, y=0.15, z=-0.129), orientation=overhead_orientation)
    pose = Pose(position= Point(x=-0.3, y=0, z=0.8), orientation=overhead_orientation)
    x = random.uniform(-0.09, 0.09)
    y = random.uniform(-0.09, 0.09)
    pose.position.x = pose.position.x + x
    pose.position.y = pose.position.y + y
    return pose

def move_to_start():
    arm_group.set_named_target('up')
    arm_group.go(wait=True)
    print("Moved to start")

def gripper_open():
    grp_group.set_joint_value_target([9.800441184282249e-05, -9.800441184282249e-05, 9.800441184282249e-05, 9.800441184282249e-05, -9.800441184282249e-05, 9.800441184282249e-05])
    grp_group.go(wait=True)

def gripper_close():
    grp_group.set_joint_value_target([0.8039005131791948, -0.8039005131791948, 0.8039005131791948, 0.8039005131791948, -0.8039005131791948, 0.8039005131791948])
    grp_group.go(wait=True)

def move_to_pose(pose):
    arm_group.set_pose_target(pose)
    arm_group.go(wait=True)

def approach(pose):
    approachPose = copy.deepcopy(pose)
    hover_distance = 0.15
    approachPose.position.z = approachPose.position.z + hover_distance
    arm_group.set_pose_target(approachPose)
    arm_group.go(wait=True)
	
def pick(pose, filename):
    fn = "ur5_grasp_model_" + filename
    filename = "ur5_pick_model_" + filename
    #rps = start_rosbag_recording(fn)
    time.sleep(0.5)
    gripper_open()
    approach(pose)
    move_to_pose(pose)
    gripper_close()
    #stop_rosbag_recording(rps)
    #rosbag_process = start_rosbag_recording(filename)
    time.sleep(0.5)
    approach(pose)
    #stop_rosbag_recording(rosbag_process)

def place(pose, filename):
    fn = "ur5_place_model" + filename
    #rosbag_process = start_rosbag_recording(fn)
    approach(pose)
    move_to_pose(pose)
    gripper_open()
    #stop_rosbag_recording(rosbag_process)
    approach(pose)

robot = moveit_commander.RobotCommander()
arm_group = moveit_commander.MoveGroupCommander("manipulator")
grp_group = moveit_commander.MoveGroupCommander("gripper")


def init():
    rospy.sleep(3)
    load_gazebo_models()
    frameid_var = "/world"
    while pub == True:

        rate = rospy.Rate(10) # 10hz
        rospy.wait_for_service('/gazebo/get_model_state')
        rospy.wait_for_service('/gazebo/get_link_state')

        #### Get plastic_cup pose
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
            cup_ms = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            resp_cup_ms = cup_ms("cup", "");
            pose_cup = resp_cup_ms.pose
            header_cup = resp_cup_ms.header
            header_cup.frame_id = frameid_var
            poseStamped_cup = PoseStamped(header=header_cup, pose=pose_cup)
            pub_cup_pose.publish(poseStamped_cup)
        except rospy.ServiceException, e:
            rospy.logerr("get_model_state for block service call failed: {0}".format(e))

###################################################################################################
###  START: Gripper stuff 

        # pose_lglf = None
        # pose_lgrf = None
        # hdr = Header(frame_id=frameid_var)

        # try:
        #     lglf_link_state = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
        #     resp_lglf_link_state = lglf_link_state('l_gripper_l_finger', 'world')
        #     # lglf_reference = resp_lglf_link_state.link_state.reference_frame
        #     pose_lglf = resp_lglf_link_state.link_state.pose
        # except rospy.ServiceException, e:
        #     rospy.logerr("get_link_state for l_gripper_l_finger: {0}".format(e))
        # try:
        #     lgrf_link_state = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
        #     resp_lgrf_link_state = lgrf_link_state('l_gripper_r_finger', 'world')
        #     # lgrf_reference = resp_lgrf_link_state.link_state.reference_frame
        #     pose_lgrf = resp_lgrf_link_state.link_state.pose
        # except rospy.ServiceException, e:
        #     rospy.logerr("get_link_state for l_gripper_r_finger: {0}".format(e))

        # leftGripperPose = Pose()
        # leftGripperPose.position.x = (pose_lglf.position.x + pose_lgrf.position.x)/2
        # leftGripperPose.position.y = (pose_lglf.position.y + pose_lgrf.position.y)/2
        # leftGripperPose.position.z = (pose_lglf.position.z + pose_lgrf.position.z)/2
        # leftGripperPose.orientation = pose_lglf.orientation # TODO get the actual gripper orientation
        
        # poseStamped_left_gripper = PoseStamped(header=hdr, pose=leftGripperPose)
        # pub_left_gripper_pose.publish(blockPoseToGripper(poseStamped_left_gripper))

        # pose_rglf = None
        # pose_rgrf = None


        # try:
        #     lglf_link_state = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
        #     resp_rglf_link_state = lglf_link_state('r_gripper_l_finger', 'world')
        #     # lglf_reference = resp_lglf_link_state.link_state.reference_frame
        #     pose_rglf = resp_rglf_link_state.link_state.pose
        # except rospy.ServiceException, e:
        #     rospy.logerr("get_link_state for r_gripper_l_finger: {0}".format(e))
        # try:
        #     rgrf_link_state = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
        #     resp_rgrf_link_state = rgrf_link_state('r_gripper_r_finger', 'world')
        #     # lgrf_reference = resp_lgrf_link_state.link_state.reference_frame
        #     pose_rgrf = resp_rgrf_link_state.link_state.pose
        # except rospy.ServiceException, e:
        #     rospy.logerr("get_link_state for r_gripper_r_finger: {0}".format(e))

        # rightGripperPose = Pose()
        # rightGripperPose.position.x = (pose_rglf.position.x + pose_rgrf.position.x)/2
        # rightGripperPose.position.y = (pose_rglf.position.y + pose_rgrf.position.y)/2
        # rightGripperPose.position.z = (pose_rglf.position.z + pose_rgrf.position.z)/2
        # rightGripperPose.orientation = pose_rgrf.orientation # TODO get the actual gripper orientation
        
        # poseStamped_right_gripper = PoseStamped(header=hdr, pose=rightGripperPose)
        # pub_right_gripper_pose.publish(blockPoseToGripper(poseStamped_right_gripper))

###  END: Gripper stuff 
###################################################################################################

def main():
    rospy.init_node('initialize_environment_node', anonymous=True)
    # rospy.wait_for_message("/robot/sim/started", Empty) # causing the assimp error 

    rospy.on_shutdown(delete_gazebo_models)
    
    filename = '0'
    num_of_run = 1  

    move_to_start()
    init()

    # block_pose = Pose(position=Point(x=-0.2, y= 0, z=0.6))

    # for iBlock in range(0,12):

    #     print("Block " + str(iBlock))
    #     filename = str(iBlock)

    #     for run in range(0, num_of_run):
    #         if(not rospy.is_shutdown()):
    #             print("Picking up block")
    #             pick(block_pose, filename)
    #             placePose = addnoise_pose()
    #             print("Placing block")
    #             place(placePose, filename)
    #             gripper_open()
    #             move_to_start()


    #             #delete_gazebo_block()
    #             #load_gazebo_block(filename)
    #         else:
    #             break   
    rospy.spin()
    return 0

if __name__ == '__main__':
    sys.exit(main())