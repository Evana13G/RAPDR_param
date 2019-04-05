#!/usr/bin/env python
# Modified from RethinkRobotics website

import argparse
import struct
import sys
import copy
import rospy
import rospkg

from gazebo_msgs.srv import (
    SpawnModel,
    DeleteModel,
    GetModelState,
    GetLinkState,
)
from gazebo_msgs.msg import (
    LinkState,
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

from util.wall_controller import *
from util.physical_agent import PhysicalAgent
from environment.srv import * 
from util.goal_management import * 

from tf.transformations import *

pub = True

pub_cafe_table_pose = rospy.Publisher('cafe_table_pose', PoseStamped, queue_size = 10)
pub_left_gripper_pose = rospy.Publisher('left_gripper_pose', PoseStamped, queue_size = 10)
pub_right_gripper_pose = rospy.Publisher('right_gripper_pose', PoseStamped, queue_size = 10)
pub_cup_pose = rospy.Publisher('cup_pose', PoseStamped, queue_size = 10)
pub_cover_pose = rospy.Publisher('cover_pose', PoseStamped, queue_size = 10)

def load_gazebo_models(table_pose=Pose(position=Point(x=1, y=0.0, z=0.0)),
                       cup_pose=Pose(position=Point(x=0.9, y=0.0185, z=0.8)),
                       cover_pose=Pose(position=Point(x=0.9, y=0.0185, z=1.0)),
                       reference_frame="world"):

    model_path = rospkg.RosPack().get_path('environment')+"/models/"

    ############################################################################
    ########### Table
    table_xml = ''
    with open (model_path + "cafe_table/model.urdf", "r") as table_file:
        table_xml=table_file.read().replace('\n', '')    
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    try:
        spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        resp_sdf = spawn_sdf("cafe_table", table_xml, "/",
                             table_pose, reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn SDF service call failed: {0}".format(e))
    
    ############################################################################
    ###### Spawn URDFs


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



def delete_gazebo_models():
    try:
        delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        resp_delete = delete_model("cafe_table")
        resp_delete = delete_model("cup_with_cover")
    except rospy.ServiceException, e:
        rospy.loginfo("Delete Model service call failed: {0}".format(e))
        
def poseFromPoint(poseVar):
    newPose = poseVar.pose
    newPose.position.z -= 1.075
    q_orientation = quaternion_from_euler(3.14, 0, 0).tolist()
    newPose.orientation = Quaternion(q_orientation [0], q_orientation [1], q_orientation [2],q_orientation [3])
    newPoseStamped = PoseStamped(header = poseVar.header, pose = newPose)
    return newPoseStamped

def blockPoseToGripper(poseVar):
    newPose = poseVar.pose
    newPose.position.z -= 1.075
    oldOrientationQ = newPose.orientation
    q_orientation = quaternion_from_euler(3.14, 0, 0).tolist()
    newPose.orientation = Quaternion(q_orientation [0], q_orientation [1], q_orientation [2],q_orientation [3])
    newPoseStamped = PoseStamped(header = poseVar.header, pose = newPose)
    return newPoseStamped

def _moveLeftArmToStart():
    pa = PhysicalAgent('left_gripper')
    starting_joint_angles_l = {'left_w0': 0.6699952259595108,
                               'left_w1': 1.030009435085784,
                               'left_w2': -0.4999997247485215,
                               'left_e0': -1.189968899785275,
                               'left_e1': 1.9400238130755056,
                               'left_s0': -0.08000397926829805,
                               'left_s1': -0.9999781166910306}
    pa.move_to_start(starting_joint_angles_l)

def _moveRightArmToStart():
    pa = PhysicalAgent('right_gripper')
    starting_joint_angles_r = {'right_e0': -0.39888044530362166,
                                'right_e1': 1.9341522973651006,
                                'right_s0': 0.936293285623961,
                                'right_s1': -0.9939970420424453,
                                'right_w0': 0.27417171168213983,
                                'right_w1': 0.8298780975195674,
                                'right_w2': -0.5085333554167599}
    pa.move_to_start(starting_joint_angles_r)


def init():

    _moveLeftArmToStart()
    _moveRightArmToStart()
    rospy.sleep(3)
    load_gazebo_models()
    raiseWall()
    rospy.sleep(5)
    frameid_var = "/world"

    rospy.wait_for_service('/gazebo/get_model_state')
    rospy.wait_for_service('/gazebo/get_link_state')
    
    while pub == True:
        
        rate = rospy.Rate(10) # 10hz
        
        #Get cafe_table pose
        try:
            cafe_table_ms = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            resp_cafe_table_ms = cafe_table_ms("cafe_table", "");
            pose_cafe_table = resp_cafe_table_ms.pose
            header_cafe_table = resp_cafe_table_ms.header
            header_cafe_table.frame_id = frameid_var
            poseStamped_cafe_table = PoseStamped(header=header_cafe_table, pose=pose_cafe_table)
            pub_cafe_table_pose.publish(poseStamped_cafe_table)
        except rospy.ServiceException, e:
            rospy.logerr("get_model_state for cafe_table service call failed: {0}".format(e))

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

        pose_lglf = None
        pose_lgrf = None
        hdr = Header(frame_id=frameid_var)

        try:
            lglf_link_state = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
            resp_lglf_link_state = lglf_link_state('l_gripper_l_finger', 'world')
            # lglf_reference = resp_lglf_link_state.link_state.reference_frame
            pose_lglf = resp_lglf_link_state.link_state.pose
        except rospy.ServiceException, e:
            rospy.logerr("get_link_state for l_gripper_l_finger: {0}".format(e))
        try:
            lgrf_link_state = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
            resp_lgrf_link_state = lgrf_link_state('l_gripper_r_finger', 'world')
            # lgrf_reference = resp_lgrf_link_state.link_state.reference_frame
            pose_lgrf = resp_lgrf_link_state.link_state.pose
        except rospy.ServiceException, e:
            rospy.logerr("get_link_state for l_gripper_r_finger: {0}".format(e))

        leftGripperPose = Pose()
        leftGripperPose.position.x = (pose_lglf.position.x + pose_lgrf.position.x)/2
        leftGripperPose.position.y = (pose_lglf.position.y + pose_lgrf.position.y)/2
        leftGripperPose.position.z = (pose_lglf.position.z + pose_lgrf.position.z)/2
        leftGripperPose.orientation = pose_lglf.orientation # TODO get the actual gripper orientation
        
        poseStamped_left_gripper = PoseStamped(header=hdr, pose=leftGripperPose)
        pub_left_gripper_pose.publish(blockPoseToGripper(poseStamped_left_gripper))

        pose_rglf = None
        pose_rgrf = None


        try:
            lglf_link_state = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
            resp_rglf_link_state = lglf_link_state('r_gripper_l_finger', 'world')
            # lglf_reference = resp_lglf_link_state.link_state.reference_frame
            pose_rglf = resp_rglf_link_state.link_state.pose
        except rospy.ServiceException, e:
            rospy.logerr("get_link_state for r_gripper_l_finger: {0}".format(e))
        try:
            rgrf_link_state = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
            resp_rgrf_link_state = rgrf_link_state('r_gripper_r_finger', 'world')
            # lgrf_reference = resp_lgrf_link_state.link_state.reference_frame
            pose_rgrf = resp_rgrf_link_state.link_state.pose
        except rospy.ServiceException, e:
            rospy.logerr("get_link_state for r_gripper_r_finger: {0}".format(e))

        rightGripperPose = Pose()
        rightGripperPose.position.x = (pose_rglf.position.x + pose_rgrf.position.x)/2
        rightGripperPose.position.y = (pose_rglf.position.y + pose_rgrf.position.y)/2
        rightGripperPose.position.z = (pose_rglf.position.z + pose_rgrf.position.z)/2
        rightGripperPose.orientation = pose_rgrf.orientation # TODO get the actual gripper orientation
        
        poseStamped_right_gripper = PoseStamped(header=hdr, pose=rightGripperPose)
        pub_right_gripper_pose.publish(blockPoseToGripper(poseStamped_right_gripper))

###  END: Gripper stuff 
###################################################################################################

def handle_environment_request(req):
    #global pub 
    if req.action == "init":
        #pub = True
        try:
            _moveLeftArmToStart()
            _moveRightArmToStart()
            rospy.sleep(3)
            load_gazebo_models()
            rospy.sleep(1)
            raiseWall()
            rospy.sleep(1)
            return HandleEnvironmentSrvResponse(1)
        except rospy.ServiceException, e:
            rospy.logerr("Init environment call failed: {0}".format(e))
            return HandleEnvironmentSrvResponse(0)

    elif req.action == 'destroy':
        #pub = False
        try:
            delete_gazebo_models()
            return HandleEnvironmentSrvResponse(1)
        except rospy.ServiceException, e:
            rospy.logerr("Destroy environment call failed: {0}".format(e))
            return HandleEnvironmentSrvResponse(0)
    elif req.action == 'restart':
        try:
            ##pub = False
            _moveLeftArmToStart()
            _moveRightArmToStart()
            rospy.sleep(3)
            load_gazebo_models()
            rospy.sleep(1)
            raiseWall()
            rospy.sleep(1)
            return HandleEnvironmentSrvResponse(1)

        except rospy.ServiceException, e:
            rospy.logerr("Destroy environment call failed: {0}".format(e))
            return HandleEnvironmentSrvResponse(0)
    else:
        print('No Action')



def main():

    rospy.init_node("initialize_environment_node")
    rospy.wait_for_message("/robot/sim/started", Empty) 

    s = rospy.Service("init_environment", HandleEnvironmentSrv, handle_environment_request)
    init()
    rospy.spin()

    return 0

if __name__ == '__main__':
    sys.exit(main())
