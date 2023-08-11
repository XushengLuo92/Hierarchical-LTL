import rospy 
from stmotion_controller.srv import robot_action
import geometry_msgs.msg
import math
from tf.transformations import quaternion_from_euler
from gazebo_msgs.msg import ModelState,LinkState
from gazebo_msgs.srv import SetModelState,GetLinkState,GetModelState
import _thread
DE2RA = math.pi / 180
from scipy.spatial.transform import Rotation
import numpy as np
def get_pose2base(pose2world:geometry_msgs.msg.Pose,base2world:geometry_msgs.msg.Pose):
    Rot_pose2world=Rotation.from_quat((pose2world.orientation.x,pose2world.orientation.y,pose2world.orientation.z,pose2world.orientation.w))

    Rot_base2world=Rotation.from_quat((base2world.orientation.x,base2world.orientation.y,base2world.orientation.z,pose2world.orientation.w))
    # Rotation.
    Rot=Rot_base2world.inv()*Rot_pose2world 
    p=Rot_base2world.inv().as_matrix()*(np.matrix([
        [pose2world.position.x-base2world.position.x],
        [pose2world.position.y-base2world.position.y],
        [pose2world.position.z-base2world.position.z],
    ]))
    # return (Rot.as_euler('zyx',degrees=True),p)
    return (Rot.as_quat(),p)
    pass 
def lego_action(brick_name='b14_9',reference_frame='world',x=0,y=0,z=0,orientation=90):
    # rospy.init_node('set_pose')

    state_msg = ModelState()
    state_msg.model_name = brick_name
    state_msg.reference_frame = reference_frame
    state_msg.pose.position.x = x
    state_msg.pose.position.y = y
    state_msg.pose.position.z = z
    # orientation=90
    q = quaternion_from_euler(0 * DE2RA, 0 * DE2RA, orientation * DE2RA)
    state_msg.pose.orientation.w = q[0]
    state_msg.pose.orientation.x = q[1]
    state_msg.pose.orientation.y = q[2]
    state_msg.pose.orientation.z = q[3]

    rospy.wait_for_service('/gazebo/set_model_state')
    try:
        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        resp = set_state( state_msg )
        return resp
    except rospy.ServiceException as e:
        print( "Service call failed: %s" % e)

def fanuc_action(x=0,y=1,z=1,orientation=0):
    rospy.wait_for_service('/fanuc_gazebo/action_msg')
    try:
        action_msg = rospy.ServiceProxy('/fanuc_gazebo/action_msg', robot_action)
        
        pose_goal = geometry_msgs.msg.PoseStamped()
        pose_goal.header.frame_id = "world"
        orientation=0
        q = quaternion_from_euler(orientation * DE2RA, 90 * DE2RA, 0 * DE2RA)
        pose_goal.pose.orientation.w = q[0]
        pose_goal.pose.orientation.x = q[1]
        pose_goal.pose.orientation.y = q[2]
        pose_goal.pose.orientation.z = q[3]
        # pose_goal.pose.orientation.x = 
        # pose_goal.pose.orientation.y = 1e-6
        # pose_goal.pose.orientation.z = 1e-6
        # pose_goal.pose.orientation.w = 1.000000
        pose_goal.pose.position.x = x
        pose_goal.pose.position.y = y
        pose_goal.pose.position.z = z

        res = action_msg(pose_goal, pose_goal)
        return res.finished
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


def human_right_action():
    rospy.wait_for_service('/human_gazebo/action_msg/right_arm')
    try:
        action_msg = rospy.ServiceProxy('/human_gazebo/action_msg/right_arm', robot_action)
        
        pose_goal = geometry_msgs.msg.PoseStamped()
        pose_goal.header.frame_id = "world"
        orientation=0
        q = quaternion_from_euler(orientation * DE2RA, 90 * DE2RA, 0 * DE2RA)
        pose_goal.pose.orientation.w = q[0]
        pose_goal.pose.orientation.x = q[1]
        pose_goal.pose.orientation.y = q[2]
        pose_goal.pose.orientation.z = q[3]
        # pose_goal.pose.orientation.x = 
        # pose_goal.pose.orientation.y = 1e-6
        # pose_goal.pose.orientation.z = 1e-6
        # pose_goal.pose.orientation.w = 1.000000
        pose_goal.pose.position.x = 0
        pose_goal.pose.position.y = 0.3
        pose_goal.pose.position.z = 0.16+0.015

        res = action_msg(pose_goal, pose_goal)
        return res.finished
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


def human_left_action(ox=0,oy=0,oz=0,ow=0,px=0,py=0,pz=1):
    rospy.wait_for_service('/human_gazebo/action_msg/left_arm')
    try:
        action_msg = rospy.ServiceProxy('/human_gazebo/action_msg/left_arm', robot_action)
        
        pose_goal = geometry_msgs.msg.PoseStamped()
        pose_goal.header.frame_id = "world"
        pose_goal.pose.orientation.x = ox
        pose_goal.pose.orientation.y = oy 
        pose_goal.pose.orientation.z = oz 
        pose_goal.pose.orientation.w = ow
        pose_goal.pose.position.x = px 
        pose_goal.pose.position.y = py 
        pose_goal.pose.position.z = pz

        res = action_msg(pose_goal, pose_goal)
        return res.finished
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def get_fanuc_gripper_pose2world():
    rospy.wait_for_service('/gazebo/get_link_state')
    try:
        action_msg = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
        
        # pose_goal = GetLinkState()
        link_name = "fanuc_gazebo::link_tool"
        reference_frame = 'world'

        res = action_msg(link_name,reference_frame)
        return res
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
def get_human_lefthand_pose2world():
    rospy.wait_for_service('/gazebo/get_link_state')
    try:
        action_msg = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
        
        # pose_goal = GetLinkState()
        link_name = "human_gazebo::LeftHand"
        # reference_frame = 'world'
        reference_frame ='Pelvis'
        res = action_msg(link_name,reference_frame)
        return res
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
def getLink2world(name="fanuc_gazebo::base"):
    rospy.wait_for_service('/gazebo/get_link_state')
    try:
        action_msg = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
        
        # pose_goal = GetLinkState()
        link_name = name
        reference_frame = 'world'
        res = action_msg(link_name,reference_frame)
        return res.link_state.pose
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
def fanuc_action_reletive2world(x=3,y=0.3,z=0.1,orientation=0):
    currentpose=getLink2world(name='fanuc_gazebo::base')

    pose_goal = geometry_msgs.msg.Pose()
    # pose_goal.header.frame_id = "world"
    # orientation=0
    q = quaternion_from_euler(orientation * DE2RA, 90 * DE2RA, 0 * DE2RA)
    pose_goal.orientation.x = q[0]
    pose_goal.orientation.y = q[1]
    pose_goal.orientation.z = q[2]
    pose_goal.orientation.w = q[3]
    pose_goal.position.x = x
    pose_goal.position.y = y
    pose_goal.position.z = z

    pose2base=get_pose2base(pose_goal,currentpose)
    print(pose2base)
    # res=currentpose
    # fanuc_action()
    rospy.wait_for_service('/fanuc_gazebo/action_msg')
    try:
        action_msg = rospy.ServiceProxy('/fanuc_gazebo/action_msg', robot_action)
        
        pose_goal = geometry_msgs.msg.PoseStamped()
        pose_goal.header.frame_id = "world"
        # orientation=0
        # q = quaternion_from_euler(orientation * DE2RA, 90 * DE2RA, 0 * DE2RA)
        pose_goal.pose.orientation.x = pose2base[0][0]
        pose_goal.pose.orientation.y = pose2base[0][1]
        pose_goal.pose.orientation.z = pose2base[0][2]
        pose_goal.pose.orientation.w = pose2base[0][3]
        pose_goal.pose.position.x = pose2base[1][0]
        pose_goal.pose.position.y = pose2base[1][1]
        pose_goal.pose.position.z = pose2base[1][2]

        res = action_msg(pose_goal, pose_goal)
        return res.finished
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
def human_left_hand_action_reletive2world(x=0,y=0,z=0.17,orientation=-45,pick=True,brink_name=None):
    if pick and brink_name != None:
        lego_action(brick_name=brink_name,reference_frame='human_gazebo::LeftHand',y=0.17,orientation=-90)
    currentpose=getLink2world(name='human_gazebo::Pelvis')

    pose_goal = geometry_msgs.msg.Pose()
    # pose_goal.header.frame_id = "world"
    # orientation=0
    q = quaternion_from_euler(48 * DE2RA, -11 * DE2RA, -158 * DE2RA)
    pose_goal.orientation.w = q[0]
    pose_goal.orientation.x = q[1]
    pose_goal.orientation.y = q[2]
    pose_goal.orientation.z = q[3]
    pose_goal.position.x = x
    pose_goal.position.y = y
    pose_goal.position.z = z

    pose2base=get_pose2base(pose_goal,currentpose)
    print(pose2base)
    # res=currentpose
    # fanuc_action()
    rospy.wait_for_service('/human_gazebo/action_msg/left_arm')
    try:
        action_msg = rospy.ServiceProxy('/human_gazebo/action_msg/left_arm', robot_action)
        
        pose_goal = geometry_msgs.msg.PoseStamped()
        pose_goal.header.frame_id = "world"
        # orientation=0
        # q = quaternion_from_euler(orientation * DE2RA, 90 * DE2RA, 0 * DE2RA)
        pose_goal.pose.orientation.x = pose2base[0][0]
        pose_goal.pose.orientation.y = pose2base[0][1]
        pose_goal.pose.orientation.z = pose2base[0][2]
        pose_goal.pose.orientation.w = pose2base[0][3]
        pose_goal.pose.position.x = pose2base[1][0]
        pose_goal.pose.position.y = pose2base[1][1]
        pose_goal.pose.position.z = pose2base[1][2]

        res = action_msg(pose_goal, pose_goal)
        return res.finished
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

        # position: 
        # x: -0.08890735728892357
        # y: 0.028897506508673176
        # z: 0.19514962874265734
        # orientation: 
        # x: -0.2131141193741749
        # y: -0.7107201635843112
        # z: 0.6184903264237472
        # w: 0.2587062761478927




if __name__=='__main__':
    # print(lego_action())
    # print(fanuc_action_reletive2world())
    print(getLink2world(name='human_gazebo::LeftHand'))
    # print(human_left_hand_action_reletive2world(brink_name='b14_9'))
    print(fanuc_action_reletive2world(x=0,y=0.3,z=0.3,orientation=0))
#     print(human_left_action(      
#         ox=-0.027391944045042717,
#         oy=-0.04646070418412789,
#         oz=-0.004298856136034362,
#         ow=0.9985352293254705,
#         px=-0.022892628281627705,
#         py=0.6578494544826015,
#         pz=0.39188342533147774
# ))
    # print(get_fanuc_gripper_pose2world())
#     print(get_human_lefthand_pose2world())
    