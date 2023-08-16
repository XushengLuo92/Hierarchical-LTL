# Python 2/3 compatibility imports
import rospy 
from hierarchial_simulation.srv import robot_action,lego_pickup
import geometry_msgs.msg
import math
from tf.transformations import quaternion_from_euler
import gazebo_msgs.msg 
DE2RA = math.pi / 180
from gazebo_msgs.srv import SetModelState,GetLinkState,GetModelState
from gazebo_msgs.msg import ModelState,LinkState
# from human_fanuc_controller import human_left_hand_action_reletive2world,fanuc_action_reletive2world
import json
import _thread as thread 
import time
import rospy 
from hierarchial_simulation.srv import robot_action,lego_pickup
import geometry_msgs.msg
import math
from tf.transformations import quaternion_from_euler,euler_from_quaternion
from gazebo_msgs.msg import ModelState,LinkState
from gazebo_msgs.srv import SetModelState,GetLinkState,GetModelState
import std_msgs
import _thread
DE2RA = math.pi / 180
from scipy.spatial.transform import Rotation
import numpy as np


def get_pose2base(pose2world:geometry_msgs.msg.Pose,base2world:geometry_msgs.msg.Pose):
    Rot_pose2world=Rotation.from_quat((pose2world.orientation.x,pose2world.orientation.y,pose2world.orientation.z,pose2world.orientation.w))

    Rot_base2world=Rotation.from_quat((base2world.orientation.x,base2world.orientation.y,base2world.orientation.z,pose2world.orientation.w))
    # Rotation.
    Rot=Rot_base2world.as_matrix().T*Rot_pose2world.as_matrix()
    p=Rot_base2world.as_matrix().T*(np.matrix([
        [pose2world.position.x-base2world.position.x],
        [pose2world.position.y-base2world.position.y],
        [pose2world.position.z-base2world.position.z],
    ]))
    # print (Rot.as_euler('XYZ',degrees=True),'\n',p)
    return (Rotation.from_matrix(Rot).as_quat(),p)
    pass 


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
def fanuc_action_reletive2world(x=0.1,y=-0.2,z=0.15,orientation=40,pick=True,brick_name=None):
    print('-----------------------')
    currentpose=getLink2world(name='fanuc_gazebo::base')
    print(Rotation.from_quat([currentpose.orientation.x,currentpose.orientation.y,currentpose.orientation.z,currentpose.orientation.w]).as_euler('XYZ'),'\n',currentpose.position)


    pose_goal = geometry_msgs.msg.PoseStamped()
    # pose_goal.header.frame_id = "world"
    # orientation=0
    # relative to the world config
    q = Rotation.from_euler('xyz',[orientation * DE2RA, 90 * DE2RA, 0 * DE2RA]).as_quat()
    pose_goal.pose.orientation.x = q[0]
    pose_goal.pose.orientation.y = q[1]
    pose_goal.pose.orientation.z = q[2]
    pose_goal.pose.orientation.w = q[3]
    pose_goal.pose.position.x = x-currentpose.position.x
    pose_goal.pose.position.y = y-currentpose.position.y
    pose_goal.pose.position.z = z-currentpose.position.z
    print(pose_goal.pose.position,'........................')
    rospy.wait_for_service('/fanuc_gazebo/action_msg')
    try:
        action_msg = rospy.ServiceProxy('/fanuc_gazebo/action_msg', robot_action)
        
        # pose_goal = geometry_msgs.msg.PoseStamped()
        pose_goal.header.frame_id = "world"

        res = action_msg(pose_goal, pose_goal)

        tippose=getLink2world(name='fanuc_gazebo::link_tool')
        print(tippose.position.x -currentpose.position.x,
        tippose.position.y -currentpose.position.y,
        tippose.position.z -currentpose.position.z)
        return res.finished
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
def fanuc1_action_reletive2world(x=0.1,y=-0.2,z=0.15,orientation=40,pick=True,brick_name=None):
    print('-----------------------')
    currentpose=getLink2world(name='fanuc1_gazebo::base')
    print(Rotation.from_quat([currentpose.orientation.x,currentpose.orientation.y,currentpose.orientation.z,currentpose.orientation.w]).as_euler('XYZ'),'\n',currentpose.position)


    pose_goal = geometry_msgs.msg.PoseStamped()
    # pose_goal.header.frame_id = "world"
    # orientation=0
    # relative to the world config
    q = Rotation.from_euler('xyz',[orientation * DE2RA, 90 * DE2RA, 0 * DE2RA]).as_quat()
    pose_goal.pose.orientation.x = q[0]
    pose_goal.pose.orientation.y = q[1]
    pose_goal.pose.orientation.z = q[2]
    pose_goal.pose.orientation.w = q[3]
    pose_goal.pose.position.x = x-currentpose.position.x
    pose_goal.pose.position.y = y-currentpose.position.y
    pose_goal.pose.position.z = z-currentpose.position.z
    print(pose_goal.pose.position,'........................')
    rospy.wait_for_service('/fanuc1_gazebo/action_msg')
    try:
        action_msg = rospy.ServiceProxy('/fanuc1_gazebo/action_msg', robot_action)
        
        # pose_goal = geometry_msgs.msg.PoseStamped()
        pose_goal.header.frame_id = "world"

        res = action_msg(pose_goal, pose_goal)

        tippose=getLink2world(name='fanuc1_gazebo::link_tool')
        print(tippose.position.x -currentpose.position.x,
        tippose.position.y -currentpose.position.y,
        tippose.position.z -currentpose.position.z)
        return res.finished
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
def human_left_hand_action_reletive2world(x=-0,y=-0.1,z=0.30,orientation=90,pick=True,brick_name=None):
    # if brick_name != None:
    #     if pick:
    #         legeo_action_msg = rospy.ServiceProxy('/lego_gazebo/action_msg', lego_pickup)
    #         res = legeo_action_msg("human_gazebo::LeftHand","pick",brick_name)
    #     else:
    #         legeo_action_msg = rospy.ServiceProxy('/lego_gazebo/action_msg', lego_pickup)
    #         res = legeo_action_msg("human_gazebo::LeftHand","unpick",brick_name)


    # currentpose=getLink2world(name='world')
    currentpose=getLink2world(name="human_gazebo::LeftHand")
    print(Rotation.from_quat([currentpose.orientation.x,currentpose.orientation.y,currentpose.orientation.z,currentpose.orientation.w]).as_euler('XYZ'),'\n',currentpose.position)
    print('-----------------------')
    currentpose=getLink2world(name="human_gazebo::Pelvis")
    # print(Rotation.from_quat([currentpose.orientation.x,currentpose.orientation.y,currentpose.orientation.z,currentpose.orientation.w]).as_euler('XYZ'),'\n',currentpose.position)
    # currentpose=getLink2world(name='world_fixed')
    pose_goal = geometry_msgs.msg.PoseStamped()
    # pose_goal.header.frame_id = "world"
    # orientation=0
    q = Rotation.from_euler('yxz',[-orientation * DE2RA, -90 * DE2RA, 0 * DE2RA]).as_quat()
    # q = quaternion_from_euler(-90 * DE2RA, 0 * DE2RA, 0 * DE2RA)
    pose_goal.pose.orientation.x = q[0]
    pose_goal.pose.orientation.y = q[1]
    pose_goal.pose.orientation.z = q[2]
    pose_goal.pose.orientation.w = q[3]
    pose_goal.pose.position.x = currentpose.position.x-x
    pose_goal.pose.position.y = currentpose.position.y-y
    pose_goal.pose.position.z = z-currentpose.position.z
    print(pose_goal.pose.position,'\n........................')
    # pose2base=get_pose2base(pose_goal.pose,currentpose)
    # print(pose2base)

    rospy.wait_for_service('/human_gazebo/action_msg/left_arm')
    try:
        action_msg = rospy.ServiceProxy('/human_gazebo/action_msg/left_arm', robot_action)
        
        # pose_goal = geometry_msgs.msg.PoseStamped()
        pose_goal.header.frame_id = "world"
        # # orientation=0
        # # q = quaternion_from_euler(orientation * DE2RA, 90 * DE2RA, 0 * DE2RA)
        # pose_goal.pose.orientation.x = pose2base[0][0]
        # pose_goal.pose.orientation.y = pose2base[0][1]
        # pose_goal.pose.orientation.z = pose2base[0][2]
        # pose_goal.pose.orientation.w = pose2base[0][3]
        # pose_goal.pose.position.x = pose2base[1][0]
        # pose_goal.pose.position.y = pose2base[1][1]
        # pose_goal.pose.position.z = pose2base[1][2]

        res = action_msg(pose_goal, pose_goal)
        # tippose=getLink2world(name='human_gazebo::LeftHand')
        # # print(Rotation.from_quat([tippose.orientation.x,tippose.orientation.y,tippose.orientation.z,tippose.orientation.w]).as_euler('XYZ'),'\n',tippose.position)
        # print(tippose.position.x -currentpose.position.x,
        # tippose.position.y -currentpose.position.y,
        # tippose.position.z -currentpose.position.z)
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

def test():
    time.sleep(5)
    # action_msg = rospy.ServiceProxy('/hltl_msg/human_action', lego_pickup)
    action_msg = rospy.ServiceProxy('/hltl_msg/fanuc1_action', lego_pickup)
    action_msg("23", "pick", "b15_1")
    action_msg("23", "pick", "b15_2")
    action_msg("23", "pick", "b13_1")
    action_msg("23", "pick", "b15_3")
    action_msg("23", "pick", "b15_4")
    action_msg("23", "pick", "b13_2")
    action_msg("23", "pick", "b13_3")
    action_msg("23", "pick", "b15_5")
    action_msg("23", "pick", "b11_2")
    action_msg("23", "pick", "b11_1")
    action_msg("23", "pick", "b13_4")
    pass 


class lego_state():
    def __init__(self,config_json=None) -> None:
        if config_json!=None:
            with open(config_json,'r') as config_file:
                self.config_data=json.load(config_file)
                self.brick_len=len(self.config_data["config"])
                self.name2id={}
                for id in range(self.brick_len):
                    self.name2id[self.config_data['config'][id]["brick_name"]]=id
                # print(self.config_data["config"].index("b15_1"))
                # self.pick=["init" for i in range(self.brick_len)]
                # print(self.config_data["config"][0])
        print(self.config_data['config'][0]["brick_name"])
        # self.init_state()
        # s = rospy.Service('action_msg/lego_state', lego_pickup, self.lego_state_service_handler)
        print("lego python service ready")
        rospy.init_node("hltl_python_interface", anonymous=True)
        # thread.start_new_thread(self.refresh_lego_state,())
        # thread.start_new_thread(self.human_HLTL_service_launch,())
        thread.start_new_thread(self.fanuc_HLTL_service_launch,())
        thread.start_new_thread(self.fanuc1_HLTL_service_launch,())
        thread.start_new_thread(test,())
        self.refresh_lego_state()
        # rospy.spin()
    def init_state(self):
        for id in range(self.brick_len):
            if self.config_data['config'][id]["state"]=="init":
                lego_action(self.config_data['config'][id]["brick_name"],
                            x=self.config_data['config'][id]["des"]["x"],
                            y=self.config_data['config'][id]["des"]["y"],
                            z=self.config_data['config'][id]["des"]["z"],orientation=self.config_data['config'][id]["des"]["o"])
                self.config_data['config'][id]["state"]="src"
    # def refresh_state(self):
    #     pass 
    def change_state(self,brick_name,reference_frame):
        self.config_data['config'][self.name2id[brick_name]]["state"]=reference_frame
        pass 

    def lego_state_service_handler(self,req:lego_pickup):

        # pose_goal = geometry_msgs.msg.PoseStamped()
        # pose_goal.header.frame_id = "world"
        # pose_goal.pose.orientation.x = 1e-6
        # pose_goal.pose.orientation.y = 1e-6
        # pose_goal.pose.orientation.z = 1e-6
        # pose_goal.pose.orientation.w = -1.000000
        # pose_goal.pose.position.x = 0.5
        # pose_goal.pose.position.y = 0
        # pose_goal.pose.position.z = 0.16+0.015

        if req.is_pick=="pick":
            self.change_state(req.pick_lego_name,req.reference_frame)
        else:
            self.change_state(req.pick_lego_name,"des")
        return True

    def refresh_lego_state(self):
        rate = rospy.Rate(30) 
        while not rospy.is_shutdown():
            for id in range(self.brick_len):
                if self.config_data['config'][id]["state"] =='fanuc_gazebo::link_tool':
                    lego_action(brick_name=self.config_data['config'][id]["brick_name"] ,reference_frame='fanuc_gazebo::link_tool',x=0,y=0,z=0.075,orientation=180)
                elif self.config_data['config'][id]["state"] =='human_gazebo::LeftHand':
                    lego_action(brick_name=self.config_data['config'][id]["brick_name"],reference_frame='human_gazebo::LeftHand',y=0.175,orientation=-90)
            rate.sleep()
        pass 
    def human_HLTL_service_launch(self):
        def human_HLTL_service(req:lego_pickup):
            id=self.name2id[req.pick_lego_name]
            # human_left_hand_action_reletive2world(x= self.config_data['config'][id]["src"]["x"],y=self.config_data['config'][id]["src"]["y"],z=0.3,orientation=self.config_data['config'][id]["src"]["o"]+90)
            # move to the above of the brick
            human_left_hand_action_reletive2world(x= self.config_data['config'][id]["src"]["x"],y=self.config_data['config'][id]["src"]["y"],z=0.18,orientation=self.config_data['config'][id]["src"]["o"])
            # move to the top of the brick
            self.change_state(req.pick_lego_name,'human_gazebo::LeftHand')
            human_left_hand_action_reletive2world(x=(self.config_data['config'][id]["src"]["x"]+self.config_data['config'][id]["des"]["x"])/2,y=(self.config_data['config'][id]["src"]["y"]+self.config_data['config'][id]["des"]["y"])/2,z=0.3,orientation=self.config_data['config'][id]["src"]["o"])
            # move to the above of the brick
            # human_left_hand_action_reletive2world(x= self.config_data['config'][id]["des"]["x"],y=self.config_data['config'][id]["des"]["y"],z=0.3,orientation=self.config_data['config'][id]["des"]["o"])
            # move to the above of the brick des
            human_left_hand_action_reletive2world(x= self.config_data['config'][id]["des"]["x"],y=self.config_data['config'][id]["des"]["y"],z=0.18,orientation=self.config_data['config'][id]["des"]["o"])
            self.change_state(req.pick_lego_name,'des')
            # move to the top of the brick des
            return True
            pass 
        s = rospy.Service('hltl_msg/human_action', lego_pickup, human_HLTL_service)
        print("hltl_msg/human_action python service ready")
        rospy.spin()
        pass 
    def fanuc_HLTL_service_launch(self):
        def fanuc_HLTL_service(req:lego_pickup):
            id=self.name2id[req.pick_lego_name]
            fanuc_action_reletive2world(x= self.config_data['config'][id]["src"]["x"],y=self.config_data['config'][id]["src"]["y"],z=0.2,orientation=self.config_data['config'][id]["src"]["o"])
            # move to the above of the brick
            fanuc_action_reletive2world(x= self.config_data['config'][id]["src"]["x"],y=self.config_data['config'][id]["src"]["y"],z=0.12,orientation=self.config_data['config'][id]["src"]["o"])
            # move to the top of the brick
            self.change_state(req.pick_lego_name,'fanuc_gazebo::link_tool')
            fanuc_action_reletive2world(x= self.config_data['config'][id]["src"]["x"],y=self.config_data['config'][id]["src"]["y"],z=0.2,orientation=self.config_data['config'][id]["src"]["o"])
            # move to the above of the brick
            fanuc_action_reletive2world(x= self.config_data['config'][id]["des"]["x"],y=self.config_data['config'][id]["des"]["y"],z=0.2,orientation=self.config_data['config'][id]["des"]["o"])
            # move to the above of the brick des
            fanuc_action_reletive2world(x= self.config_data['config'][id]["des"]["x"],y=self.config_data['config'][id]["des"]["y"],z=0.12,orientation=self.config_data['config'][id]["des"]["o"])
            self.change_state(req.pick_lego_name,'des')
            # move to the top of the brick des
            return True
            pass 
        s = rospy.Service('hltl_msg/fanuc_action', lego_pickup, fanuc_HLTL_service)
        print("hltl_msg/fanuc_action python service ready")
        rospy.spin()
        pass 
    def fanuc1_HLTL_service_launch(self):
        def fanuc_HLTL_service(req:lego_pickup):
            id=self.name2id[req.pick_lego_name]
            fanuc1_action_reletive2world(x= self.config_data['config'][id]["src"]["x"],y=self.config_data['config'][id]["src"]["y"],z=0.2,orientation=self.config_data['config'][id]["src"]["o"])
            # move to the above of the brick
            fanuc1_action_reletive2world(x= self.config_data['config'][id]["src"]["x"],y=self.config_data['config'][id]["src"]["y"],z=0.12,orientation=self.config_data['config'][id]["src"]["o"])
            # move to the top of the brick
            self.change_state(req.pick_lego_name,'fanuc1_gazebo::link_tool')
            fanuc1_action_reletive2world(x= self.config_data['config'][id]["src"]["x"],y=self.config_data['config'][id]["src"]["y"],z=0.2,orientation=self.config_data['config'][id]["src"]["o"])
            # move to the above of the brick
            fanuc1_action_reletive2world(x= self.config_data['config'][id]["des"]["x"],y=self.config_data['config'][id]["des"]["y"],z=0.2,orientation=self.config_data['config'][id]["des"]["o"])
            # move to the above of the brick des
            fanuc1_action_reletive2world(x= self.config_data['config'][id]["des"]["x"],y=self.config_data['config'][id]["des"]["y"],z=0.12,orientation=self.config_data['config'][id]["des"]["o"])
            self.change_state(req.pick_lego_name,'des')
            # move to the top of the brick des
            return True
            pass 
        s = rospy.Service('hltl_msg/fanuc1_action', lego_pickup, fanuc_HLTL_service)
        print("hltl_msg/fanuc1_action python service ready")
        rospy.spin()
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



if __name__=='__main__':
    lego_state(config_json=rospy.get_param("/lego_state_refresher/config_json"))


    # /lego_state_refresher/config_json
    # lego=lego_state(config_json="/home/xusj/Documents/0LAB/23ChangliuLiu/Hierarchical-LTL/hierarchial_simulation/src/hierarchial_simulation/config/ICL.json")