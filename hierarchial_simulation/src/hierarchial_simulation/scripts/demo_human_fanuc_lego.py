# Python 2/3 compatibility imports
import rospy 
from hierarchial_simulation.srv import robot_action,lego_pickup
import geometry_msgs.msg
import math
import gazebo_msgs.msg 
DE2RA = math.pi / 180
import json
import _thread as thread 
import time
import rospy 
from tf.transformations import quaternion_from_euler,euler_from_quaternion
from gazebo_msgs.msg import ModelState,LinkState
from gazebo_msgs.srv import SetModelState,GetLinkState,GetModelState
import std_msgs
import _thread
DE2RA = math.pi / 180
from scipy.spatial.transform import Rotation
import numpy as np
import sys
sys.path.append("/home/xusj/Documents/0LAB/23ChangliuLiu/Hierarchical-LTL/")
from hierarchical_LTL import hierarchical_ltl_planner

import argparse
import rospy
def create_parser():
    """ create parser

    Returns:
        _type_: _description_
    """
    parser = argparse.ArgumentParser(description='FM')
    parser.add_argument('--task', default="man", type=str)
    parser.add_argument('--case', default=6, type=int)
    parser.add_argument('--vis', action='store_true', help='Enable visualization')
    parser.add_argument('--dot', action='store_true', help='Enable dot graph')

    return parser





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
    # print('-----------------------')
    currentpose=getLink2world(name='fanuc_gazebo::base')
    # print(Rotation.from_quat([currentpose.orientation.x,currentpose.orientation.y,currentpose.orientation.z,currentpose.orientation.w]).as_euler('XYZ'),'\n',currentpose.position)


    pose_goal = geometry_msgs.msg.PoseStamped()
    q = Rotation.from_euler('xyz',[orientation * DE2RA, 90 * DE2RA, 0 * DE2RA]).as_quat()
    pose_goal.pose.orientation.x = q[0]
    pose_goal.pose.orientation.y = q[1]
    pose_goal.pose.orientation.z = q[2]
    pose_goal.pose.orientation.w = q[3]
    pose_goal.pose.position.x = x-currentpose.position.x
    pose_goal.pose.position.y = y-currentpose.position.y
    pose_goal.pose.position.z = z-currentpose.position.z
    # print(pose_goal.pose.position,'........................')
    rospy.wait_for_service('/fanuc_gazebo/action_msg')
    try:
        action_msg = rospy.ServiceProxy('/fanuc_gazebo/action_msg', robot_action)
        
        # pose_goal = geometry_msgs.msg.PoseStamped()
        pose_goal.header.frame_id = "world"

        res = action_msg(pose_goal, pose_goal)

        tippose=getLink2world(name='fanuc_gazebo::link_tool')
        # print(tippose.position.x -currentpose.position.x,
        # tippose.position.y -currentpose.position.y,
        # tippose.position.z -currentpose.position.z)
        return res.finished
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
def fanuc1_action_reletive2world(x=0.1,y=-0.2,z=0.15,orientation=40,pick=True,brick_name=None):
    # print('-----------------------')
    currentpose=getLink2world(name='fanuc1_gazebo::base')
    # print(Rotation.from_quat([currentpose.orientation.x,currentpose.orientation.y,currentpose.orientation.z,currentpose.orientation.w]).as_euler('XYZ'),'\n',currentpose.position)


    pose_goal = geometry_msgs.msg.PoseStamped()
    q = Rotation.from_euler('xyz',[orientation * DE2RA, 90 * DE2RA, 0 * DE2RA]).as_quat()
    pose_goal.pose.orientation.x = q[0]
    pose_goal.pose.orientation.y = q[1]
    pose_goal.pose.orientation.z = q[2]
    pose_goal.pose.orientation.w = q[3]
    pose_goal.pose.position.x = currentpose.position.x-x
    pose_goal.pose.position.y = currentpose.position.y-y
    pose_goal.pose.position.z = z-currentpose.position.z
    # print(pose_goal.pose.position,'........................')
    rospy.wait_for_service('/fanuc1_gazebo/action_msg')
    try:
        action_msg = rospy.ServiceProxy('/fanuc1_gazebo/action_msg', robot_action)
        
        # pose_goal = geometry_msgs.msg.PoseStamped()
        pose_goal.header.frame_id = "world"

        res = action_msg(pose_goal, pose_goal)

        tippose=getLink2world(name='fanuc1_gazebo::link_tool')
        # print(tippose.position.x -currentpose.position.x,
        # tippose.position.y -currentpose.position.y,
        # tippose.position.z -currentpose.position.z)
        return res.finished
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
def human_left_hand_action_reletive2world(x=-0,y=-0.1,z=0.30,orientation=90,pick=True,brick_name=None):
    currentpose=getLink2world(name="human_gazebo::LeftHand")
    print(Rotation.from_quat([currentpose.orientation.x,currentpose.orientation.y,currentpose.orientation.z,currentpose.orientation.w]).as_euler('XYZ'),'\n',currentpose.position)
    print('-----------------------')
    currentpose=getLink2world(name="human_gazebo::Pelvis")
    pose_goal = geometry_msgs.msg.PoseStamped()
    q = Rotation.from_euler('yxz',[-orientation * DE2RA, -90 * DE2RA, 0 * DE2RA]).as_quat()
    pose_goal.pose.orientation.x = q[0]
    pose_goal.pose.orientation.y = q[1]
    pose_goal.pose.orientation.z = q[2]
    pose_goal.pose.orientation.w = q[3]
    pose_goal.pose.position.x = currentpose.position.x-x
    pose_goal.pose.position.y = currentpose.position.y-y
    pose_goal.pose.position.z = z-currentpose.position.z
    print(pose_goal.pose.position,'\n........................')

    rospy.wait_for_service('/human_gazebo/action_msg/left_arm')
    try:
        action_msg = rospy.ServiceProxy('/human_gazebo/action_msg/left_arm', robot_action)
        pose_goal.header.frame_id = "world"
        res = action_msg(pose_goal, pose_goal)
        return res.finished
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


def send2fanuc(robotname='/hltl_msg/fanuc_action',bringkname='b15_1'):
    action_msg = rospy.ServiceProxy(robotname, lego_pickup)
    action_msg("23", "pick", bringkname)
    return



class HLTL_sim():
    def __init__(self,config_json=None) -> None:
        self.global_task_finished_id=-1
        self.num_operators=2
        self.robot_id=['fanuc','fanuc1','human']

        self.invalid_str = -1
        self.finished_task_str = np.zeros(self.num_operators)

        if config_json!=None:
            with open(config_json,'r') as config_file:
                self.config_data=json.load(config_file)
                self.brick_len=len(self.config_data["config"])
                self.name2id={}
                self.task_id2brick_id={}
                for id in range(self.brick_len):
                    self.name2id[self.config_data['config'][id]["brick_name"]]=id
                    self.task_id2brick_id[self.config_data['config'][id]["task_id"]+self.config_data['config'][id]["belong"]]=id
                # print(self.task_id2brick_id)
        self.someBodyPlacing=np.zeros(self.num_operators)
        print(self.config_data['config'][0]["brick_name"])
        print("lego python service ready")
        rospy.init_node("hltl_python_interface", anonymous=True)
        self.init_state(pose="src")
        thread.start_new_thread(self.fanuc_HLTL_service_launch,())
        thread.start_new_thread(self.fanuc1_HLTL_service_launch,())
        thread.start_new_thread(self.test,())
        self.refresh_lego_state()

        # rospy.spin()

    def task_execution(self,time_task_element_type_robot_axis, reduced_task_network, type_robots):
        finished_tasks = []
        current_exec_tasks = []
        current_exec_robots = []
        candidate_tasks = set()
        
        task_element2type_robot = {time_task_element_type_robot[1] : time_task_element_type_robot[2] \
            for time_task_element_type_robot in time_task_element_type_robot_axis if len(time_task_element_type_robot) == 3}
        task_element2time =  {time_task_element_type_robot[1] : time_task_element_type_robot[0] \
            for time_task_element_type_robot in time_task_element_type_robot_axis if len(time_task_element_type_robot) == 3}
        print("task_element2type_robot: ", task_element2type_robot)
        print("task_element2time: ", task_element2time)
        executing_task_network = reduced_task_network.copy()
        

        while executing_task_network.nodes():
            # candidate tasks: 1. no parent in updated task network 2. no parent in executing tasks
            candidate_tasks = set(node for node in executing_task_network.nodes() \
                if executing_task_network.in_degree(node) == 0)
            removed_candidate_tasks = set(task for task in candidate_tasks for exec_task in current_exec_tasks \
                if (exec_task, task) in executing_task_network.edges())
            removed_candidate_tasks.update(removed_candidate_tasks)
            candidate_tasks.difference_update(removed_candidate_tasks)
            # deal with subtasks that are ignored due to or relation
            remove_task_element_from_network = set(task for task in candidate_tasks if task not in task_element2type_robot.keys())
            candidate_tasks.difference_update(remove_task_element_from_network)
            executing_task_network.remove_nodes_from(remove_task_element_from_network)
            
            for type_robot in type_robots:
                if type_robot in current_exec_robots:
                    continue
                candidate_tasks_for_robot = [task_element for task_element in candidate_tasks \
                    if task_element2type_robot[task_element] == type_robot]
                if not candidate_tasks_for_robot:
                    continue
                # if there are several candiate tasks for a robot, sort according to task plan
                ordered_candidata_tasks = [task_element2time[task_element] for task_element in candidate_tasks_for_robot]
                task_index = ordered_candidata_tasks.index(min(ordered_candidata_tasks))

                current_exec_robots.append(type_robot)
                current_exec_tasks.append(candidate_tasks_for_robot[task_index])
                
                # generate message 
                # is_human = type_robot == (1, 1)
                topic = 'hltl_msg/' + ['human','fanuc'][type_robot[0]] +['','1'][type_robot[1]]+ '_action'
                task_id = reduced_task_network.nodes[current_exec_tasks[-1]]['label'][0][0][0]

                thread.start_new_thread(send2fanuc,(topic,self.config_data['config'][self.task_id2brick_id[task_id+['human','fanuc'][type_robot[0]] +['','1'][type_robot[1]]]]["brick_name"]))
                print('_____________send_______________\n',(topic,self.config_data['config'][self.task_id2brick_id[task_id+['human','fanuc'][type_robot[0]] +['','1'][type_robot[1]]]]["brick_name"]))
                # if current_exec_robots[0][1]==1:
                #     thread.start_new_thread(send2fanuc,('/hltl_msg/fanuc1_action','b15_1'))
                # elif current_exec_robots[0][1]==0:
                #     thread.start_new_thread(send2fanuc,('/hltl_msg/fanuc_action','b15_1'))


                # print("is human {0}, task id {1}".format(is_human, task_id))
            # send message
            for idx, robot in enumerate(current_exec_robots):
                topic = 'fanuc_' + str(robot[0]) + '_' + str(robot[1])
                task = reduced_task_network.nodes[current_exec_tasks[idx]]['label'][0][0][0]
                
            # remove current subtasks
            executing_task_network.remove_nodes_from(current_exec_tasks)
            print("current_exec_robots: ", current_exec_robots)
            for task in current_exec_tasks:
                print("current_exec_tasks: ", task, reduced_task_network.nodes[task]['label'], 
                    reduced_task_network.nodes[task]['label'][0][0][0])
                # for i in

            
            # if self.finished_task_str == self.invalid_str:
            #     self.finished_task_str = input("Finished task: ")
            #     finished_task_split = self.finished_task_str.split()     
            #     self.finished_task_str = self.invalid_str
            #     # update when some tasks are finished
            #     finished_task = (finished_task_split[0], int(finished_task_split[1]))
            #     finished_tasks.append(finished_task)
            #     print("finished_task: ", finished_task)
            #     current_exec_tasks.remove(finished_task)
            #     current_exec_robots.remove(task_element2type_robot[finished_task])
                
            # (1, 1) human (1, 0) robot
            while self.finished_task_str.any() == 0:
                time.sleep(0.11)
                # self.finished_task_str = self.global_task_finished_id
                # self.global_task_finished_id=-1
                # robot_id=
                # if self.finished_task_str != 0 and self.finished_task_str != 1 :
                #     self.finished_task_str = self.invalid_str
            
            # is_human = int(self.finished_task_str) == 1
            # self.finished_task_str = self.invalid_str
            # if robot_id==1:
                # task_idx = current_exec_robots.index((1,1))
            # else:
            robot_id=np.where(self.finished_task_str==1)[0]
            task_idx = current_exec_robots.index((1,robot_id))
            self.finished_task_str[robot_id]=0
            # robot_id = get_from_topic()
            print("finished_task: ", current_exec_tasks[task_idx])
            current_exec_tasks.pop(task_idx)
            current_exec_robots.pop(task_idx)
    def test(self):

        time.sleep(3)
        parser = create_parser()
        args = parser.parse_known_args()[0]
        
        if args.task == "man":
            time_task_element_type_robot_axis, reduced_task_network, type_robots = hierarchical_ltl_planner(args=args)
            self.task_execution(time_task_element_type_robot_axis, reduced_task_network, type_robots)
        else:
            hierarchical_ltl_planner(args=args)

        #  python main.py --task man --case 5


        # def send2fanuc2(bringkname='b15_1'):
        # action_msg = rospy.ServiceProxy('/hltl_msg/fanuc1_action', lego_pickup)
        #     action_msg("23", "pick", "b15_1")
        # action1_msg = rospy.ServiceProxy('/hltl_msg/fanuc1_action', lego_pickup)
        
        
        # action_msg("23", "pick", "b16_1")  
        # # 3
        # action_msg("23", "pick", "b16_2")
        # # 2
        # action_msg("23", "pick", "b16_3")
        # # 1
        # action_msg("23", "pick", "b16_4")
        # # 4

        # action_msg("23", "pick", "b14_1")
        # # 11
        # action_msg("23", "pick", "b14_2")
        # # 7
        # action_msg("23", "pick", "b14_3")
        # # 5
        # action_msg("23", "pick", "b14_4")
        # # 9
        # action_msg("23", "pick", "b14_5")
        # # 10
        # action_msg("23", "pick", "b14_6")
        # # 6
        # action_msg("23", "pick", "b14_7")
        # # 8
        # action_msg("23", "pick", "b14_8")
        # # 12

        # action_msg("23", "pick", "b16_5")
        # # 16
        # action_msg("23", "pick", "b16_6")
        # # 15
        # action_msg("23", "pick", "b28_1")
        # # 13
        # action_msg("23", "pick", "b28_2")
        # # 14
        # action_msg("23", "pick", "b26_1")
        # # 17
        # action_msg("23", "pick", "b22_1")
        # # 18

        # action_msg("23", "pick", "b13_1")
        # action_msg("23", "pick", "b15_3")
        # action_msg("23", "pick", "b15_4")
        # action_msg("23", "pick", "b13_2")
        # action_msg("23", "pick", "b13_3")
        # action_msg("23", "pick", "b15_5")
        # action_msg("23", "pick", "b11_2")
        # action_msg("23", "pick", "b11_1")
        # action_msg("23", "pick", "b13_4")
        pass 
    def init_state(self,pose="src"):
        for id in range(self.brick_len):
            if self.config_data['config'][id]["state"]=="init":
                lego_action(self.config_data['config'][id]["brick_name"],
                            x=self.config_data['config'][id][pose]["x"],
                            y=self.config_data['config'][id][pose]["y"],
                            z=self.config_data['config'][id][pose]["z"],orientation=self.config_data['config'][id][pose]["o"])
                print(self.config_data['config'][id]["brick_name"],
                            self.config_data['config'][id][pose]["x"],
                            self.config_data['config'][id][pose]["y"],
                            self.config_data['config'][id][pose]["z"],
                            self.config_data['config'][id][pose]["o"])
                self.config_data['config'][id]["state"]=pose
    def change_state(self,brick_name,reference_frame):
        self.config_data['config'][self.name2id[brick_name]]["state"]=reference_frame
        pass 

    def lego_state_service_handler(self,req:lego_pickup):


        if req.is_pick=="pick":
            self.change_state(req.pick_lego_name,req.reference_frame)
        else:
            self.change_state(req.pick_lego_name,"des")
        return True

    def refresh_lego_state(self):
        rate = rospy.Rate(60) 
        while not rospy.is_shutdown():
            for id in range(self.brick_len):
                if self.config_data['config'][id]["state"] =='fanuc_gazebo::link_tool':
                    lego_action(brick_name=self.config_data['config'][id]["brick_name"] ,reference_frame='fanuc_gazebo::link_tool',x=0,y=0,z=0.064,orientation=0,upside_orientation=180)
                elif self.config_data['config'][id]["state"] =='fanuc1_gazebo::link_tool':
                    lego_action(brick_name=self.config_data['config'][id]["brick_name"] ,reference_frame='fanuc1_gazebo::link_tool',x=0,y=0,z=0.064,orientation=0,upside_orientation=180)
                elif self.config_data['config'][id]["state"] =='human_gazebo::LeftHand':
                    lego_action(brick_name=self.config_data['config'][id]["brick_name"],reference_frame='human_gazebo::LeftHand',y=0.175,orientation=-90)
            rate.sleep()
        pass 
    def human_HLTL_service_launch(self):
        def human_HLTL_service(req:lego_pickup):
            id=self.name2id[req.pick_lego_name]
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
            robot_id=0
            id=self.name2id[req.pick_lego_name]
            fanuc_action_reletive2world(x= self.config_data['config'][id]["src"]["x"],y=self.config_data['config'][id]["src"]["y"],z=0.35,orientation=self.config_data['config'][id]["src"]["o"])
            # move to the above of the brick
            # while(self.someBodyPlacing.any()>0):
            #     time.sleep(0.2)
            # self.someBodyPlacing[0]=1
            fanuc_action_reletive2world(x= self.config_data['config'][id]["src"]["x"],y=self.config_data['config'][id]["src"]["y"],z=0.12,orientation=self.config_data['config'][id]["src"]["o"])
            # move to the top of the brick
            self.change_state(req.pick_lego_name,'fanuc_gazebo::link_tool')
            fanuc_action_reletive2world(x= self.config_data['config'][id]["src"]["x"],y=self.config_data['config'][id]["src"]["y"],z=0.35,orientation=self.config_data['config'][id]["src"]["o"])
            # self.someBodyPlacing[0]=0
            # move to the above of the brick
            while(self.someBodyPlacing.any()>0):
                time.sleep(0.1)
            self.someBodyPlacing[robot_id]=1
            fanuc_action_reletive2world(x= self.config_data['config'][id]["des"]["x"],y=self.config_data['config'][id]["des"]["y"],z=0.35,orientation=self.config_data['config'][id]["des"]["o"])
            # move to the above of the brick des

            fanuc_action_reletive2world(x= self.config_data['config'][id]["des"]["x"],y=self.config_data['config'][id]["des"]["y"],z=0.11+self.config_data['config'][id]["des"]["z"],orientation=self.config_data['config'][id]["des"]["o"])
            self.change_state(req.pick_lego_name,'des')
            fanuc_action_reletive2world(x= self.config_data['config'][id]["des"]["x"],y=self.config_data['config'][id]["des"]["y"],z=0.35,orientation=self.config_data['config'][id]["des"]["o"])
            self.someBodyPlacing[robot_id]=0

            self.finished_task_str[robot_id]=1
            fanuc_action_reletive2world(x= self.config_data['config'][id]["src"]["x"],y=self.config_data['config'][id]["src"]["y"],z=0.35,orientation=self.config_data['config'][id]["src"]["o"])

            
            return True
            pass 
        s = rospy.Service('hltl_msg/fanuc_action', lego_pickup, fanuc_HLTL_service)
        print("hltl_msg/fanuc_action python service ready")
        rospy.spin()
        pass 
    def fanuc1_HLTL_service_launch(self):
        def fanuc_HLTL_service(req:lego_pickup):
            robot_id=1
            id=self.name2id[req.pick_lego_name]
            fanuc1_action_reletive2world(x= self.config_data['config'][id]["src"]["x"],y=self.config_data['config'][id]["src"]["y"],z=0.2,orientation=self.config_data['config'][id]["src"]["o"])
            # move to the above of the brick
            # while(self.someBodyPlacing.any()>0):
            #     time.sleep(0.2)
            # self.someBodyPlacing[1]=1
            fanuc1_action_reletive2world(x= self.config_data['config'][id]["src"]["x"],y=self.config_data['config'][id]["src"]["y"],z=0.12,orientation=self.config_data['config'][id]["src"]["o"])
            # move to the top of the brick
            self.change_state(req.pick_lego_name,'fanuc1_gazebo::link_tool')

            fanuc1_action_reletive2world(x= self.config_data['config'][id]["src"]["x"],y=self.config_data['config'][id]["src"]["y"],z=0.2,orientation=self.config_data['config'][id]["src"]["o"])
            # self.someBodyPlacing[1]=0
            # move to the above of the brick
            while(self.someBodyPlacing.any()>0):
                time.sleep(0.1)
            self.someBodyPlacing[robot_id]=1
            fanuc1_action_reletive2world(x= self.config_data['config'][id]["des"]["x"],y=self.config_data['config'][id]["des"]["y"],z=0.2,orientation=self.config_data['config'][id]["des"]["o"])
            # move to the above of the brick des

            fanuc1_action_reletive2world(x= self.config_data['config'][id]["des"]["x"],y=self.config_data['config'][id]["des"]["y"],z=0.11+self.config_data['config'][id]["des"]["z"],orientation=self.config_data['config'][id]["des"]["o"])
            self.change_state(req.pick_lego_name,'des')
            fanuc1_action_reletive2world(x= self.config_data['config'][id]["des"]["x"],y=self.config_data['config'][id]["des"]["y"],z=0.2,orientation=self.config_data['config'][id]["des"]["o"])
            self.someBodyPlacing[robot_id]=0
            # move to the top of the brick des
            self.finished_task_str[robot_id]=1
            fanuc1_action_reletive2world(x= self.config_data['config'][id]["src"]["x"],y=self.config_data['config'][id]["src"]["y"],z=0.2,orientation=self.config_data['config'][id]["src"]["o"])

            # while (self.finished_task_str!=self.invalid_str):
            #     time.sleep(0.2)
            # if self.finished_task_str==self.invalid_str:
            #     self.finished_task_str=1

            return True
            pass 
        s = rospy.Service('hltl_msg/fanuc1_action', lego_pickup, fanuc_HLTL_service)
        print("hltl_msg/fanuc1_action python service ready")
        rospy.spin()
        pass 

def lego_action(brick_name='b14_9',reference_frame='world',x=0,y=0,z=0,orientation=90,upside_orientation=0):
    # rospy.init_node('set_pose')

    state_msg = ModelState()
    state_msg.model_name = brick_name
    state_msg.reference_frame = reference_frame
    state_msg.pose.position.x = x
    state_msg.pose.position.y = y
    state_msg.pose.position.z = z
    # orientation=90
    # q = quaternion_from_euler(0 * DE2RA, 0 * DE2RA, orientation * DE2RA)
    q = Rotation.from_euler('ZYX',[upside_orientation * DE2RA, 0 * DE2RA, orientation * DE2RA]).as_quat()
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
    HLTL_sim(config_json=rospy.get_param("/lego_state_refresher/config_json"))


    # /lego_state_refresher/config_json
    # lego=lego_state(config_json="/home/xusj/Documents/0LAB/23ChangliuLiu/Hierarchical-LTL/hierarchial_simulation/src/hierarchial_simulation/config/ICL.json")