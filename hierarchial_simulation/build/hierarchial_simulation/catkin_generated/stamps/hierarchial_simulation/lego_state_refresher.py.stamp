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
from human_fanuc_controller import human_left_hand_action_reletive2world,fanuc_action_reletive2world
import json
import _thread as thread 
class lego_state():
    def __init__(self,config_json=None) -> None:
        pass
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
        self.init_state()
        # s = rospy.Service('action_msg/lego_state', lego_pickup, self.lego_state_service_handler)
        print("lego python service ready")
        thread.start_new_thread(self.refresh_lego_state)
        thread.start_new_thread(self.human_HLTL_service_launch)
        thread.start_new_thread(self.fanuc_HLTL_service_launch)
        # rospy.spin()
    def init_state(self):
        for id in range(self.brick_len):
            if self.config_data['config'][id]["state"]=="init":
                lego_action(self.config_data['config'][id]["brick_name"],x=self.config_data['config'[id]["des"]["x"]],y=self.config_data['config'[id]["des"]["y"]],z=self.config_data['config'[id]["des"]["z"]],orientation=self.config_data['config'[id]["des"]["o"]])
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
            for i in range(self.brick_len):
                if self.config_data['config'][id]["state"] =='fanuc_gazebo::link_tool':
                    lego_action(brick_name=self.config_data['config'][id]["brink_name"] ,reference_frame='fanuc_gazebo::link_tool',y=0.17,orientation=-90)
                elif self.config_data['config'][id]["state"] =='human_gazebo::LeftHand':
                    lego_action(brick_name=self.config_data['config'][id]["brink_name"],reference_frame='human_gazebo::LeftHand',y=0.17,orientation=-90)
            rate.sleep()
        pass 
    def human_HLTL_service_launch(self):
        def human_HLTL_service(req:lego_pickup):
            id=self.name2id[req.pick_lego_name]
            human_left_hand_action_reletive2world(x= self.config_data['config'][id]["src"]["x"],y=self.config_data['config'][id]["src"]["x"],z=0.2,orientation=self.config_data['config'][id]["src"]["o"])
            # move to the above of the brick
            human_left_hand_action_reletive2world(x= self.config_data['config'][id]["src"]["x"],y=self.config_data['config'][id]["src"]["x"],z=0.17,orientation=self.config_data['config'][id]["src"]["o"])
            # move to the top of the brick
            self.change_state(req.pick_lego_name,'human_gazebo::LeftHand')
            human_left_hand_action_reletive2world(x= self.config_data['config'][id]["src"]["x"],y=self.config_data['config'][id]["src"]["x"],z=0.2,orientation=self.config_data['config'][id]["src"]["o"])
            # move to the above of the brick
            human_left_hand_action_reletive2world(x= self.config_data['config'][id]["des"]["x"],y=self.config_data['config'][id]["des"]["x"],z=0.2,orientation=self.config_data['config'][id]["des"]["o"])
            # move to the above of the brick des
            human_left_hand_action_reletive2world(x= self.config_data['config'][id]["des"]["x"],y=self.config_data['config'][id]["des"]["x"],z=0.17,orientation=self.config_data['config'][id]["des"]["o"])
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
            fanuc_action_reletive2world(x= self.config_data['config'][id]["src"]["x"],y=self.config_data['config'][id]["src"]["x"],z=0.2,orientation=self.config_data['config'][id]["src"]["o"])
            # move to the above of the brick
            fanuc_action_reletive2world(x= self.config_data['config'][id]["src"]["x"],y=self.config_data['config'][id]["src"]["x"],z=0.17,orientation=self.config_data['config'][id]["src"]["o"])
            # move to the top of the brick
            self.change_state(req.pick_lego_name,'fanuc_gazebo::link_tool')
            fanuc_action_reletive2world(x= self.config_data['config'][id]["src"]["x"],y=self.config_data['config'][id]["src"]["x"],z=0.2,orientation=self.config_data['config'][id]["src"]["o"])
            # move to the above of the brick
            fanuc_action_reletive2world(x= self.config_data['config'][id]["des"]["x"],y=self.config_data['config'][id]["des"]["x"],z=0.2,orientation=self.config_data['config'][id]["des"]["o"])
            # move to the above of the brick des
            fanuc_action_reletive2world(x= self.config_data['config'][id]["des"]["x"],y=self.config_data['config'][id]["des"]["x"],z=0.17,orientation=self.config_data['config'][id]["des"]["o"])
            # move to the top of the brick des
            return True
            pass 
        s = rospy.Service('hltl_msg/fanuc_action', lego_pickup, fanuc_action_reletive2world)
        print("hltl_msg/fanuc_action python service ready")
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
    lego_state(config_json=rospy.get_param("/lego_gazebo/hierarchial_simulation/config_json"))



    # lego=lego_state(config_json="/home/xusj/Documents/0LAB/23ChangliuLiu/Hierarchical-LTL/hierarchial_simulation/src/hierarchial_simulation/config/ICL.json")