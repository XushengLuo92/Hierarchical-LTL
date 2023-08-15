import rospy 
from stmotion_controller.srv import robot_action,lego_pickup
import geometry_msgs.msg
import math
from tf.transformations import quaternion_from_euler
import gazebo_msgs.msg 
DE2RA = math.pi / 180
from gazebo_msgs.srv import SetModelState,GetLinkState,GetModelState
import json
class lego_state():
    def __init__(self,config_json=None) -> None:
        pass
        if config_json!=None:
            with open(config_json,'r') as config_file:
                config_data=json.load(config_file)
                
    def refresh_state(self):
        pass 
    def change_state(self):
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
        
def lego_state_service_handler(req:robot_action):
    rospy.wait_for_service('/fanuc_gazebo/action_msg')
    print('fanuc_gazebo arm start state',robot_fanuc.get_current_state())
    group_name = "arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    des=req.des
    # pose_goal = geometry_msgs.msg.PoseStamped()
    # pose_goal.header.frame_id = "world"
    # pose_goal.pose.orientation.x = 1e-6
    # pose_goal.pose.orientation.y = 1e-6
    # pose_goal.pose.orientation.z = 1e-6
    # pose_goal.pose.orientation.w = -1.000000
    # pose_goal.pose.position.x = 0.5
    # pose_goal.pose.position.y = 0
    # pose_goal.pose.position.z = 0.16+0.015

    move_group.set_pose_target(des)

    # `go()` returns a boolean indicating whether the planning and execution was successful.
    success = move_group.go(wait=True)
    print('fanuc_gazebo arm stop state',robot_fanuc.get_current_state())
    # Calling `stop()` ensures that there is no residual movement
    move_group.stop()
    move_group.clear_pose_targets()
    return success

s = rospy.Service('action_msg/lego_state', lego_pickup, lego_state_service_handler)
print("lego python service ready")
rospy.spin()


if __name__=='__main__':
    lego_state(config_json="/home/xusj/Documents/0LAB/23ChangliuLiu/Hierarchical-LTL/hierarchial_simulation/src/hierarchial_simulation/config/ICL.json")
    lego_action(brick_name=brink_name,reference_frame='fanuc_gazebo::link_tool',y=0.17,orientation=-90)
    lego_action(brick_name=brink_name,reference_frame='human_gazebo::LeftHand',y=0.17,orientation=-90)
    # print(fanuc_action())
    # print(human_right_action())