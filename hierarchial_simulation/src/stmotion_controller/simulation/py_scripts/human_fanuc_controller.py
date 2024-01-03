import rospy 
from stmotion_controller.srv import robot_action
import geometry_msgs.msg
import math
from tf.transformations import quaternion_from_euler
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SetModelState
DE2RA = math.pi / 180
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
    state_msg.pose.orientation.x = q[0]
    state_msg.pose.orientation.y = q[1]
    state_msg.pose.orientation.z = q[2]
    state_msg.pose.orientation.w = q[3]

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
        pose_goal.pose.orientation.x = q[0]
        pose_goal.pose.orientation.y = q[1]
        pose_goal.pose.orientation.z = q[2]
        pose_goal.pose.orientation.w = q[3]
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
        pose_goal.pose.orientation.x = q[0]
        pose_goal.pose.orientation.y = q[1]
        pose_goal.pose.orientation.z = q[2]
        pose_goal.pose.orientation.w = q[3]
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


def human_left_action():
    rospy.wait_for_service('/human_gazebo/action_msg/left_arm')
    try:
        action_msg = rospy.ServiceProxy('/human_gazebo/action_msg/left_arm', robot_action)
        
        pose_goal = geometry_msgs.msg.PoseStamped()
        pose_goal.header.frame_id = "world"
        pose_goal.pose.orientation.x = 1e-6
        pose_goal.pose.orientation.y = 1e-6
        pose_goal.pose.orientation.z = 1e-6
        pose_goal.pose.orientation.w = 1.000000
        pose_goal.pose.position.x = 0.5
        pose_goal.pose.position.y = 0
        pose_goal.pose.position.z = 0.16+0.015

        res = action_msg(pose_goal, pose_goal)
        return res.finished
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


if __name__=='__main__':
    # print(lego_action())
    # print(fanuc_action())
    print(human_left_action())