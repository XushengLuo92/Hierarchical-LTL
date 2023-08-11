import rospy 
from stmotion_controller.srv import robot_action
import geometry_msgs.msg
import math
from tf.transformations import quaternion_from_euler
import gazebo_msgs.msg 
DE2RA = math.pi / 180

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

s = rospy.Service('action_msg', gazebo_msgs, lego_state_service_handler)
print("fanuc python service ready")
rospy.spin()


if __name__=='__main__':
    print(fanuc_action())
    # print(human_right_action())