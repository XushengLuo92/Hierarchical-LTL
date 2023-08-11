# Python 2/3 compatibility imports
from __future__ import print_function
from six.moves import input
import os 
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from moveit_human_config.srv import robot_action
# os.environ["ROS_NAMESPACE"] = "/human_gazebo"   it seems meaningless

moveit_commander.roscpp_initialize(sys.argv)
# moveit_commander.roscpp_initialize()
rospy.init_node("move_group_python_interface", anonymous=True)
# rospy.init_node("move_group_python_interface_human_gazebo", anonymous=True)
# rospy.spin()
robot_human = moveit_commander.RobotCommander()
# robot_human = moveit_commander.RobotCommander(robot_description="/human_gazebo/robot_description",ns="/human_gazebo")
# scene = moveit_commander.PlanningSceneInterface(ns="/human_gazebo")
scene = moveit_commander.PlanningSceneInterface()



display_trajectory_publisher = rospy.Publisher(
    "/human_gazebo/move_group/display_planned_path",
    moveit_msgs.msg.DisplayTrajectory,
    queue_size=20,
)
# rospy.spin()
print(robot_human.get_current_state())
def robot_action_service_handler_right_arm(req:robot_action):
    print('human_gazebo right_arm start state',robot_human.get_current_state())
    group_name = "right_arm"
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
    print('human_gazebo right_arm stop state',robot_human.get_current_state())
    # Calling `stop()` ensures that there is no residual movement
    move_group.stop()
    move_group.clear_pose_targets()
    return success
def robot_action_service_handler_left_arm(req:robot_action):
    print('human_gazebo left_arm start state',robot_human.get_current_state())
    group_name = "left_arm"
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
    print('human_gazebo left_arm stop state',robot_human.get_current_state())
    # Calling `stop()` ensures that there is no residual movement
    move_group.stop()
    move_group.clear_pose_targets()
    return success
s1 = rospy.Service('action_msg/right_arm', robot_action, robot_action_service_handler_right_arm)
s2 = rospy.Service('action_msg/left_arm', robot_action, robot_action_service_handler_left_arm)
print("human python service ready")
rospy.spin()


