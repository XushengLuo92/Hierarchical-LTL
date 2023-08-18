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
from moveit_fanuc_config.srv import robot_action
# os.environ["ROS_NAMESPACE"] = "/fanuc_gazebo"   it seems meaningless

moveit_commander.roscpp_initialize(sys.argv)
# moveit_commander.roscpp_initialize()
rospy.init_node("move_group_python_interface", anonymous=True)
# rospy.init_node("move_group_python_interface_fanuc_gazebo", anonymous=True)
# rospy.spin()
robot_fanuc = moveit_commander.RobotCommander()
# robot_fanuc = moveit_commander.RobotCommander(robot_description="/fanuc_gazebo/robot_description",ns="/fanuc_gazebo")
# scene = moveit_commander.PlanningSceneInterface(ns="/fanuc_gazebo")
scene = moveit_commander.PlanningSceneInterface()



display_trajectory_publisher = rospy.Publisher(
    "/fanuc_gazebo/move_group/display_planned_path",
    moveit_msgs.msg.DisplayTrajectory,
    queue_size=20,
)
# rospy.spin()
print(robot_fanuc.get_current_state())
def robot_action_service_handler(req:robot_action):
    # print('fanuc_gazebo arm start state',robot_fanuc.get_current_state())
    group_name = "arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)
    move_group.set_pose_reference_frame("fanuc_gazebo::base")
    des=req.des
    end_effector_link = move_group.get_end_effector_link()
    move_group.set_goal_position_tolerance(0.0001)
    move_group.set_goal_orientation_tolerance(0.0001)
    move_group.set_goal_joint_tolerance(0.0001)
    move_group.set_goal_tolerance(0.0001)
    move_group.set_start_state_to_current_state()
    move_group.set_planner_id('TRRT')
    # move_group.set_pose_target(des)

    move_group.set_pose_target(des, end_effector_link)
    # `go()` returns a boolean indicating whether the planning and execution was successful.
    success = move_group.go(wait=True)
    # print('fanuc_gazebo arm stop state',robot_fanuc.get_current_state())
    # Calling `stop()` ensures that there is no residual movement
    move_group.stop()
    move_group.clear_pose_targets()
    return success

group_name = "arm"
move_group = moveit_commander.MoveGroupCommander(group_name)
move_group.set_named_target('main')
move_group.go()  #让机械臂先规划，再运动，阻塞指令，直到机械臂到达home后再向下执行
rospy.sleep(1)
s = rospy.Service('action_msg', robot_action, robot_action_service_handler)
print("fanuc python service ready")
rospy.spin()


