# FANUC_Stream_Motion_Controller_CPP
This is a C++ controller for controlling the FANUC robot using the stream motion interface.

## Prerequisites
Linux (All the modules were tested on Linux Ubuntu 20.04. Non-ROS version was tested also on Linux Debian 11)

Eigen 3.3.7

[librealsense](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md)

OpenCV

## Build
1. Clone the repo to `/path/to/catkin_ws/src/`
2. `cd /path/to/catkin_ws/`
3. `catkin_make`



## Use Robot
### Setup Ethernet
1. Connect to the robot via an Ethernet cable.
2. Set PC IP address to `192.168.1.xxx`, except `192.168.1.100` (robot IP). A valid IP address could be `192.168.1.101`. 
3. Connect to the designated Ethernet.

### Run task
1. Modify `./config/user_config.json` as needed. Set `Use_Robot` to `1`.
2. Open a terminal and `cd /path/to/catkin_ws/`.
3. `source devel/setup.bash`
4. `roslaunch stmotion_controller controller_node.launch`
5. Open a terminal and `cd /path/to/catkin_ws/`.
6. `source devel/setup.bash`
7. `roslaunch stmotion_controller task_planning_cartesian_node.launch`

## Digital Twin
### Rviz
`roslaunch stmotion_controller fanuc_rviz.launch`


### Gazebo for LEGO Assembly
1. Modify `./config/user_config.json` as needed. Set `Use_Robot` to `0`.
2. Open a terminal and `cd /path/to/catkin_ws/`.
3. `source devel/setup.bash`
4. `roslaunch stmotion_controller fanuc_gazebo.launch`
5. Open a terminal and `cd /path/to/catkin_ws/`.
6. `source devel/setup.bash`
7. `roslaunch stmotion_controller controller_node.launch`
8. Open a terminal and `cd /path/to/catkin_ws/`.
9. `source devel/setup.bash`
10. `roslaunch stmotion_controller task_planning_cartesian_node.launch`


### Gazebo with Human Model
1. Modify `./config/user_config.json` as needed. Set `Use_Robot` to `0`.
2. Open a terminal and `cd /path/to/catkin_ws/`.
3. `source devel/setup.bash`
4. `roslaunch stmotion_controller human_fanuc_gazebo.launch`
5. Open a terminal and `python3 ./simulation/py_scripts/manual_controller.py` to move the human in Gazebo.