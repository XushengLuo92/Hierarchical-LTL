cmake_minimum_required(VERSION 3.1.3)
project(moveit_human_config)

find_package(catkin REQUIRED
            geometry_msgs
            roscpp
            rospy
            std_msgs
            message_generation)
## Generate services in the 'srv' folder
add_service_files(
  FILES
  robot_action.srv
#   Service2.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs#   std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES motion_info
#  CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs
CATKIN_DEPENDS message_runtime
)
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
catkin_install_python(PROGRAMS scripts/human_controller.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})