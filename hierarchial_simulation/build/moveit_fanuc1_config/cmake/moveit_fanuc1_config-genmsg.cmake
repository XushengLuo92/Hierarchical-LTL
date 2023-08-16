# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "moveit_fanuc1_config: 0 messages, 1 services")

set(MSG_I_FLAGS "-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(moveit_fanuc1_config_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/xusj/Documents/0LAB/23ChangliuLiu/Hierarchical-LTL/hierarchial_simulation/src/moveit_fanuc1_config/srv/robot_action.srv" NAME_WE)
add_custom_target(_moveit_fanuc1_config_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "moveit_fanuc1_config" "/home/xusj/Documents/0LAB/23ChangliuLiu/Hierarchical-LTL/hierarchial_simulation/src/moveit_fanuc1_config/srv/robot_action.srv" "geometry_msgs/Pose:geometry_msgs/PoseStamped:geometry_msgs/Quaternion:geometry_msgs/Point:std_msgs/Header"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(moveit_fanuc1_config
  "/home/xusj/Documents/0LAB/23ChangliuLiu/Hierarchical-LTL/hierarchial_simulation/src/moveit_fanuc1_config/srv/robot_action.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/moveit_fanuc1_config
)

### Generating Module File
_generate_module_cpp(moveit_fanuc1_config
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/moveit_fanuc1_config
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(moveit_fanuc1_config_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(moveit_fanuc1_config_generate_messages moveit_fanuc1_config_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/xusj/Documents/0LAB/23ChangliuLiu/Hierarchical-LTL/hierarchial_simulation/src/moveit_fanuc1_config/srv/robot_action.srv" NAME_WE)
add_dependencies(moveit_fanuc1_config_generate_messages_cpp _moveit_fanuc1_config_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(moveit_fanuc1_config_gencpp)
add_dependencies(moveit_fanuc1_config_gencpp moveit_fanuc1_config_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS moveit_fanuc1_config_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages

### Generating Services
_generate_srv_eus(moveit_fanuc1_config
  "/home/xusj/Documents/0LAB/23ChangliuLiu/Hierarchical-LTL/hierarchial_simulation/src/moveit_fanuc1_config/srv/robot_action.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/moveit_fanuc1_config
)

### Generating Module File
_generate_module_eus(moveit_fanuc1_config
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/moveit_fanuc1_config
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(moveit_fanuc1_config_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(moveit_fanuc1_config_generate_messages moveit_fanuc1_config_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/xusj/Documents/0LAB/23ChangliuLiu/Hierarchical-LTL/hierarchial_simulation/src/moveit_fanuc1_config/srv/robot_action.srv" NAME_WE)
add_dependencies(moveit_fanuc1_config_generate_messages_eus _moveit_fanuc1_config_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(moveit_fanuc1_config_geneus)
add_dependencies(moveit_fanuc1_config_geneus moveit_fanuc1_config_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS moveit_fanuc1_config_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(moveit_fanuc1_config
  "/home/xusj/Documents/0LAB/23ChangliuLiu/Hierarchical-LTL/hierarchial_simulation/src/moveit_fanuc1_config/srv/robot_action.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/moveit_fanuc1_config
)

### Generating Module File
_generate_module_lisp(moveit_fanuc1_config
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/moveit_fanuc1_config
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(moveit_fanuc1_config_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(moveit_fanuc1_config_generate_messages moveit_fanuc1_config_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/xusj/Documents/0LAB/23ChangliuLiu/Hierarchical-LTL/hierarchial_simulation/src/moveit_fanuc1_config/srv/robot_action.srv" NAME_WE)
add_dependencies(moveit_fanuc1_config_generate_messages_lisp _moveit_fanuc1_config_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(moveit_fanuc1_config_genlisp)
add_dependencies(moveit_fanuc1_config_genlisp moveit_fanuc1_config_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS moveit_fanuc1_config_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages

### Generating Services
_generate_srv_nodejs(moveit_fanuc1_config
  "/home/xusj/Documents/0LAB/23ChangliuLiu/Hierarchical-LTL/hierarchial_simulation/src/moveit_fanuc1_config/srv/robot_action.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/moveit_fanuc1_config
)

### Generating Module File
_generate_module_nodejs(moveit_fanuc1_config
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/moveit_fanuc1_config
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(moveit_fanuc1_config_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(moveit_fanuc1_config_generate_messages moveit_fanuc1_config_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/xusj/Documents/0LAB/23ChangliuLiu/Hierarchical-LTL/hierarchial_simulation/src/moveit_fanuc1_config/srv/robot_action.srv" NAME_WE)
add_dependencies(moveit_fanuc1_config_generate_messages_nodejs _moveit_fanuc1_config_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(moveit_fanuc1_config_gennodejs)
add_dependencies(moveit_fanuc1_config_gennodejs moveit_fanuc1_config_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS moveit_fanuc1_config_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(moveit_fanuc1_config
  "/home/xusj/Documents/0LAB/23ChangliuLiu/Hierarchical-LTL/hierarchial_simulation/src/moveit_fanuc1_config/srv/robot_action.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/moveit_fanuc1_config
)

### Generating Module File
_generate_module_py(moveit_fanuc1_config
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/moveit_fanuc1_config
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(moveit_fanuc1_config_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(moveit_fanuc1_config_generate_messages moveit_fanuc1_config_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/xusj/Documents/0LAB/23ChangliuLiu/Hierarchical-LTL/hierarchial_simulation/src/moveit_fanuc1_config/srv/robot_action.srv" NAME_WE)
add_dependencies(moveit_fanuc1_config_generate_messages_py _moveit_fanuc1_config_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(moveit_fanuc1_config_genpy)
add_dependencies(moveit_fanuc1_config_genpy moveit_fanuc1_config_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS moveit_fanuc1_config_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/moveit_fanuc1_config)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/moveit_fanuc1_config
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(moveit_fanuc1_config_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/moveit_fanuc1_config)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/moveit_fanuc1_config
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(moveit_fanuc1_config_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/moveit_fanuc1_config)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/moveit_fanuc1_config
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(moveit_fanuc1_config_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/moveit_fanuc1_config)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/moveit_fanuc1_config
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(moveit_fanuc1_config_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/moveit_fanuc1_config)
  install(CODE "execute_process(COMMAND \"/usr/local/miniconda3/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/moveit_fanuc1_config\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/moveit_fanuc1_config
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(moveit_fanuc1_config_generate_messages_py geometry_msgs_generate_messages_py)
endif()
