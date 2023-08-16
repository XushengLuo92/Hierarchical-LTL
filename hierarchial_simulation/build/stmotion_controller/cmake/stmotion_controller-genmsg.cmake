# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "stmotion_controller: 0 messages, 2 services")

set(MSG_I_FLAGS "-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(stmotion_controller_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/xusj/Documents/0LAB/23ChangliuLiu/Hierarchical-LTL/hierarchial_simulation/src/stmotion_controller/srv/robot_action.srv" NAME_WE)
add_custom_target(_stmotion_controller_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "stmotion_controller" "/home/xusj/Documents/0LAB/23ChangliuLiu/Hierarchical-LTL/hierarchial_simulation/src/stmotion_controller/srv/robot_action.srv" "geometry_msgs/Quaternion:geometry_msgs/PoseStamped:geometry_msgs/Pose:std_msgs/Header:geometry_msgs/Point"
)

get_filename_component(_filename "/home/xusj/Documents/0LAB/23ChangliuLiu/Hierarchical-LTL/hierarchial_simulation/src/stmotion_controller/srv/lego_pickup.srv" NAME_WE)
add_custom_target(_stmotion_controller_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "stmotion_controller" "/home/xusj/Documents/0LAB/23ChangliuLiu/Hierarchical-LTL/hierarchial_simulation/src/stmotion_controller/srv/lego_pickup.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(stmotion_controller
  "/home/xusj/Documents/0LAB/23ChangliuLiu/Hierarchical-LTL/hierarchial_simulation/src/stmotion_controller/srv/robot_action.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/stmotion_controller
)
_generate_srv_cpp(stmotion_controller
  "/home/xusj/Documents/0LAB/23ChangliuLiu/Hierarchical-LTL/hierarchial_simulation/src/stmotion_controller/srv/lego_pickup.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/stmotion_controller
)

### Generating Module File
_generate_module_cpp(stmotion_controller
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/stmotion_controller
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(stmotion_controller_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(stmotion_controller_generate_messages stmotion_controller_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/xusj/Documents/0LAB/23ChangliuLiu/Hierarchical-LTL/hierarchial_simulation/src/stmotion_controller/srv/robot_action.srv" NAME_WE)
add_dependencies(stmotion_controller_generate_messages_cpp _stmotion_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xusj/Documents/0LAB/23ChangliuLiu/Hierarchical-LTL/hierarchial_simulation/src/stmotion_controller/srv/lego_pickup.srv" NAME_WE)
add_dependencies(stmotion_controller_generate_messages_cpp _stmotion_controller_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(stmotion_controller_gencpp)
add_dependencies(stmotion_controller_gencpp stmotion_controller_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS stmotion_controller_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages

### Generating Services
_generate_srv_eus(stmotion_controller
  "/home/xusj/Documents/0LAB/23ChangliuLiu/Hierarchical-LTL/hierarchial_simulation/src/stmotion_controller/srv/robot_action.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/stmotion_controller
)
_generate_srv_eus(stmotion_controller
  "/home/xusj/Documents/0LAB/23ChangliuLiu/Hierarchical-LTL/hierarchial_simulation/src/stmotion_controller/srv/lego_pickup.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/stmotion_controller
)

### Generating Module File
_generate_module_eus(stmotion_controller
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/stmotion_controller
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(stmotion_controller_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(stmotion_controller_generate_messages stmotion_controller_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/xusj/Documents/0LAB/23ChangliuLiu/Hierarchical-LTL/hierarchial_simulation/src/stmotion_controller/srv/robot_action.srv" NAME_WE)
add_dependencies(stmotion_controller_generate_messages_eus _stmotion_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xusj/Documents/0LAB/23ChangliuLiu/Hierarchical-LTL/hierarchial_simulation/src/stmotion_controller/srv/lego_pickup.srv" NAME_WE)
add_dependencies(stmotion_controller_generate_messages_eus _stmotion_controller_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(stmotion_controller_geneus)
add_dependencies(stmotion_controller_geneus stmotion_controller_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS stmotion_controller_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(stmotion_controller
  "/home/xusj/Documents/0LAB/23ChangliuLiu/Hierarchical-LTL/hierarchial_simulation/src/stmotion_controller/srv/robot_action.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/stmotion_controller
)
_generate_srv_lisp(stmotion_controller
  "/home/xusj/Documents/0LAB/23ChangliuLiu/Hierarchical-LTL/hierarchial_simulation/src/stmotion_controller/srv/lego_pickup.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/stmotion_controller
)

### Generating Module File
_generate_module_lisp(stmotion_controller
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/stmotion_controller
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(stmotion_controller_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(stmotion_controller_generate_messages stmotion_controller_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/xusj/Documents/0LAB/23ChangliuLiu/Hierarchical-LTL/hierarchial_simulation/src/stmotion_controller/srv/robot_action.srv" NAME_WE)
add_dependencies(stmotion_controller_generate_messages_lisp _stmotion_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xusj/Documents/0LAB/23ChangliuLiu/Hierarchical-LTL/hierarchial_simulation/src/stmotion_controller/srv/lego_pickup.srv" NAME_WE)
add_dependencies(stmotion_controller_generate_messages_lisp _stmotion_controller_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(stmotion_controller_genlisp)
add_dependencies(stmotion_controller_genlisp stmotion_controller_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS stmotion_controller_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages

### Generating Services
_generate_srv_nodejs(stmotion_controller
  "/home/xusj/Documents/0LAB/23ChangliuLiu/Hierarchical-LTL/hierarchial_simulation/src/stmotion_controller/srv/robot_action.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/stmotion_controller
)
_generate_srv_nodejs(stmotion_controller
  "/home/xusj/Documents/0LAB/23ChangliuLiu/Hierarchical-LTL/hierarchial_simulation/src/stmotion_controller/srv/lego_pickup.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/stmotion_controller
)

### Generating Module File
_generate_module_nodejs(stmotion_controller
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/stmotion_controller
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(stmotion_controller_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(stmotion_controller_generate_messages stmotion_controller_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/xusj/Documents/0LAB/23ChangliuLiu/Hierarchical-LTL/hierarchial_simulation/src/stmotion_controller/srv/robot_action.srv" NAME_WE)
add_dependencies(stmotion_controller_generate_messages_nodejs _stmotion_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xusj/Documents/0LAB/23ChangliuLiu/Hierarchical-LTL/hierarchial_simulation/src/stmotion_controller/srv/lego_pickup.srv" NAME_WE)
add_dependencies(stmotion_controller_generate_messages_nodejs _stmotion_controller_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(stmotion_controller_gennodejs)
add_dependencies(stmotion_controller_gennodejs stmotion_controller_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS stmotion_controller_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(stmotion_controller
  "/home/xusj/Documents/0LAB/23ChangliuLiu/Hierarchical-LTL/hierarchial_simulation/src/stmotion_controller/srv/robot_action.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/stmotion_controller
)
_generate_srv_py(stmotion_controller
  "/home/xusj/Documents/0LAB/23ChangliuLiu/Hierarchical-LTL/hierarchial_simulation/src/stmotion_controller/srv/lego_pickup.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/stmotion_controller
)

### Generating Module File
_generate_module_py(stmotion_controller
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/stmotion_controller
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(stmotion_controller_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(stmotion_controller_generate_messages stmotion_controller_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/xusj/Documents/0LAB/23ChangliuLiu/Hierarchical-LTL/hierarchial_simulation/src/stmotion_controller/srv/robot_action.srv" NAME_WE)
add_dependencies(stmotion_controller_generate_messages_py _stmotion_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xusj/Documents/0LAB/23ChangliuLiu/Hierarchical-LTL/hierarchial_simulation/src/stmotion_controller/srv/lego_pickup.srv" NAME_WE)
add_dependencies(stmotion_controller_generate_messages_py _stmotion_controller_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(stmotion_controller_genpy)
add_dependencies(stmotion_controller_genpy stmotion_controller_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS stmotion_controller_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/stmotion_controller)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/stmotion_controller
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(stmotion_controller_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/stmotion_controller)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/stmotion_controller
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(stmotion_controller_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/stmotion_controller)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/stmotion_controller
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(stmotion_controller_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/stmotion_controller)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/stmotion_controller
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(stmotion_controller_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/stmotion_controller)
  install(CODE "execute_process(COMMAND \"/usr/local/miniconda3/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/stmotion_controller\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/stmotion_controller
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(stmotion_controller_generate_messages_py geometry_msgs_generate_messages_py)
endif()
