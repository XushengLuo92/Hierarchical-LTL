cmake_minimum_required(VERSION 3.10 FATAL_ERROR)
set(CMAKE_CXX_STANDARD 14)
project(stmotion_controller VERSION 1.0.0 LANGUAGES CXX)

# /* -------------------------------------------------------------------------- */
# /*                              Build Properties                              */
# /* -------------------------------------------------------------------------- */

add_compile_options(-std=c++11)
add_definitions(-D_OS_UNIX)

# /* -------------------------------------------------------------------------- */
# /*                                Find Package                                */
# /* -------------------------------------------------------------------------- */

find_package(Eigen3 3.3.7 REQUIRED)
find_package(OpenCV REQUIRED )
find_package(realsense2 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  image_transport
  cv_bridge
  message_generation
  geometry_msgs
)
## Generate services in the 'srv' folder
add_service_files(
  FILES
  robot_action.srv
  lego_pickup.srv
#   Service2.srv
)
## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs#   std_msgs
)
catkin_package(
  CATKIN_DEPENDS message_runtime
)
include_directories(
    include
    ${OpenCV_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS})

# /* -------------------------------------------------------------------------- */
# /*                                   QP lib                                   */
# /* -------------------------------------------------------------------------- */

include_directories(3rd-party/qplib)
add_subdirectory(3rd-party)

# /* -------------------------------------------------------------------------- */
# /*                               Create Library                               */
# /* -------------------------------------------------------------------------- */

file(GLOB src_files src/*.cpp
                    src/Utils/*.cpp
                    src/FTS/*.cpp)

add_library(${PROJECT_NAME} SHARED ${src_files})

target_link_libraries(${PROJECT_NAME}
    qplib
    Eigen3::Eigen
    realsense2
    ${OpenCV_LIBRARIES}
    ${catkin_LIBRARIES}
    jsoncpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

set_target_properties(${PROJECT_NAME} PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION 1)



# /* -------------------------------------------------------------------------- */
# /*                             Testing Executables                            */
# /* -------------------------------------------------------------------------- */

add_executable(${PROJECT_NAME}_single_traj
  src/Test/single_traj.cpp
)
target_link_libraries(${PROJECT_NAME}_single_traj
    ${PROJECT_NAME}
)

add_executable(${PROJECT_NAME}_lego_test
  src/Test/lego_assembly.cpp
)
target_link_libraries(${PROJECT_NAME}_lego_test
    ${PROJECT_NAME}
)

add_executable(${PROJECT_NAME}_ik_test
  src/Test/IK_test.cpp
)
target_link_libraries(${PROJECT_NAME}_ik_test
    ${PROJECT_NAME}
)


add_executable(${PROJECT_NAME}_bringup
  src/ros_nodes/controller_node.cpp
)
target_link_libraries(${PROJECT_NAME}_bringup
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
)

add_executable(${PROJECT_NAME}_realsense
  src/ros_nodes/realsense_node.cpp
)
target_link_libraries(${PROJECT_NAME}_realsense
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
)

add_executable(${PROJECT_NAME}_fts
  src/ros_nodes/fts_node.cpp
)
target_link_libraries(${PROJECT_NAME}_fts
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
)

add_executable(${PROJECT_NAME}_pick_up_test
  src/ros_nodes/lego_pick_up_test_node.cpp
)
target_link_libraries(${PROJECT_NAME}_pick_up_test
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
)


add_executable(${PROJECT_NAME}_task_planning_cartesian
  src/ros_nodes/lego_task_planning_cartesian_node.cpp
)
target_link_libraries(${PROJECT_NAME}_task_planning_cartesian
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
)

add_executable(${PROJECT_NAME}_LTL_task_planning_cartesian
  src/ros_nodes/LTL_lego_task_planning_cartesian_node.cpp
)
target_link_libraries(${PROJECT_NAME}_LTL_task_planning_cartesian
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
)
catkin_install_python(PROGRAMS simulation/py_scripts/lego_state_refresher.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})